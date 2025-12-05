/**
 * @file arm_controller.cpp
 * @brief Custom PID controller for the LDR Humanoid Arm
 *
 * ARCHITECTURE OVERVIEW:
 * ======================
 * This controller implements effort-based PID control for a 6-DOF robotic arm.
 * It reads desired positions (from topics or action server) and computes torque/effort
 * commands using PID control to achieve those positions.
 *
 * CONTROL FLOW:
 * =============
 * 1. INPUT: Position commands arrive via:
 *    a) Topic: /arm_controller/commands (Float64MultiArray)
 *    b) Action: /arm_controller/follow_joint_trajectory (for MoveIt/MTC)
 *
 * 2. STORAGE: Desired positions stored internally in position_commands_ vector
 *    CRITICAL: We do NOT write to position command interfaces (that triggers Gazebo's
 *              internal position controller). We only store internally!
 *
 * 3. PID COMPUTATION: update() loop (100 Hz):
 *    - Read current position and velocity from state interfaces
 *    - Compare desired vs current position → error
 *    - Compute PID: effort = Kp*error + Ki*integral + Kd*(-velocity)
 *    - Note: We use -velocity for derivative term (better noise rejection)
 *
 * 4. OUTPUT: Write computed effort to effort command interface
 *    - Gazebo receives effort values and moves joints using physics simulation
 *    - This is realistic simulation of real hardware (motor torque control)
 *
 * 5. FEEDBACK: State interfaces provide current position/velocity back to controller
 *
 * WHY EFFORT CONTROL (NOT POSITION)?
 * ===================================
 * - Position interface → Gazebo uses built-in PD controller (can't tune)
 * - Effort interface → We compute torque, Gazebo simulates physics (realistic)
 * - Allows PID tuning in simulation before deploying to real hardware
 * - Prepares for real motor controllers that accept torque/current commands
 *
 * KEY DESIGN DECISIONS:
 * =====================
 * 1. Only claim EFFORT command interfaces (not position)
 *    → Prevents Gazebo's position controller from activating
 *
 * 2. Store desired positions internally (position_commands_ vector)
 *    → Don't write to position interfaces (would conflict with Gazebo)
 *
 * 3. Use -velocity for derivative term
 *    → Better noise rejection than numerical differentiation of error
 *
 * 4. Low Kd values (1-2) for Gazebo simulation
 *    → Velocity feedback can be noisy in simulation
 *    → High Kd amplifies noise → unstable (we learned this the hard way!)
 *
 * PID TUNING NOTES:
 * =================
 * Current gains (after extensive tuning):
 * - Kp: [100, 100, 80, 80, 60, 60] - Moderate proportional response
 * - Ki: [0, 0, 0, 0, 0, 0] - Disabled (not needed for position holding)
 * - Kd: [2, 2, 1.5, 1.5, 1, 1] - Low damping (simulation has noisy velocity)
 *
 * Tuning history (what we learned):
 * - Started with Kp=50, Kd=5 → worked but position interface conflict
 * - Removed position interface → arm moved!
 * - Tried Kp=30, Kd=15 → oscillation
 * - Tried Kp=15, Kd=30 → HUGE oscillation (475 Nm effort spikes!)
 * - Root cause: High Kd + noisy velocity = disaster
 * - Final: Kp=100, Kd=2 → stable and responsive
 */

#include "arm_control/arm_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace arm_control
{

//=============================================================================
// LIFECYCLE: Constructor
//=============================================================================
ArmController::ArmController()
: controller_interface::ControllerInterface()
{
  // Controller starts in unconfigured state
  // Actual initialization happens in on_init() and on_configure()
}

//=============================================================================
// LIFECYCLE: Initialization
// Called once when controller is first loaded
//=============================================================================
controller_interface::CallbackReturn ArmController::on_init()
{
  try {
    // Declare ROS parameters that will be loaded from YAML config file
    // These are declared here so ros2_control knows to look for them
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::vector<double>>("pid.kp", std::vector<double>());
    auto_declare<std::vector<double>>("pid.ki", std::vector<double>());
    auto_declare<std::vector<double>>("pid.kd", std::vector<double>());
    auto_declare<std::vector<double>>("effort_limits.rated", std::vector<double>());
    auto_declare<std::vector<double>>("effort_limits.max", std::vector<double>());
    auto_declare<double>("effort_limits.peak_duration", 2.0);
    auto_declare<std::vector<double>>("velocity_limits.rated", std::vector<double>());
    auto_declare<std::vector<double>>("velocity_limits.max", std::vector<double>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  //===========================================================================
  // STEP 1: Load joint names from YAML configuration
  //===========================================================================
  // Example from controllers.yaml:
  //   joints: [left_shoulder_pitch_rs04, left_shoulder_roll_rs04, ...]
  joint_names_ = get_node()->get_parameter("joints").as_string_array();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(logger, "No joints specified in 'joints' parameter!");
    return controller_interface::CallbackReturn::ERROR;
  }

  const size_t num_joints = joint_names_.size();
  RCLCPP_INFO(logger, "Configuring controller for %zu joints", num_joints);

  // Resize all internal storage vectors to match number of joints
  pid_gains_.resize(num_joints);        // PID parameters (Kp, Ki, Kd) per joint
  pid_states_.resize(num_joints);       // PID state (integral, last_error) per joint
  effort_limits_.resize(num_joints);    // Effort limits (rated, max) per joint
  velocity_limits_.resize(num_joints);  // Velocity limits (rated, max) per joint
  thermal_states_.resize(num_joints);   // Thermal tracking per joint
  position_commands_.resize(num_joints, 0.0);      // Desired positions (from topic/action)
  last_position_commands_.resize(num_joints, 0.0); // Previous commands for change detection

  //===========================================================================
  // STEP 2: Create topic subscriber for simple position commands
  //===========================================================================
  // Users can publish to: /arm_controller/commands (Float64MultiArray)
  // Example: ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.5, 0.0, 0.0]"
  //
  // This provides a simple interface for testing and basic control without MoveIt
  //
  // THREAD SAFETY: We use realtime_tools::RealtimeBuffer to safely pass data
  // from subscriber callback (non-RT thread) → update() loop (RT thread)
  command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands",
    rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      RCLCPP_INFO(get_node()->get_logger(), "Received command via topic!");
      if (msg->data.size() == joint_names_.size()) {
        // Write to realtime buffer (thread-safe, lock-free)
        position_command_buffer_.writeFromNonRT(std::vector<double>(msg->data.begin(), msg->data.end()));
        RCLCPP_INFO(get_node()->get_logger(),
          "Command written to buffer: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
          msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5]);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(), 1000,
          "Received command with %zu values, expected %zu",
          msg->data.size(), joint_names_.size());
      }
    }
  );

  RCLCPP_INFO(logger, "Command subscriber created on ~/commands");

  //===========================================================================
  // STEP 3: Create FollowJointTrajectory action server (MoveIt integration)
  //===========================================================================
  // MoveIt/MTC sends trajectories to: /arm_controller/follow_joint_trajectory
  // This is the standard ROS 2 control interface for trajectory execution
  //
  // Action flow:
  // 1. MoveIt plans a trajectory (sequence of waypoints with timestamps)
  // 2. MoveIt sends trajectory as action goal
  // 3. handle_goal() validates the trajectory
  // 4. handle_accepted() starts execution
  // 5. update() loop samples trajectory at current time
  // 6. PID controller tracks the sampled positions
  // 7. Action completes when trajectory finishes
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    get_node(),
    "~/follow_joint_trajectory",
    std::bind(&ArmController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&ArmController::handle_cancel, this, std::placeholders::_1),
    std::bind(&ArmController::handle_accepted, this, std::placeholders::_1)
  );

  RCLCPP_INFO(logger, "FollowJointTrajectory action server created on ~/follow_joint_trajectory");

  //===========================================================================
  // STEP 4: Load PID gains from YAML configuration
  //===========================================================================
  // Example from controllers.yaml:
  //   pid:
  //     kp: [100.0, 100.0, 80.0, 80.0, 60.0, 60.0]  # One per joint
  //     ki: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  //     kd: [2.0, 2.0, 1.5, 1.5, 1.0, 1.0]
  //
  // TUNING LESSONS LEARNED (from debugging session):
  // - Gazebo simulation has noisy velocity feedback
  // - High Kd (>10) amplifies noise → instability
  // - Low Kd (1-2) + moderate Kp (80-100) → stable
  // - Ki=0 sufficient for position holding
  std::vector<double> kp_values, ki_values, kd_values;
  std::vector<double> rated_effort_values, max_effort_values;
  std::vector<double> rated_velocity_values, max_velocity_values;
  double peak_duration = 2.0;  // Default 2 seconds

  try {
    kp_values = get_node()->get_parameter("pid.kp").as_double_array();
    ki_values = get_node()->get_parameter("pid.ki").as_double_array();
    kd_values = get_node()->get_parameter("pid.kd").as_double_array();
    rated_effort_values = get_node()->get_parameter("effort_limits.rated").as_double_array();
    max_effort_values = get_node()->get_parameter("effort_limits.max").as_double_array();
    peak_duration = get_node()->get_parameter("effort_limits.peak_duration").as_double();
    rated_velocity_values = get_node()->get_parameter("velocity_limits.rated").as_double_array();
    max_velocity_values = get_node()->get_parameter("velocity_limits.max").as_double_array();
  } catch (const std::exception & e) {
    RCLCPP_WARN(logger, "PID parameters not fully specified, using defaults");
  }

  // Apply PID gains, effort limits, and velocity limits to each joint (use defaults if not specified in YAML)
  // Defaults from header: Kp=100.0, Ki=0.1, Kd=10.0
  for (size_t i = 0; i < num_joints; ++i) {
    if (i < kp_values.size()) pid_gains_[i].kp = kp_values[i];
    if (i < ki_values.size()) pid_gains_[i].ki = ki_values[i];
    if (i < kd_values.size()) pid_gains_[i].kd = kd_values[i];
    if (i < rated_effort_values.size()) effort_limits_[i].rated = rated_effort_values[i];
    if (i < max_effort_values.size()) effort_limits_[i].max = max_effort_values[i];
    if (i < rated_velocity_values.size()) velocity_limits_[i].rated = rated_velocity_values[i];
    if (i < max_velocity_values.size()) velocity_limits_[i].max = max_velocity_values[i];

    // Peak duration (from config, default 2 seconds)
    effort_limits_[i].peak_duration = peak_duration;

    RCLCPP_INFO(
      logger, "Joint %zu (%s): Kp=%.2f, Ki=%.4f, Kd=%.2f, Effort=[%.1f/%.1f Nm, %.1fs peak], Vel=[%.2f/%.2f rad/s]",
      i, joint_names_[i].c_str(),
      pid_gains_[i].kp, pid_gains_[i].ki, pid_gains_[i].kd,
      effort_limits_[i].rated, effort_limits_[i].max, effort_limits_[i].peak_duration,
      velocity_limits_[i].rated, velocity_limits_[i].max);

    // EXTRA DEBUG: Highlight crazy high gains (for testing PID functionality)
    if (pid_gains_[i].kp > 1000.0) {
      RCLCPP_WARN(
        logger, "  ⚠️  VERY HIGH Kp DETECTED: %.2f (testing PID functionality)",
        pid_gains_[i].kp);
    }
  }

  //===========================================================================
  // STEP 5: Enable dynamic PID tuning via ROS 2 parameters
  //===========================================================================
  // This allows changing PID gains on-the-fly without restarting the controller
  // Example: ros2 param set /arm_controller pid.kp "[120.0, 120.0, 90.0, 90.0, 70.0, 70.0]"
  param_callback_handle_ = get_node()->add_on_set_parameters_callback(
    std::bind(&ArmController::on_parameter_change, this, std::placeholders::_1));
  RCLCPP_INFO(logger, "Dynamic PID tuning enabled - use ros2 param set to change gains on the fly");

  //===========================================================================
  // STEP 6: Create debug publishers for PlotJuggler visualization
  //===========================================================================
  // These publishers continuously publish desired positions and errors
  // This makes it easy to visualize tracking performance in PlotJuggler
  debug_desired_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/debug/desired_positions", 10);
  debug_error_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/debug/position_errors", 10);
  RCLCPP_INFO(logger, "Debug publishers created for visualization");

  return controller_interface::CallbackReturn::SUCCESS;
}

//=============================================================================
// INTERFACE CONFIGURATION: Command Interfaces
// This is THE MOST CRITICAL function - it determines what interfaces we claim
//=============================================================================
controller_interface::InterfaceConfiguration
ArmController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // ⚠️ CRITICAL DESIGN DECISION ⚠️
  // We ONLY claim the EFFORT interface (torque output)
  // We do NOT claim the POSITION interface
  //
  // WHY?
  // ====
  // - If we claim position interface → Gazebo's GazeboSimSystem uses its built-in
  //   PD controller and IGNORES our effort commands
  // - By claiming ONLY effort → Gazebo uses effort-based physics simulation
  // - This allows us to control the PID gains and prepare for real hardware
  //
  // WHAT WE TRIED (and failed):
  // ===========================
  // 1. Claiming both position + effort → Gazebo ignored effort, used position
  // 2. Writing to position interface → Triggered Gazebo's controller (conflict!)
  // 3. Final solution → Claim only effort, store positions internally ✓
  //
  // Result: 1 interface per joint (effort only)
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
  }

  return config;
}

//=============================================================================
// INTERFACE CONFIGURATION: State Interfaces
// Tell ros2_control what sensor data we need to read
//=============================================================================
controller_interface::InterfaceConfiguration
ArmController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Request position and velocity feedback for each joint
  // - position: Used to compute error (desired - current)
  // - velocity: Used for derivative term in PID (damping)
  //
  // Result: 2 interfaces per joint (position + velocity)
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
  }

  return config;
}

//=============================================================================
// ACTIVATION: on_activate()
// Called when controller transitions from INACTIVE → ACTIVE state
// This is where we prepare to start controlling the robot
//=============================================================================
controller_interface::CallbackReturn ArmController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = get_node()->get_logger();

  //===========================================================================
  // VALIDATION: Check that we have the correct number of interfaces
  //===========================================================================
  // Command interfaces: 1 per joint (effort only)
  // State interfaces: 2 per joint (position + velocity)

  // Check command interfaces (effort only = 1 per joint)
  if (command_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      logger, "Expected %zu command interfaces (effort only), got %zu",
      joint_names_.size(), command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Check state interfaces (position + velocity = 2 per joint)
  if (state_interfaces_.size() != joint_names_.size() * 2) {
    RCLCPP_ERROR(
      logger, "Expected %zu state interfaces, got %zu",
      joint_names_.size() * 2, state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  //===========================================================================
  // INITIALIZATION: Set initial position commands to current robot state
  //===========================================================================
  // This prevents sudden jumps when the controller activates
  // We read the current position and use it as the initial desired position
  //
  // ⚠️ CRITICAL: We store these positions INTERNALLY only
  // We do NOT write to position command interfaces (we only claim effort!)
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    position_commands_[i] = state_interfaces_[i * 2].get_value();  // current position
    last_position_commands_[i] = position_commands_[i];

    // DO NOT write to position command interface - we only use effort commands to Gazebo!
    // Writing to position interface would activate Gazebo's internal PD controller
  }

  //===========================================================================
  // RESET PID STATE: Clear integral and error history
  //===========================================================================
  // This ensures we start with a clean slate (no accumulated error from previous runs)
  reset_pid_states();

  RCLCPP_INFO(logger, "Controller activated successfully!");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset PID states when deactivating
  reset_pid_states();

  RCLCPP_INFO(get_node()->get_logger(), "Controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

//=============================================================================
// MAIN CONTROL LOOP: update()
// Called at 100 Hz (every 10ms) - this is where the magic happens!
//=============================================================================
controller_interface::return_type ArmController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Calculate time step for PID integration/differentiation
  const double dt = period.seconds();

  if (dt <= 0.0) {
    return controller_interface::return_type::OK;
  }

  //===========================================================================
  // STEP 1: Get desired positions from either trajectory or topic
  //===========================================================================
  std::vector<double> trajectory_positions;
  std::vector<double> trajectory_velocities;

  if (sample_trajectory(time, trajectory_positions, trajectory_velocities)) {
    // We have an active trajectory from MoveIt/MTC action server
    // Use interpolated positions from the trajectory
    position_commands_ = trajectory_positions;
  } else {
    // No trajectory → use simple position commands from topic
    // (realtime buffer for thread-safe access from subscriber callback)
    auto position_commands_ptr = position_command_buffer_.readFromRT();
    if (position_commands_ptr && position_commands_ptr->size() == joint_names_.size()) {
      position_commands_ = *position_commands_ptr;

      // DEBUG: Periodically log received commands
      static int cmd_counter = 0;
      if (cmd_counter++ % 100 == 0) {
        RCLCPP_INFO(get_node()->get_logger(),
          "Topic command: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
          position_commands_[0], position_commands_[1], position_commands_[2],
          position_commands_[3], position_commands_[4], position_commands_[5]);
      }
    }
  }

  //===========================================================================
  // STEP 2: For each joint, compute PID and send effort command
  //===========================================================================
  // Prepare debug messages for PlotJuggler
  std_msgs::msg::Float64MultiArray desired_msg;
  std_msgs::msg::Float64MultiArray error_msg;
  desired_msg.data.resize(joint_names_.size());
  error_msg.data.resize(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    // Read current joint state from Gazebo/hardware
    // State interfaces have 2 values per joint: [position, velocity]
    const double current_position = state_interfaces_[i * 2].get_value();      // position
    const double current_velocity = state_interfaces_[i * 2 + 1].get_value();  // velocity

    // Get desired position from our internal storage
    // ⚠️ We do NOT read from command interfaces (we don't claim position interface)
    const double desired_position = position_commands_[i];

    // Store for debug publishing
    desired_msg.data[i] = desired_position;
    error_msg.data[i] = desired_position - current_position;

    // Compute PID control: converts position error → torque/effort
    // PID formula: effort = Kp*error + Ki*integral - Kd*velocity
    double effort = compute_pid(
      desired_position,
      current_position,
      current_velocity,
      pid_gains_[i],
      pid_states_[i],
      dt);

    // ===========================================================================
    // THERMAL-AWARE TORQUE LIMITING
    // ===========================================================================
    // Strategy: Allow peak torque for brief periods, then enforce rated torque
    // - If |effort| <= rated → always allow (thermal cooldown)
    // - If |effort| > rated → track duration, clamp to max, enforce peak_duration limit

    const double abs_effort = std::abs(effort);
    double effort_limit = effort_limits_[i].max;  // Start with max (peak) limit

    // Track thermal accumulation
    if (abs_effort > effort_limits_[i].rated) {
      // High effort - accumulate thermal load
      thermal_states_[i].high_effort_duration += dt;

      // Check if we've exceeded peak duration
      if (thermal_states_[i].high_effort_duration > effort_limits_[i].peak_duration) {
        // Thermal limit reached - restrict to rated torque
        effort_limit = effort_limits_[i].rated;

        // Log warning (throttled to once per second)
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(),
          *get_node()->get_clock(), 1000,
          "Joint %zu (%s) thermal limit: %.1fs at high effort, clamping to rated %.1f Nm",
          i, joint_names_[i].c_str(),
          thermal_states_[i].high_effort_duration,
          effort_limits_[i].rated);
      }
    } else {
      // Low effort - thermal cooldown (exponential decay)
      // Cooldown at 2x the rate of heating for safety margin
      thermal_states_[i].high_effort_duration = std::max(
        0.0,
        thermal_states_[i].high_effort_duration - dt * 2.0);
    }

    // Apply thermal-aware limit
    effort = std::clamp(effort, -effort_limit, effort_limit);

    // Velocity monitoring: Warn if joint exceeds rated velocity
    // Note: We don't clamp velocity here (it's feedback, not a command)
    // But we log warnings to detect if trajectory planning is too aggressive
    if (std::abs(current_velocity) > velocity_limits_[i].rated * 1.1) {  // 10% tolerance
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(), 1000,  // Log once per second max
        "Joint %zu (%s) velocity %.2f rad/s exceeds rated limit %.2f rad/s",
        i, joint_names_[i].c_str(), current_velocity, velocity_limits_[i].rated);
    }

    // DEBUG: Log elbow joint details every update
    if (i == 3) {
      static int counter = 0;
      if (counter++ % 100 == 0) {  // Every 1 second at 100Hz
        RCLCPP_INFO(
          get_node()->get_logger(),
          "ELBOW: desired=%.4f, current=%.4f, error=%.4f, effort=%.2f Nm, Kp=%.1f",
          desired_position, current_position,
          desired_position - current_position,
          effort, pid_gains_[i].kp);
      }
    }

    //=========================================================================
    // STEP 3: Send computed effort to Gazebo/hardware
    //=========================================================================
    // Write to effort command interface
    // Command interfaces now only have 1 value per joint (effort only)
    // Index is simply i (not i*2+1 as it was when we claimed position+effort)
    bool success = command_interfaces_[i].set_value(effort);

    // DEBUG: Check if write succeeded
    if (i == 3 && !success) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to write effort for elbow!");
    }
  }

  //===========================================================================
  // STEP 4: Publish debug data for PlotJuggler visualization
  //===========================================================================
  // This publishes continuously at 100Hz for smooth plotting
  debug_desired_pub_->publish(desired_msg);
  debug_error_pub_->publish(error_msg);

  return controller_interface::return_type::OK;
}

//=============================================================================
// PID COMPUTATION: The Heart of the Controller
// Converts position error into torque/effort command
//=============================================================================
double ArmController::compute_pid(
  double desired,       // Target position (rad)
  double current,       // Actual position (rad)
  double velocity,      // Current velocity (rad/s)
  const PIDGains & pid_gains,  // Kp, Ki, Kd values
  PIDState & pid_state,        // Integral accumulator, last error
  double dt)            // Time step (seconds)
{
  //===========================================================================
  // PROPORTIONAL TERM: Kp * error
  // Provides restoring force proportional to position error
  //===========================================================================
  const double error = desired - current;

  //===========================================================================
  // INTEGRAL TERM: Ki * ∫error dt
  // Eliminates steady-state error (accumulates error over time)
  // Anti-windup: Limit integral to prevent excessive buildup
  //===========================================================================
  pid_state.error_integral += error * dt;
  const double max_integral = 100.0;  // Prevent integral windup
  pid_state.error_integral = std::clamp(
    pid_state.error_integral, -max_integral, max_integral);

  //===========================================================================
  // DERIVATIVE TERM: Kd * d(error)/dt ≈ -Kd * velocity
  // Provides damping to reduce overshoot and oscillation
  //
  // Why -velocity instead of (error - last_error)/dt?
  // - Since d(desired)/dt ≈ 0 (desired changes slowly)
  // - d(error)/dt = d(desired - current)/dt ≈ -d(current)/dt = -velocity
  // - Using velocity directly → better noise rejection than numerical differentiation
  // - This is a standard technique in motor control
  //===========================================================================
  const double error_derivative = -velocity;

  //===========================================================================
  // FINAL PID OUTPUT
  // effort (Nm) = Kp*error + Ki*integral - Kd*velocity
  //
  // Physical meaning:
  // - Positive effort → motor applies torque to increase joint angle
  // - Negative effort → motor applies torque to decrease joint angle
  // - Gazebo/hardware receives this and simulates/applies the torque
  //===========================================================================
  const double effort =
    pid_gains.kp * error +
    pid_gains.ki * pid_state.error_integral +
    pid_gains.kd * error_derivative;

  // Store error for debugging/future use
  pid_state.last_error = error;

  return effort;
}

void ArmController::reset_pid_states()
{
  for (auto & state : pid_states_) {
    state.error_integral = 0.0;
    state.last_error = 0.0;
  }
}

//=============================================================================
// FOLLOWJOINTTRAJECTORY ACTION SERVER IMPLEMENTATION
// This is how MoveIt/MTC sends trajectories to the controller
//=============================================================================

/**
 * @brief Validate incoming trajectory goal from MoveIt/MTC
 *
 * Called when a new trajectory goal is received (before execution starts)
 * We validate that:
 * 1. The trajectory includes all required joints (6 arm joints)
 * 2. The trajectory has at least one waypoint
 * 3. The joint names match what we expect
 *
 * Returns: ACCEPT_AND_EXECUTE or REJECT
 */
rclcpp_action::GoalResponse ArmController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
  (void)uuid;  // Unused
  auto logger = get_node()->get_logger();

  RCLCPP_INFO(logger, "Received trajectory goal with %zu points",
    goal->trajectory.points.size());

  //===========================================================================
  // VALIDATION 1: Check joint count
  //===========================================================================
  if (goal->trajectory.joint_names.size() != joint_names_.size()) {
    RCLCPP_ERROR(logger, "Trajectory has %zu joints, expected %zu",
      goal->trajectory.joint_names.size(), joint_names_.size());
    return rclcpp_action::GoalResponse::REJECT;
  }

  //===========================================================================
  // VALIDATION 2: Check all required joints are present (order doesn't matter)
  //===========================================================================
  // MoveIt might send joints in different order, so we check by name
  for (const auto & required_joint : joint_names_) {
    bool found = false;
    for (const auto & traj_joint : goal->trajectory.joint_names) {
      if (traj_joint == required_joint) {
        found = true;
        break;
      }
    }
    if (!found) {
      RCLCPP_ERROR(logger, "Required joint '%s' not found in trajectory", required_joint.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  //===========================================================================
  // VALIDATION 3: Check trajectory has waypoints
  //===========================================================================
  if (goal->trajectory.points.empty()) {
    RCLCPP_ERROR(logger, "Trajectory has no points");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(logger, "Trajectory goal accepted");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Handle trajectory cancellation request
 *
 * Called when user/MoveIt requests to cancel the current trajectory
 * We immediately stop trajectory execution and return to topic command mode
 */
rclcpp_action::CancelResponse ArmController::handle_cancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  auto logger = get_node()->get_logger();
  RCLCPP_INFO(logger, "Received request to cancel trajectory");

  // Stop trajectory execution (update() loop will revert to topic commands)
  trajectory_state_.active = false;
  trajectory_state_.trajectory.reset();
  trajectory_state_.goal_handle.reset();

  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Start trajectory execution
 *
 * Called after goal is accepted and ready to execute
 * We store the trajectory and let the update() loop handle the actual execution
 *
 * Execution flow:
 * 1. Store trajectory and start time
 * 2. Mark trajectory as active
 * 3. update() loop samples trajectory at current time
 * 4. PID controller tracks the sampled positions
 * 5. When trajectory completes, send result to MoveIt
 */
void ArmController::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto logger = get_node()->get_logger();
  const auto goal = goal_handle->get_goal();

  RCLCPP_INFO(logger, "Starting trajectory execution");

  //===========================================================================
  // Store trajectory state for update() loop to use
  //===========================================================================
  trajectory_state_.active = true;
  trajectory_state_.trajectory = std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal->trajectory);
  trajectory_state_.current_point_index = 0;
  trajectory_state_.start_time = get_node()->now();
  trajectory_state_.goal_handle = goal_handle;

  // The actual trajectory execution happens in the update() loop
  // We just set it up here and let update() handle the interpolation
}

/**
 * @brief Sample trajectory at current time using linear interpolation
 *
 * This function is called by update() loop to get desired positions from the active trajectory
 *
 * Trajectory format (from MoveIt):
 *   points[0]: positions=[...], time_from_start=0.0s
 *   points[1]: positions=[...], time_from_start=1.5s
 *   points[2]: positions=[...], time_from_start=3.0s
 *   ...
 *
 * We linearly interpolate between waypoints based on current time
 *
 * @param current_time Current ROS time
 * @param positions [output] Interpolated joint positions
 * @param velocities [output] Interpolated joint velocities
 * @return true if trajectory is active and sampled successfully
 */
bool ArmController::sample_trajectory(
  const rclcpp::Time & current_time,
  std::vector<double> & positions,
  std::vector<double> & velocities)
{
  // No active trajectory → use topic commands instead
  if (!trajectory_state_.active || !trajectory_state_.trajectory) {
    return false;
  }

  const auto & traj = *trajectory_state_.trajectory;
  const size_t num_points = traj.points.size();

  //===========================================================================
  // STEP 1: Calculate elapsed time since trajectory started
  //===========================================================================
  const double elapsed = (current_time - trajectory_state_.start_time).seconds();

  //===========================================================================
  // STEP 2: Find which trajectory segment we're currently in
  //===========================================================================
  // We find the next waypoint whose timestamp is after current time
  size_t next_idx = trajectory_state_.current_point_index;

  for (size_t i = trajectory_state_.current_point_index; i < num_points; ++i) {
    const double point_time = traj.points[i].time_from_start.sec +
                              traj.points[i].time_from_start.nanosec * 1e-9;
    if (elapsed < point_time) {
      next_idx = i;
      break;
    }
  }

  //===========================================================================
  // STEP 3: Check if trajectory is complete
  //===========================================================================
  if (next_idx == 0 || elapsed >= (traj.points.back().time_from_start.sec +
                                   traj.points.back().time_from_start.nanosec * 1e-9)) {
    // We've reached or passed the final waypoint - trajectory complete!

    // Use last point positions (remap joint order if needed)
    const auto & last_point = traj.points.back();
    positions.resize(joint_names_.size());
    velocities.resize(joint_names_.size(), 0.0);

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      for (size_t j = 0; j < traj.joint_names.size(); ++j) {
        if (traj.joint_names[j] == joint_names_[i] && j < last_point.positions.size()) {
          positions[i] = last_point.positions[j];
          break;
        }
      }
    }

    // Notify MoveIt that trajectory execution succeeded
    if (trajectory_state_.goal_handle) {
      auto result = std::make_shared<FollowJTrajAction::Result>();
      result->error_code = FollowJTrajAction::Result::SUCCESSFUL;
      trajectory_state_.goal_handle->succeed(result);

      RCLCPP_INFO(get_node()->get_logger(), "Trajectory execution completed successfully");
    }

    // Clean up trajectory state
    trajectory_state_.active = false;
    trajectory_state_.trajectory.reset();
    trajectory_state_.goal_handle.reset();
    return true;
  }

  //===========================================================================
  // STEP 4: Linear interpolation between waypoints
  //===========================================================================
  // Example: If trajectory has points at t=0s, t=2s, t=4s
  //          and current time is t=1.5s (elapsed=1.5)
  //          → prev_idx=0 (t=0s), next_idx=1 (t=2s)
  //          → alpha = (1.5 - 0) / (2 - 0) = 0.75
  //          → position = 0.25*pos[0] + 0.75*pos[1]

  const size_t prev_idx = (next_idx > 0) ? next_idx - 1 : 0;
  const auto & prev_point = traj.points[prev_idx];
  const auto & next_point = traj.points[next_idx];

  const double prev_time = prev_point.time_from_start.sec + prev_point.time_from_start.nanosec * 1e-9;
  const double next_time = next_point.time_from_start.sec + next_point.time_from_start.nanosec * 1e-9;

  // Calculate interpolation factor (0.0 = at prev_point, 1.0 = at next_point)
  double alpha = 0.0;
  if (next_time > prev_time) {
    alpha = (elapsed - prev_time) / (next_time - prev_time);
    alpha = std::clamp(alpha, 0.0, 1.0);  // Safety clamp
  }

  //===========================================================================
  // STEP 5: Interpolate positions and velocities
  //===========================================================================
  positions.resize(joint_names_.size());
  velocities.resize(joint_names_.size(), 0.0);

  // MoveIt might send joints in different order than our controller expects
  // So we need to remap: trajectory joint order → controller joint order
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    // Find this controller joint in the trajectory
    size_t traj_idx = 0;
    for (size_t j = 0; j < traj.joint_names.size(); ++j) {
      if (traj.joint_names[j] == joint_names_[i]) {
        traj_idx = j;
        break;
      }
    }

    // Linear interpolation: pos = prev + alpha * (next - prev)
    if (traj_idx < prev_point.positions.size() && traj_idx < next_point.positions.size()) {
      positions[i] = prev_point.positions[traj_idx] + alpha * (next_point.positions[traj_idx] - prev_point.positions[traj_idx]);
    }

    // Interpolate velocities if provided by trajectory
    if (!next_point.velocities.empty() && traj_idx < next_point.velocities.size()) {
      velocities[i] = prev_point.velocities.empty() ? next_point.velocities[traj_idx] :
                      prev_point.velocities[traj_idx] + alpha * (next_point.velocities[traj_idx] - prev_point.velocities[traj_idx]);

      // Clamp trajectory velocities to rated limits for safety
      // This prevents MoveIt from commanding velocities that exceed continuous operation limits
      velocities[i] = std::clamp(velocities[i], -velocity_limits_[i].rated, velocity_limits_[i].rated);
    }
  }

  // Update current point index for next iteration
  trajectory_state_.current_point_index = prev_idx;

  //===========================================================================
  // STEP 6: Publish feedback to MoveIt (desired vs actual positions)
  //===========================================================================
  // This allows MoveIt to monitor trajectory execution progress
  if (trajectory_state_.goal_handle) {
    auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
    feedback->desired.positions = positions;
    feedback->desired.velocities = velocities;
    feedback->desired.time_from_start = traj.points[next_idx].time_from_start;

    // Get actual positions from state interfaces (for comparison)
    feedback->actual.positions.resize(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      feedback->actual.positions[i] = state_interfaces_[i * 2].get_value();
    }

    trajectory_state_.goal_handle->publish_feedback(feedback);
  }

  return true;
}

//=============================================================================
// DYNAMIC PID TUNING: on_parameter_change()
// Allows changing PID gains on-the-fly without restarting controller
//=============================================================================
/**
 * @brief Handle dynamic parameter changes for PID tuning
 *
 * This callback is triggered when ROS 2 parameters are changed at runtime
 * Example usage:
 *   ros2 param set /arm_controller pid.kp "[120.0, 120.0, 90.0, 90.0, 70.0, 70.0]"
 *   ros2 param set /arm_controller pid.kd "[3.0, 3.0, 2.0, 2.0, 1.5, 1.5]"
 *
 * This is extremely useful for tuning PID gains while the robot is running
 *
 * @param parameters List of parameters being changed
 * @return Result indicating success/failure
 */
rcl_interfaces::msg::SetParametersResult ArmController::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (param.get_name() == "pid.kp") {
      // Update proportional gains
      auto values = param.as_double_array();
      for (size_t i = 0; i < std::min(values.size(), pid_gains_.size()); ++i) {
        pid_gains_[i].kp = values[i];
      }
      RCLCPP_INFO(get_node()->get_logger(), "Updated Kp gains dynamically");

    } else if (param.get_name() == "pid.ki") {
      // Update integral gains and reset integral state
      auto values = param.as_double_array();
      for (size_t i = 0; i < std::min(values.size(), pid_gains_.size()); ++i) {
        pid_gains_[i].ki = values[i];
      }
      // Reset integral terms when ki changes to prevent windup issues
      reset_pid_states();
      RCLCPP_INFO(get_node()->get_logger(), "Updated Ki gains dynamically (integral reset)");

    } else if (param.get_name() == "pid.kd") {
      // Update derivative gains
      auto values = param.as_double_array();
      for (size_t i = 0; i < std::min(values.size(), pid_gains_.size()); ++i) {
        pid_gains_[i].kd = values[i];
      }
      RCLCPP_INFO(get_node()->get_logger(), "Updated Kd gains dynamically");
    }
  }

  return result;
}

}  // namespace arm_control

// Register controller as a plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_control::ArmController, controller_interface::ControllerInterface)
