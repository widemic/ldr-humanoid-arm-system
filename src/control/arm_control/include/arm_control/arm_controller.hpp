#ifndef ARM_CONTROL__ARM_CONTROLLER_HPP_
#define ARM_CONTROL__ARM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

namespace arm_control
{

/**
 * @brief Custom Position Controller with PID control for the LDR Humanoid Arm
 *
 * This controller implements basic PID position control for each joint.
 * It's designed to replace Gazebo's built-in controller to allow:
 * - PID parameter tuning in simulation
 * - Preparation for real hardware integration
 * - Direct control over position commands
 *
 * Features:
 * - Per-joint PID gains (configurable via parameters)
 * - Position command interface
 * - Joint state feedback (position, velocity, effort)
 * - FollowJointTrajectory action interface (future)
 *
 * TODO (incremental features):
 * - [ ] Trajectory interpolation
 * - [ ] Velocity feedforward
 * - [ ] Anti-windup for integral term
 * - [ ] Dynamic reconfigure for PID gains
 * - [ ] Joint limits enforcement
 */
class ArmController : public controller_interface::ControllerInterface
{
public:
  ArmController();

  /**
   * @brief Initialize the controller
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Joint configuration
  std::vector<std::string> joint_names_;

  // PID parameters (per joint)
  struct PIDGains {
    double kp{100.0};  // Proportional gain
    double ki{0.1};    // Integral gain
    double kd{10.0};   // Derivative gain
  };
  std::vector<PIDGains> pid_gains_;

  // PID state (per joint)
  struct PIDState {
    double error_integral{0.0};
    double last_error{0.0};
  };
  std::vector<PIDState> pid_states_;

  // Command and state buffers
  std::vector<double> position_commands_;
  std::vector<double> last_position_commands_;

  // Command subscriber (for simple position commands)
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_subscriber_;
  realtime_tools::RealtimeBuffer<std::vector<double>> position_command_buffer_;

  // FollowJointTrajectory action server (for MoveIt integration)
  using FollowJTrajAction = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJTrajAction>;
  rclcpp_action::Server<FollowJTrajAction>::SharedPtr action_server_;

  // Trajectory state
  struct TrajectoryState {
    bool active{false};
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory;
    size_t current_point_index{0};
    rclcpp::Time start_time;
    std::shared_ptr<GoalHandle> goal_handle;
  };
  TrajectoryState trajectory_state_;

  /**
   * @brief Compute PID control output
   * @param desired Desired position
   * @param current Current position
   * @param velocity Current velocity
   * @param pid_gains PID gains for this joint
   * @param pid_state PID state for this joint
   * @param dt Time step
   * @return Control effort (force/torque)
   */
  double compute_pid(
    double desired,
    double current,
    double velocity,
    const PIDGains & pid_gains,
    PIDState & pid_state,
    double dt);

  /**
   * @brief Reset PID state (clear integrators)
   */
  void reset_pid_states();

  /**
   * @brief Action server callbacks
   */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJTrajAction::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

  /**
   * @brief Sample trajectory at current time
   */
  bool sample_trajectory(
    const rclcpp::Time & current_time,
    std::vector<double> & positions,
    std::vector<double> & velocities);

  /**
   * @brief Parameter callback for dynamic PID tuning
   */
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters);

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace arm_control

#endif  // ARM_CONTROL__ARM_CONTROLLER_HPP_
