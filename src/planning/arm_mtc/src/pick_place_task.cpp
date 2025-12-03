/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Based on MoveIt Task Constructor tutorial (Panda pick_place_demo)
 * Adapted for LDR Humanoid Arm System - ROS 2 Jazzy
 *
 * Pick and place task using MoveIt Task Constructor
 * - Picks cylinder from source table
 * - Places cylinder on destination table
 * - Full gripper integration
 *********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <Eigen/Geometry>

#include <iostream>
#include <unistd.h>  // for getpid()

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place");
namespace mtc = moveit::task_constructor;

class MTCTaskNode : public rclcpp::Node
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
  {
    return node_base_interface_;
  }

  void doTask();
  void setupPlanningScene();

private:
  mtc::Task task_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface_;
  rclcpp::Publisher<moveit_task_constructor_msgs::msg::Solution>::SharedPtr solution_pub_;

  bool publishBestSolution();
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : Node("mtc_pick_place", options)
{
  node_base_interface_ = this->get_node_base_interface();

  // Publishes the selected MTC solution so the external executor can run it
  solution_pub_ = this->create_publisher<moveit_task_constructor_msgs::msg::Solution>("/solution", 10);

  // Parameters are automatically declared from YAML via automatically_declare_parameters_from_overrides(true)
  // in main(), so we don't need to manually declare them here
}

void MTCTaskNode::setupPlanningScene()
{
  RCLCPP_INFO(LOGGER, "Waiting for planning scene objects...");

  moveit::planning_interface::PlanningSceneInterface psi;

  // Wait for test_cylinder to appear
  int max_attempts = 200;
  int attempt = 0;
  bool found = false;

  while (attempt < max_attempts && !found)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    auto objects = psi.getKnownObjectNames();
    for (const auto& obj : objects)
    {
      if (obj == "test_cylinder")
      {
        found = true;
        RCLCPP_INFO(LOGGER, "✓ Found test_cylinder in planning scene");
        break;
      }
    }

    if (!found)
    {
      RCLCPP_INFO(LOGGER, "  Waiting for test_cylinder... (attempt %d/%d)", attempt + 1, max_attempts);
      rclcpp::sleep_for(std::chrono::milliseconds(400));
      attempt++;
    }
  }

  if (!found)
  {
    RCLCPP_ERROR(LOGGER, "❌ test_cylinder not found in planning scene!");
    RCLCPP_ERROR(LOGGER, "   Make sure planning_scene_publisher.py is running:");
    RCLCPP_ERROR(LOGGER, "   ros2 run arm_perception planning_scene_publisher.py");
  }
  else
  {
    auto objects = psi.getKnownObjectNames();
    RCLCPP_INFO(LOGGER, "Planning scene objects: %zu", objects.size());
    for (const auto& obj : objects)
    {
      RCLCPP_INFO(LOGGER, "  - %s", obj.c_str());
    }
  }
}

void MTCTaskNode::doTask()
{
  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "Starting MTC Pick and Place Task");
  RCLCPP_INFO(LOGGER, "========================================");

  task_.stages()->setName("Pick and Place");
  task_.loadRobotModel(shared_from_this());

  setupPlanningScene();

  // Load parameters from YAML
  const std::string arm_group = this->get_parameter("groups.arm").as_string();
  const std::string hand_group = this->get_parameter("groups.hand").as_string();
  const std::string eef_group = this->get_parameter("groups.eef").as_string();
  const std::string hand_frame = this->get_parameter("groups.hand_frame").as_string();
  const std::string hand_open_pose = this->get_parameter("poses.hand_open").as_string();
  const std::string hand_close_pose = this->get_parameter("poses.hand_close").as_string();
  const std::string arm_home_pose = this->get_parameter("poses.arm_home").as_string();
  const std::string arm_ready_pose = this->get_parameter("poses.arm_ready").as_string();
  const std::string world_frame = this->get_parameter("world_frame").as_string();

  const double dest_table_x = this->get_parameter("destination_table.position.x").as_double();
  const double dest_table_y = this->get_parameter("destination_table.position.y").as_double();
  const double dest_table_z = this->get_parameter("destination_table.position.z").as_double();
  const double dest_table_thickness = this->get_parameter("destination_table.dimensions.thickness").as_double();
  const double cylinder_height = this->get_parameter("cylinder.dimensions.height").as_double();

  const double goal_joint_tolerance = this->get_parameter("planner.goal_joint_tolerance").as_double();
  const double cartesian_step_size = this->get_parameter("planner.cartesian_step_size").as_double();
  const double max_velocity_scaling = this->get_parameter("planner.max_velocity_scaling").as_double();
  const double max_acceleration_scaling = this->get_parameter("planner.max_acceleration_scaling").as_double();

  const double object_radius = this->get_parameter("cylinder.dimensions.radius").as_double();

  // Create planners (following tutorial pattern exactly)
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
  sampling_planner->setProperty("goal_joint_tolerance", goal_joint_tolerance);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(max_velocity_scaling);
  cartesian_planner->setMaxAccelerationScalingFactor(max_acceleration_scaling);
  cartesian_planner->setStepSize(cartesian_step_size);

  // Set task properties (following tutorial pattern)
  task_.setProperty("group", arm_group);
  task_.setProperty("eef", eef_group);
  task_.setProperty("hand", hand_group);
  task_.setProperty("hand_grasping_frame", hand_frame);
  task_.setProperty("ik_frame", hand_frame);

  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/
  mtc::Stage* initial_state_ptr = nullptr;
  {
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");

    // Verify that object is not attached (following tutorial pattern)
    auto applicability_filter =
        std::make_unique<mtc::stages::PredicateFilter>("applicability test", std::move(current_state));
    applicability_filter->setPredicate(
        [](const mtc::SolutionBase& s, std::string& comment) {
          if (s.start()->scene()->getCurrentState().hasAttachedBody("test_cylinder")) {
            comment = "object with id 'test_cylinder' is already attached and cannot be picked";
            return false;
          }
          return true;
        });
    task_.add(std::move(applicability_filter));
  }

  /****************************************************
   *                                                  *
   *               Open Hand                          *
   *                                                  *
   ***************************************************/
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
    stage->setGroup(hand_group);
    stage->setGoal(hand_open_pose);
    initial_state_ptr = stage.get();  // Remember for monitoring grasp generator
    task_.add(std::move(stage));
  }

  /****************************************************
   *                                                  *
   *               Move to Pick                       *
   *                                                  *
   ***************************************************/
  {
    // Only move the arm between open-hand and grasp setup to avoid mixed arm+gripper trajectories
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pick", mtc::stages::Connect::GroupPlannerVector{ { "arm", sampling_planner } });
    stage->setTimeout(15.0);  // Increased timeout for complex planning
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task_.add(std::move(stage));
  }

  /****************************************************
   *                                                  *
   *               Pick Object                        *
   *                                                  *
   ***************************************************/
  mtc::Stage* pick_stage_ptr = nullptr;
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task_.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "hand", "group", "ik_frame" });


    /****************************************************
  ---- *               Approach Object (15cm offset)    *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.15);  // Approach up to 15cm
      stage->setIKFrame(hand_frame);

      // Approach direction: move forward in hand frame (negative Z in base_link)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = -1.0;  // Approach along hand frame Z axis (towards object)
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    
    /****************************************************
  ---- *               Generate Grasp Pose (sampled)    *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setObject("test_cylinder");
      stage->setPreGraspPose(hand_open_pose);
      stage->setAngleDelta(M_PI / 12);  // 15 degrees between grasp samples (tutorial pattern)
      stage->setMonitoredStage(initial_state_ptr);

      // ComputeIK wrapper - use collision detection to validate grasp poses
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);

      // Grasp frame transform: defines where on the gripper the object center should be
      // GenerateGraspPose creates poses at object surface, we need offset to position gripper correctly
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();

      // Rotation: align gripper with grasp direction
      // GenerateGraspPose creates poses where X points towards object, Z is up (cylinder axis)
      // Rotate 15 degrees around Z to align gripper optimally
      Eigen::AngleAxisd rotation(M_PI / 12, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = rotation.toRotationMatrix();

      // Translation: offset to position object between fingers
      // This offset accounts for gripper geometry - places object center at grasp point
      grasp_frame_transform.translation().y() = 0.08;  // 8cm offset for gripper geometry

      wrapper->setIKFrame(grasp_frame_transform, hand_frame);

      // Enable collision detection - validates that grasp poses are collision-free
      // This ensures gripper doesn't collide with object or environment during grasp
      wrapper->setIgnoreCollisions(false);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }

    /****************************************************
  ---- *               Allow Collision (hand object)   *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
          "test_cylinder",
          task_.getRobotModel()->getJointModelGroup("hand")->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Allow collision (object support)   *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,support)");
      stage->allowCollisions("test_cylinder", "test_table", true);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  ---- *               Close Hand (around object)      *
     ***************************************************/
    {
      // Calculate gripper closing position based on object radius
      // IMPORTANT: Gripper logic (from SRDF and joint limits):
      //   right_finger: -0.033m = OPEN (fully), +0.00026m = CLOSED (fully)
      //   Positive values = closing motion, Negative values = opening motion
      //
      // Strategy: Close gripper 5% more than object radius for firm grip
      // Formula: gripper_position = -(object_radius * 0.95)
      // This ensures fingers press slightly into object for secure grasp

      const double grip_compression = 0.95;  // Close 5% tighter than object radius

      // Map object radius to gripper joint position with compression
      // Examples:
      //   object_radius = 0.033m → gripper_position ≈ -0.031m (slightly compressed)
      //   object_radius = 0.016m → gripper_position ≈ -0.015m (5% tighter grip)
      //   object_radius = 0.000m → gripper_position ≈ 0.000m (fully closed)
      double gripper_close_position = -(object_radius * grip_compression);

      // Create MoveTo stage to close gripper around object
      // NOTE: Only control right_finger - left_finger is mimic joint (multiplier=-1.0)
      //       Setting right_finger automatically moves left_finger in opposite direction
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
      stage->setGroup(hand_group);

      // Set joint goal for RIGHT finger only (left finger mimics automatically)
      std::map<std::string, double> gripper_positions;
      gripper_positions["left_palm_right_finger"] = gripper_close_position;
      stage->setGoal(gripper_positions);

      RCLCPP_INFO(LOGGER, "Close hand: object_radius=%.4fm, position=%.4fm )",
                  object_radius, gripper_close_position);

      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Attach Object                      *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("test_cylinder", hand_frame);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Lift object                        *
     ***************************************************/
    {
      // Lift object upward after grasping
      // In base_link frame: X=right, Y=down, Z=forward
      // So to go UP, we need -Y direction!
      const double lift_min = 0.0;    // Allow zero lift (flexible)
      const double lift_max = 0.10;   // Try up to 10cm

      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(lift_min, lift_max);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // CRITICAL: Set UPWARD direction = -Y in base_link (since Y points DOWN)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame;  // base_link
      vec.vector.x = 0.0;
      vec.vector.y = 0.0;  
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /****************************************************
  .... *               Forbid collision (object support)  *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,surface)");
      stage->allowCollisions("test_cylinder", "test_table", false);
      grasp->insert(std::move(stage));
    }

    pick_stage_ptr = grasp.get();  // Remember for monitoring place pose generator
    task_.add(std::move(grasp));
  }

  /******************************************************
   *                                                    *
   *          Move to Place                             *
   *                                                    *
   *****************************************************/
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{ { "arm", sampling_planner } });
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task_.add(std::move(stage));
  }

  /******************************************************
   *                                                    *
   *          Place Object                              *
   *                                                    *
   *****************************************************/
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task_.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "hand", "group" });

    /******************************************************
     *          Generate Place Pose (Destination Table)     *
     *****************************************************/
    {
      // Place object on destination table (absolute position in world frame)
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
        stage->properties().configureInitFrom(mtc::Stage::PARENT);
        stage->properties().set("marker_ns", "place_pose");
      stage->setObject("test_cylinder");

      // Pose is in WORLD FRAME (base_link), on destination table surface
      geometry_msgs::msg::PoseStamped p;
      p.header.frame_id = world_frame;  // Absolute position in world frame
      p.pose.position.x = dest_table_x;
      p.pose.position.y = dest_table_y;
      // Place on table surface: table_z + half_thickness + half_cylinder_height
      p.pose.position.z = dest_table_z + (dest_table_thickness / 2.0) + (cylinder_height / 2.0);
      p.pose.orientation.w = 1.0;  // Identity orientation
      stage->setPose(p);
        stage->setMonitoredStage(pick_stage_ptr);

      // IK wrapper - use hand frame for IK with offset
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);  // More solutions for better chance
      wrapper->setMinSolutionDistance(0.05);

      // Place frame transform: same 8cm offset as grasp
      Eigen::Isometry3d place_frame_transform = Eigen::Isometry3d::Identity();

      // Rotation: align gripper with place direction (same as grasp)
      Eigen::AngleAxisd rotation(M_PI / 12, Eigen::Vector3d::UnitZ());
      place_frame_transform.linear() = rotation.toRotationMatrix();

      // Translation: 8cm offset along Y axis (same as grasp)
      place_frame_transform.translation().y() = 0.08;

      wrapper->setIKFrame(place_frame_transform, hand_frame);
        wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
        wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    /******************************************************
     *          Open Hand (release object)               *
     *****************************************************/
    {
      // Open gripper fully to release object after placement
      // IMPORTANT: Gripper logic (from SRDF and joint limits):
      //   right_finger: -0.033m = OPEN (fully), +0.00026m = CLOSED (fully)
      //   Negative values = opening motion, Positive values = closing motion
      //
      // Strategy: Open gripper to maximum position to ensure clean release
      // This prevents object from sticking to gripper after placement

      const double GRIPPER_LIMIT_OPEN = -0.033;    // Maximum opening (most negative value)

      // Set gripper to fully open position for complete object release
      double gripper_open_position = GRIPPER_LIMIT_OPEN;  // Fully open (-0.033m)

      // Create MoveTo stage to open gripper and release object
      // NOTE: Only control right_finger - left_finger is mimic joint (multiplier=-1.0)
      //       Setting right_finger automatically moves left_finger in opposite direction
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
      stage->setGroup(hand_group);

      // Set joint goal for RIGHT finger only (left finger mimics automatically)
      std::map<std::string, double> gripper_positions;
      gripper_positions["left_palm_right_finger"] = gripper_open_position;
      stage->setGoal(gripper_positions);

      RCLCPP_INFO(LOGGER, "Open hand: position=%.4fm (fully open at limit: %.4fm)",
                  gripper_open_position, GRIPPER_LIMIT_OPEN);

      place->insert(std::move(stage));
    }

    /******************************************************
     *          Forbid collision (hand,object)            *
     *****************************************************/
    {
      // TUTORIAL PATTERN: Forbid collisions after opening hand (line 448-451)
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("test_cylinder",
                             *task_.getRobotModel()->getJointModelGroup(hand_group), false);
      place->insert(std::move(stage));
    }

    /******************************************************
     *          Detach Object                             *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("test_cylinder", hand_frame);
      place->insert(std::move(stage));
    }

    /******************************************************
     *          Retreat Motion                            *
     *****************************************************/
    {
      // TUTORIAL PATTERN: Retreat motion (line 467-476)
      // Adapted for base_link frame: retreat in -Z direction (backward)
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat after place", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.3);  // Tutorial uses these values
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame;  // base_link
      vec.vector.z = 1;  // Retreat backward (tutorial pattern)
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task_.add(std::move(place));
  }

  /******************************************************
   *                                                    *
   *          Move to Home                              *
   *                                                    *
   *****************************************************/
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move home", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal(arm_home_pose);
    stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
    task_.add(std::move(stage));
  }

  // Prepare task for planning
  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR(LOGGER, "Task initialization failed: %s", e.what());
    return;
  }

  // Plan task
  RCLCPP_INFO(LOGGER, "Planning task...");
  if (!task_.plan(10))  // max 10 solutions
  {
    RCLCPP_ERROR(LOGGER, "Task planning failed");
    return;
  }

  // Publish best solution on /solution for the executor (sorted by cost internally)
  if (!publishBestSolution())
  {
    return;
  }

  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "✅ Task planning completed successfully!");
  RCLCPP_INFO(LOGGER, "Solution published to /solution topic");
  RCLCPP_INFO(LOGGER, "To execute, use RViz 'Execute' button or call:");
  RCLCPP_INFO(LOGGER, "  ros2 action send_goal /execute_task_solution ...");
  RCLCPP_INFO(LOGGER, "========================================");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

#ifndef NDEBUG
  // DEBUG BUILD ONLY: Wait for continue signal
  // Check for /tmp/mtc_debug_wait file - delete it to continue
  const char* debug_flag = "/tmp/mtc_debug_wait";
  if (access(debug_flag, F_OK) == 0) {
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "🐛 DEBUG BUILD: Waiting for continue signal\n";
    std::cout << "========================================\n";
    std::cout << "Process ID: " << getpid() << "\n";
    std::cout << "Build Type: Debug\n";
    std::cout << "\nTo continue execution, run in another terminal:\n";
    std::cout << "  rm /tmp/mtc_debug_wait\n";
    std::cout << "\nOr to disable debug wait:\n";
    std::cout << "  Rebuild with: colcon build --packages-select arm_perception\n";
    std::cout << "  (without -DCMAKE_BUILD_TYPE=Debug)\n";
    std::cout << "\nWaiting for signal...\n";
    std::cout << "========================================\n\n";

    // Wait until file is deleted
    while (access(debug_flag, F_OK) == 0) {
      std::cout << "Waiting... (PID: " << getpid() << ")\r" << std::flush;
      sleep(1);
    }
    std::cout << "\n✅ Continue signal received!\n\n";
  }
#endif

  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->doTask();

  spin_thread->join();

  rclcpp::shutdown();
  return 0;
}

bool MTCTaskNode::publishBestSolution()
{
  if (task_.solutions().empty())
  {
    RCLCPP_ERROR(LOGGER, "No solutions available to publish");
    return false;
  }

  moveit_task_constructor_msgs::msg::Solution solution_msg;
  task_.solutions().front()->toMsg(solution_msg);
  solution_pub_->publish(solution_msg);

  RCLCPP_INFO(LOGGER,
              "Published best MTC solution to /solution (sub-trajectories: %zu)",
              solution_msg.sub_trajectory.size());
  return true;
}
