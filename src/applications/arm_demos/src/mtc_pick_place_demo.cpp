/**
 * @file mtc_pick_place_demo.cpp
 * @brief MoveIt Task Constructor Pick and Place Demo (C++ port)
 *
 * Demonstrates using MTC to perform a pick-and-place task with the
 * 6-DOF humanoid arm. Stages:
 * - CurrentState
 * - Pick container (approach, grasp IK, allow collision, attach, lift)
 * - Connect (move to place)
 * - Place container (lower, place pose, detach, forbid collision, retreat)
 * - Return home
 *
 * Author: LDR Robotics Team
 * License: MIT
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace mtc = moveit::task_constructor;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place_demo");

class MTCPickPlaceDemo
{
public:
  MTCPickPlaceDemo(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void run();

private:
  void setupPlanningScene();
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  std::string arm_group_name_;
  std::string hand_group_name_;
  std::string eef_name_;
  std::string hand_frame_;
  std::string object_name_;
  std::string surface_name_;
};

MTCPickPlaceDemo::MTCPickPlaceDemo(const rclcpp::NodeOptions& options)
  : node_(std::make_shared<rclcpp::Node>("mtc_pick_place_demo", options))
{
  arm_group_name_ = "arm";
  hand_group_name_ = "hand";
  eef_name_ = "tcp";
  hand_frame_ = "wrist_roll_link";
  object_name_ = "target_object";
  surface_name_ = "table";

  RCLCPP_INFO(LOGGER, "MTC Pick and Place Demo initialized");
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MTCPickPlaceDemo::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCPickPlaceDemo::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;

  // Add a table as support surface
  {
    moveit_msgs::msg::CollisionObject table;
    table.id = surface_name_;
    table.header.frame_id = "base_link";
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = {0.6, 0.6, 0.02};
    table.primitive_poses.resize(1);
    table.primitive_poses[0].position.x = 0.3;
    table.primitive_poses[0].position.y = 0.0;
    table.primitive_poses[0].position.z = 0.19;
    table.primitive_poses[0].orientation.w = 1.0;
    table.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi.applyCollisionObject(table);
  }

  // Add target object (cylinder)
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = object_name_;
    object.header.frame_id = "base_link";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = {0.1, 0.025};  // height, radius
    object.primitive_poses.resize(1);
    object.primitive_poses[0].position.x = 0.3;
    object.primitive_poses[0].position.y = 0.0;
    object.primitive_poses[0].position.z = 0.25;
    object.primitive_poses[0].orientation.w = 1.0;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi.applyCollisionObject(object);
  }

  RCLCPP_INFO(LOGGER, "Planning scene set up with table and target object");
}

mtc::Task MTCPickPlaceDemo::createTask()
{
  mtc::Task task;
  task.stages()->setName("pick_place_task");
  task.loadRobotModel(node_);

  // Set task properties
  task.setProperty("group", arm_group_name_);
  task.setProperty("eef", hand_group_name_);
  task.setProperty("ik_frame", hand_frame_);

  // Create planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  // Stage 1: Current State
  mtc::Stage* current_state_ptr = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  // ==========================
  // Pick Object Container
  // ==========================
  {
    auto pick_container = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(pick_container->properties(), {"eef", "group", "ik_frame"});
    pick_container->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // 3a: Approach object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.05, 0.15);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "base_link";
      direction.vector.z = -1.0;  // Move down
      stage->setDirection(direction);
      pick_container->insert(std::move(stage));
    }

    // 3b: Generate grasp pose
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->setPreGraspPose("open");
      stage->setObject(object_name_);
      stage->setAngleDelta(0.2);
      stage->setMonitoredStage(current_state_ptr);

      // Compute IK for grasp
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setIKFrame(hand_frame_);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      pick_container->insert(std::move(wrapper));
    }

    // 3c: Allow collision (hand-object)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand-object)");
      stage->allowCollisions(
        object_name_,
        task.getRobotModel()
          ->getJointModelGroup(hand_group_name_)
          ->getLinkModelNamesWithCollisionGeometry(),
        true);
      pick_container->insert(std::move(stage));
    }

    // 3d: Attach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_name_, hand_frame_);
      pick_container->insert(std::move(stage));
    }

    // 3e: Lift object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.08, 0.15);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "base_link";
      direction.vector.z = 1.0;  // Lift up
      stage->setDirection(direction);
      pick_container->insert(std::move(stage));
    }

    task.add(std::move(pick_container));
  }

  // ==========================
  // Move to Place
  // ==========================
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{
        {arm_group_name_, sampling_planner}
      });
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // ==========================
  // Place Object Container
  // ==========================
  {
    auto place_container = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place_container->properties(), {"eef", "group", "ik_frame"});
    place_container->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // 5a: Lower object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.05, 0.15);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "base_link";
      direction.vector.z = -1.0;  // Move down
      stage->setDirection(direction);
      place_container->insert(std::move(stage));
    }

    // 5b: Generate place pose
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"ik_frame"});
      stage->setObject(object_name_);

      geometry_msgs::msg::PoseStamped place_pose;
      place_pose.header.frame_id = "base_link";
      place_pose.pose.position.x = 0.3;
      place_pose.pose.position.y = 0.3;
      place_pose.pose.position.z = 0.25;
      place_pose.pose.orientation.w = 1.0;
      stage->setPose(place_pose);
      stage->setMonitoredStage(current_state_ptr);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setIKFrame(hand_frame_);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place_container->insert(std::move(wrapper));
    }

    // 5c: Detach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_name_, hand_frame_);
      place_container->insert(std::move(stage));
    }

    // 5d: Forbid collision (hand-object)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand-object)");
      stage->allowCollisions(
        object_name_,
        task.getRobotModel()
          ->getJointModelGroup(hand_group_name_)
          ->getLinkModelNamesWithCollisionGeometry(),
        false);
      place_container->insert(std::move(stage));
    }

    // 5e: Retreat
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.05, 0.15);

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = "base_link";
      direction.vector.z = 1.0;  // Retreat up
      stage->setDirection(direction);
      place_container->insert(std::move(stage));
    }

    task.add(std::move(place_container));
  }

  // ==========================
  // Return Home
  // ==========================
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    stage->setGroup(arm_group_name_);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  RCLCPP_INFO(LOGGER, "MTC pick-and-place task created with all stages");
  return task;
}

void MTCPickPlaceDemo::run()
{
  // Setup planning scene
  RCLCPP_INFO(LOGGER, "Setting up planning scene...");
  rclcpp::sleep_for(std::chrono::seconds(2));
  setupPlanningScene();

  // Create task
  RCLCPP_INFO(LOGGER, "Creating task...");
  task_ = createTask();

  // Initialize task
  try {
    task_.init();
    RCLCPP_INFO(LOGGER, "Task initialized successfully");
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
    return;
  }

  // Plan task
  RCLCPP_INFO(LOGGER, "Planning task...");
  if (!task_.plan(5)) {
    RCLCPP_ERROR(LOGGER, "Task planning failed - no solutions found");
    return;
  }

  RCLCPP_INFO(LOGGER, "Planning succeeded! %zu solution(s) found", task_.solutions().size());

  // Publish solution for visualization
  task_.introspection().publishSolution(*task_.solutions().front());

  // Execute the best solution
  RCLCPP_INFO(LOGGER, "Executing task...");
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Task execution failed with error code: %d", result.val);
    return;
  }

  RCLCPP_INFO(LOGGER, "Pick-and-place execution completed successfully!");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto demo = std::make_shared<MTCPickPlaceDemo>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(demo->getNodeBaseInterface());

  auto spin_thread = std::make_unique<std::thread>([&executor]() {
    executor.spin();
  });

  // Wait for system to be ready
  RCLCPP_INFO(LOGGER, "Waiting for system to be ready...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  demo->run();

  RCLCPP_INFO(LOGGER, "Demo completed. Keeping node alive for visualization. Press Ctrl+C to exit.");
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
