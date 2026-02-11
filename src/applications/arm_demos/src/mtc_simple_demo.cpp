/**
 * @file mtc_simple_demo.cpp
 * @brief MoveIt Task Constructor Simple Demo (C++ port)
 *
 * A simplified demonstration of MTC showing basic task composition:
 * - Move to home position
 * - Approach a target pose
 * - Retreat back
 * - Return to home
 *
 * This example is ideal for learning MTC concepts without gripper complexity.
 *
 * Author: LDR Robotics Team
 * License: MIT
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace mtc = moveit::task_constructor;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_simple_demo");

class MTCSimpleDemo
{
public:
  MTCSimpleDemo(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void run();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  std::string arm_group_name_;
  std::string hand_frame_;
};

MTCSimpleDemo::MTCSimpleDemo(const rclcpp::NodeOptions& options)
  : node_(std::make_shared<rclcpp::Node>("mtc_simple_demo", options))
{
  arm_group_name_ = "arm";
  hand_frame_ = "wrist_roll_link";
  RCLCPP_INFO(LOGGER, "MTC Simple Demo initialized");
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MTCSimpleDemo::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

mtc::Task MTCSimpleDemo::createTask()
{
  mtc::Task task;
  task.stages()->setName("simple_approach_retreat");
  task.loadRobotModel(node_);

  // Set task properties
  task.setProperty("group", arm_group_name_);
  task.setProperty("ik_frame", hand_frame_);

  // Create planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(0.5);
  cartesian_planner->setMaxAccelerationScalingFactor(0.5);
  cartesian_planner->setStepSize(0.01);

  // Stage 1: Start from current state
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current state");
    task.add(std::move(stage));
  }

  // Stage 2: Move to home position
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to home", sampling_planner);
    stage->setGroup(arm_group_name_);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  // Stage 3: Move forward (approach)
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
    stage->setGroup(arm_group_name_);
    stage->setMinMaxDistance(0.10, 0.20);

    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = hand_frame_;
    direction.vector.x = 1.0;  // Forward
    stage->setDirection(direction);
    task.add(std::move(stage));
  }

  // Stage 4: Retreat (move back)
  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
    stage->setGroup(arm_group_name_);
    stage->setMinMaxDistance(0.10, 0.20);

    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = hand_frame_;
    direction.vector.x = -1.0;  // Backward
    stage->setDirection(direction);
    task.add(std::move(stage));
  }

  // Stage 5: Return to home
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return to home", sampling_planner);
    stage->setGroup(arm_group_name_);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  RCLCPP_INFO(LOGGER, "Simple MTC task created");
  return task;
}

void MTCSimpleDemo::run()
{
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

  RCLCPP_INFO(LOGGER, "Execution completed successfully!");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto demo = std::make_shared<MTCSimpleDemo>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(demo->getNodeBaseInterface());

  auto spin_thread = std::make_unique<std::thread>([&executor]() {
    executor.spin();
  });

  // Wait for joint states to be available
  RCLCPP_INFO(LOGGER, "Waiting for system to be ready...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  demo->run();

  RCLCPP_INFO(LOGGER, "Demo completed. Keeping node alive for visualization. Press Ctrl+C to exit.");
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
