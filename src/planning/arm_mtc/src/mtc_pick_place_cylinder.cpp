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

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_pick_place_cylinder");
namespace mtc = moveit::task_constructor;

class MTCPickPlaceCylinder
{
public:
  MTCPickPlaceCylinder(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

MTCPickPlaceCylinder::MTCPickPlaceCylinder(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_pick_place_cylinder", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCPickPlaceCylinder::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MTCPickPlaceCylinder::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject cylinder;
  cylinder.id = "target_cylinder";
  cylinder.header.frame_id = "base_link";
  cylinder.primitives.resize(1);
  cylinder.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder.primitives[0].dimensions = { 0.15, 0.025 };  // height: 15cm, radius: 2.5cm

  geometry_msgs::msg::Pose cylinder_pose;
  cylinder_pose.position.x = -0.2;   // 30cm in front of robot
  cylinder_pose.position.y = 0.5;   // Centered
  cylinder_pose.position.z = 1.2; // On table surface: table_top(0.7) + half_cylinder_height(0.075)

  // Cylinder orientation: identity quaternion means:
  // - X-axis points forward (same as base_link X)
  // - Y-axis points left (same as base_link Y)
  // - Z-axis points up (cylinder standing vertical)
  cylinder_pose.orientation.x = 0.0;
  cylinder_pose.orientation.y = 0.0;
  cylinder_pose.orientation.z = 0.0;
  cylinder_pose.orientation.w = 1.0;
  cylinder.pose = cylinder_pose;

  // Add table as collision object
  moveit_msgs::msg::CollisionObject table;
  table.id = "table";
  table.header.frame_id = "base_link";
  table.primitives.resize(1);
  table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  table.primitives[0].dimensions = { 0.5, 0.5, 0.2 };  // width, depth, height

  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0.3;   // In front of robot, aligned with cylinder
  table_pose.position.y = 0.0;   // Centered
  table_pose.position.z = 0.6;   // Table surface at z=0.7 (half_height=0.1 + 0.6)
  table_pose.orientation.w = 1.0;
  table.pose = table_pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(cylinder);
  psi.applyCollisionObject(table);

  RCLCPP_INFO(LOGGER, "Planning scene setup complete: cylinder and table added");
}

void MTCPickPlaceCylinder::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
    return;
  }

  RCLCPP_INFO(LOGGER, "Starting task planning (max 5 solutions)...");
  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  task_.introspection().publishSolution(*task_.solutions().front());
  RCLCPP_INFO(LOGGER, "Task planning succeeded! %zu solutions found.", task_.solutions().size());

  // Execute the task
  RCLCPP_INFO(LOGGER, "Attempting to execute the task...");
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with error code: " << result.val);
    return;
  }

  RCLCPP_INFO(LOGGER, "Task executed successfully!");
}

mtc::Task MTCPickPlaceCylinder::createTask()
{
  mtc::Task task;
  task.stages()->setName("Pick and Place Cylinder");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "left_palm";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Create planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  // Forward current_state to later stages
  mtc::Stage* current_state_ptr = nullptr;
  mtc::Stage* attach_object_ptr = nullptr;  // For GeneratePlacePose to monitor

  // ========== STAGE 1: Current State ==========
  {
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = current_state.get();
    task.add(std::move(current_state));
  }

  // ========== STAGE 2: Move to Home Position ==========
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to home", sampling_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  // ========== STAGE 3: Open Gripper ==========
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("open");
    task.add(std::move(stage));
  }

  // ========== STAGE 4: Move to Ready Pose ==========
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move to ready", sampling_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  // ========== STAGE 5: Connect to Grasp Pose ==========
  // Use Connect stage to bridge from current state to the grasp pose generator
  {
    auto connect = std::make_unique<mtc::stages::Connect>(
      "move to grasp",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    connect->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(connect));
  }

  // ========== STAGE 6: Grasp (SerialContainer with Generator) ==========
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("grasp");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // 6.1: Allow collision FIRST (needed for grasp pose IK to succeed)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions(
        "target_cylinder",
        task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
        true);
      stage->allowCollisions("target_cylinder", "left_hand", true);
      grasp->insert(std::move(stage));
    }

    // 6.2: Generate grasp poses (GENERATOR)
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("target_cylinder");
      stage->setAngleDelta(M_PI / 6);  // 12 poses around cylinder (30 degrees apart)
      stage->setMonitoredStage(current_state_ptr);

      // IMPORTANT: GenerateGraspPose for cylinders creates poses where:
      // - The pose rotates around the cylinder's Z-axis at different angles
      // - X-axis points radially INWARD (toward cylinder center)
      // - Y-axis points tangentially (perpendicular to radius)
      // - Z-axis points along cylinder axis (upward for standing cylinder)
      //
      // If the grey gripper doesn't look radially aligned, it means the cylinder's
      // frame orientation is not identity, or GenerateGraspPose is using a different convention

      // Wrap with ComputeIK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(0.1);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      // Grasp frame transform: defines where on the gripper the object center should be
      // The red target gripper is already correctly oriented radially around cylinder
      // So we only need translation offset, NO rotation
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();

      // Translation: 8cm offset to position object center between fingers
      // This is the distance from left_palm origin to where the object should be grasped
      grasp_frame_transform.translation().y() = 0.08;

      wrapper->setIKFrame(grasp_frame_transform, hand_frame);

      grasp->insert(std::move(wrapper));
    }

    // 6.3: Close gripper around cylinder
    // Calculate grip position based on cylinder radius (2.5cm = 0.025m)
    // Fingers move along X axis, need to leave ~2.5cm gap on each side
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
      stage->setGroup(hand_group_name);
      
      // Open: left=0.033, right=-0.033 (fingers apart)
      // Close: left=-0.0041, right=0.0002 (fingers together)
      // For 2.5cm radius cylinder, set fingers to grip around it
      std::map<std::string, double> gripper_grasp;
      gripper_grasp["left_palm_left_finger"] = 0.025;   // More open
      gripper_grasp["left_palm_right_finger"] = -0.025; // More open
      stage->setGoal(gripper_grasp);
      
      grasp->insert(std::move(stage));
    }

    // 6.4: Attach object after gripper closed
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("target_cylinder", hand_frame);
      attach_object_ptr = stage.get();  // Save pointer for GeneratePlacePose
      grasp->insert(std::move(stage));
    }

    // 6.5: Lift object (move up in base_link Z)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.01, 0.05);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base_link";
      vec.vector.z = 0.05;  // Lift 5cm
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  // ========== STAGE 7: Connect to Place Position ==========
  {
    auto connect = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    connect->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(connect));
  }

  // ========== STAGE 8: Place (SerialContainer) ==========
  {
    auto place = std::make_unique<mtc::SerialContainer>("place");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

    // 8.1: Generate place pose
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("target_cylinder");

      // Place pose: x=-0.2, y=-0.5, z=1.2
      geometry_msgs::msg::PoseStamped place_pose;
      place_pose.header.frame_id = "base_link";
      place_pose.pose.position.x = -0.2;
      place_pose.pose.position.y = -0.5;
      place_pose.pose.position.z = 1.2;
      place_pose.pose.orientation.w = 1.0;
      stage->setPose(place_pose);
      stage->setMonitoredStage(attach_object_ptr);  // Monitor stage where object is attached

      // Wrap with ComputeIK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(0.1);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      // Same grasp frame transform as picking
      Eigen::Isometry3d place_frame_transform = Eigen::Isometry3d::Identity();
      place_frame_transform.translation().y() = 0.08;
      wrapper->setIKFrame(place_frame_transform, hand_frame);

      place->insert(std::move(wrapper));
    }

    // 8.2: Open gripper to release
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("release gripper", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    // 8.3: Forbid collision
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions(
        "target_cylinder",
        task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
        false);
      stage->allowCollisions("target_cylinder", "left_hand", false);
      place->insert(std::move(stage));
    }

    // 8.4: Detach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("target_cylinder", hand_frame);
      place->insert(std::move(stage));
    }

    // 8.5: Retreat from place (move back)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.02, 0.10);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.y = -1.0;  // Retreat away from object
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  // ========== STAGE 9: Return Home ==========
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("home");
    task.add(std::move(stage));
  }

  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_node = std::make_shared<MTCPickPlaceCylinder>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_node]() {
    executor.add_node(mtc_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_node->getNodeBaseInterface());
  });

  // Wait for system initialization
  RCLCPP_INFO(LOGGER, "Waiting for system initialization (5 seconds)...");
  rclcpp::sleep_for(std::chrono::seconds(5));

  mtc_node->setupPlanningScene();

  RCLCPP_INFO(LOGGER, "Waiting 2 seconds for planning scene to update...");
  rclcpp::sleep_for(std::chrono::seconds(2));

  mtc_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
