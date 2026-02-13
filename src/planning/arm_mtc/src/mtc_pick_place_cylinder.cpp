#include <rclcpp/rclcpp.hpp>
#if __has_include(<moveit/planning_scene/planning_scene.hpp>)
  #include <moveit/planning_scene/planning_scene.hpp>
  #include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#elif __has_include(<moveit/planning_scene/planning_scene.h>)
  #include <moveit/planning_scene/planning_scene.h>
  #include <moveit/planning_scene_interface/planning_scene_interface.h>
#else
  #error "MoveIt planning_scene header not found"
#endif
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

// Object configuration structure
struct ObjectConfig {
  std::string id;
  double radius;
  double height;
  double pick_x, pick_y, pick_z;
  double place_x, place_y, place_z;
};

class MTCPickPlaceCylinder
{
public:
  MTCPickPlaceCylinder(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  bool doTask();

  void setupPlanningScene();
  
  bool loadObjectConfig();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  ObjectConfig object_config_;
};

MTCPickPlaceCylinder::MTCPickPlaceCylinder(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_pick_place_cylinder", options) }
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCPickPlaceCylinder::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

bool MTCPickPlaceCylinder::loadObjectConfig()
{
  // Declare parameters if not already declared (handles both command-line and default cases)
  auto declare_if_not_declared = [this](const std::string& name, const auto& default_value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, default_value);
    }
  };

  declare_if_not_declared("object_id", std::string("target_cylinder"));
  declare_if_not_declared("object_radius", 0.025);
  declare_if_not_declared("object_height", 0.15);
  declare_if_not_declared("pick_x", -0.2);
  declare_if_not_declared("pick_y", 0.5);
  declare_if_not_declared("pick_z", 1.2);
  declare_if_not_declared("place_x", -0.2);
  declare_if_not_declared("place_y", -0.5);
  declare_if_not_declared("place_z", 1.2);

  object_config_.id = node_->get_parameter("object_id").as_string();
  object_config_.radius = node_->get_parameter("object_radius").as_double();
  object_config_.height = node_->get_parameter("object_height").as_double();
  object_config_.pick_x = node_->get_parameter("pick_x").as_double();
  object_config_.pick_y = node_->get_parameter("pick_y").as_double();
  object_config_.pick_z = node_->get_parameter("pick_z").as_double();
  object_config_.place_x = node_->get_parameter("place_x").as_double();
  object_config_.place_y = node_->get_parameter("place_y").as_double();
  object_config_.place_z = node_->get_parameter("place_z").as_double();

  RCLCPP_INFO(LOGGER, "Loaded object config: id=%s, radius=%.3f, height=%.3f",
              object_config_.id.c_str(), object_config_.radius, object_config_.height);
  RCLCPP_INFO(LOGGER, "Pick pose: (%.3f, %.3f, %.3f)",
              object_config_.pick_x, object_config_.pick_y, object_config_.pick_z);
  RCLCPP_INFO(LOGGER, "Place pose: (%.3f, %.3f, %.3f)",
              object_config_.place_x, object_config_.place_y, object_config_.place_z);

  return true;
}

void MTCPickPlaceCylinder::setupPlanningScene()
{
  moveit::planning_interface::PlanningSceneInterface psi;
  
  // Helper to declare parameter if not already declared
  auto declare_if_not_declared = [this](const std::string& name, bool default_value) {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter(name, default_value);
    }
  };
  
  // Check if we should spawn the object (external script may have already done it)
  declare_if_not_declared("spawn_object", true);
  bool spawn_object = node_->get_parameter("spawn_object").as_bool();
  
  if (spawn_object) {
    moveit_msgs::msg::CollisionObject cylinder;
    cylinder.id = object_config_.id;
    cylinder.header.frame_id = "base_link";
    cylinder.primitives.resize(1);
    cylinder.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.primitives[0].dimensions = { object_config_.height, object_config_.radius };

    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.position.x = object_config_.pick_x;
    cylinder_pose.position.y = object_config_.pick_y;
    cylinder_pose.position.z = object_config_.pick_z;
    cylinder_pose.orientation.w = 1.0;
    cylinder.pose = cylinder_pose;

    psi.applyCollisionObject(cylinder);
    RCLCPP_INFO(LOGGER, "Spawned object '%s' at (%.3f, %.3f, %.3f)",
                object_config_.id.c_str(), object_config_.pick_x, 
                object_config_.pick_y, object_config_.pick_z);
  } else {
    RCLCPP_INFO(LOGGER, "Skipping object spawn (spawn_object=false)");
  }
  
  // Check if table should be added
  declare_if_not_declared("spawn_table", true);
  if (node_->get_parameter("spawn_table").as_bool()) {
    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "base_link";
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = { 0.5, 0.5, 0.2 };

    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.3;
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.6;
    table_pose.orientation.w = 1.0;
    table.pose = table_pose;

    psi.applyCollisionObject(table);
    RCLCPP_INFO(LOGGER, "Spawned table");
  }

  RCLCPP_INFO(LOGGER, "Planning scene setup complete");
}

bool MTCPickPlaceCylinder::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
    return false;
  }

  RCLCPP_INFO(LOGGER, "Starting task planning (max 5 solutions)...");
  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return false;
  }

  task_.introspection().publishSolution(*task_.solutions().front());
  RCLCPP_INFO(LOGGER, "Task planning succeeded! %zu solutions found.", task_.solutions().size());

  // Execute the task
  RCLCPP_INFO(LOGGER, "Attempting to execute the task...");
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with error code: " << result.val);
    return false;
  }

  RCLCPP_INFO(LOGGER, "Task executed successfully!");
  return true;
}

mtc::Task MTCPickPlaceCylinder::createTask()
{
  mtc::Task task;
  task.stages()->setName("Pick and Place Cylinder");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "end_effector_link";

  // ========== CONFIGURABLE PARAMETERS ==========
  // Pre-grasp offset: distance from palm origin for IK (must be > 0 to avoid collision)
  // This is where the gripper will be BEFORE the approach
  const double pre_grasp_offset = 0.05;  // 5cm - stand-off distance for IK
  
  // Approach: move from pre_grasp position toward the object
  // Final grasp position = pre_grasp_offset - approach_distance
  const double approach_min_dist = 0.02;  // 2cm minimum approach
  const double approach_max_dist = 0.10;  // 10cm maximum approach
  
  // Grasp offset for place stage (where object center is relative to palm)
  const double grasp_offset = 0.02;  // 2cm - approximate final grasp position
  
  const double lift_min_dist = 0.02;
  const double lift_max_dist = 0.10;
  const double retreat_min_dist = 0.02;
  const double retreat_max_dist = 0.10;

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
        object_config_.id,
        task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
        true);
      // Hand link list already provided by joint model group
      grasp->insert(std::move(stage));
    }

    // 6.2: Generate grasp poses (GENERATOR)
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject(object_config_.id);
      stage->setAngleDelta(M_PI / 6);  // 12 poses around cylinder (30 degrees apart)
      stage->setMonitoredStage(current_state_ptr);

      // Wrap with ComputeIK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(0.1);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      // Pre-grasp frame transform: offset from palm to object center
      // This positions the gripper at a stand-off distance, then approach moves closer
      Eigen::Isometry3d grasp_frame_transform = Eigen::Isometry3d::Identity();
      grasp_frame_transform.translation().y() = pre_grasp_offset;
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);

      grasp->insert(std::move(wrapper));
    }

    // 6.3: Approach object (move gripper toward object along Y-axis of hand frame)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(approach_min_dist, approach_max_dist);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "approach");

      // Move along Y-axis of hand_frame (toward the object)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.y = 1.0;  // Positive Y = toward object (based on your gripper orientation)
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // 6.4: Close gripper (use named pose from SRDF)
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }

    // 6.5: Attach object after gripper closed
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_config_.id, hand_frame);
      attach_object_ptr = stage.get();  // Save pointer for GeneratePlacePose
      grasp->insert(std::move(stage));
    }

    // 6.6: Lift object (move up in base_link Z)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(lift_min_dist, lift_max_dist);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "base_link";
      vec.vector.z = 1.0;  // Lift upward
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
      stage->setObject(object_config_.id);

      // Place pose from parameters
      geometry_msgs::msg::PoseStamped place_pose;
      place_pose.header.frame_id = "base_link";
      place_pose.pose.position.x = object_config_.place_x;
      place_pose.pose.position.y = object_config_.place_y;
      place_pose.pose.position.z = object_config_.place_z;
      place_pose.pose.orientation.w = 1.0;
      stage->setPose(place_pose);
      stage->setMonitoredStage(attach_object_ptr);  // Monitor stage where object is attached

      // Wrap with ComputeIK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(0.1);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });

      // Use same grasp frame transform as picking (configurable offset)
      Eigen::Isometry3d place_frame_transform = Eigen::Isometry3d::Identity();
      place_frame_transform.translation().y() = grasp_offset;
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
        object_config_.id,
        task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
        false);
      stage->allowCollisions(object_config_.id, "left_hand", false);
      place->insert(std::move(stage));
    }

    // 8.4: Detach object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_config_.id, hand_frame);
      place->insert(std::move(stage));
    }

    // 8.5: Retreat from place (move UP in world frame to avoid stacked objects)
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(retreat_min_dist, retreat_max_dist);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Move UP in world frame (avoids collision with stacked cylinders)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;  // Retreat upward
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
  executor.add_node(mtc_node->getNodeBaseInterface());

  // Spin in background for service calls
  auto spin_thread = std::make_unique<std::thread>([&executor]() {
    executor.spin();
  });

  // Wait for system initialization
  RCLCPP_INFO(LOGGER, "Waiting for system initialization (3 seconds)...");
  rclcpp::sleep_for(std::chrono::seconds(3));

  // Load object configuration from parameters
  if (!mtc_node->loadObjectConfig()) {
    RCLCPP_ERROR(LOGGER, "Failed to load object configuration");
    executor.cancel();
    spin_thread->join();
    rclcpp::shutdown();
    return 1;
  }

  mtc_node->setupPlanningScene();

  RCLCPP_INFO(LOGGER, "Waiting 1 second for planning scene to update...");
  rclcpp::sleep_for(std::chrono::seconds(1));

  bool success = mtc_node->doTask();

  // Task complete - shutdown cleanly
  RCLCPP_INFO(LOGGER, "Task complete, shutting down...");
  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
