/**
 * @file mtc_node.cpp
 * @brief MoveIt Task Constructor Pick and Place Node for LDR Humanoid Arm
 *
 * This node implements a complete pick-and-place task using MoveIt Task Constructor (MTC).
 * It demonstrates complex manipulation with the 6-DOF arm and 2-finger gripper.
 */

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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

namespace mtc = moveit::task_constructor;

/**
 * @brief Helper function to convert parameter vector to Eigen transform
 */
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
  if (values.size() != 6) {
    throw std::runtime_error("Transform vector must have 6 values [x, y, z, roll, pitch, yaw]");
  }
  return Eigen::Translation3d(values[0], values[1], values[2]) *
         Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}

/**
 * @brief Helper function to convert parameter vector to Pose message
 */
geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values) {
  return tf2::toMsg(vectorToEigen(values));
}

/**
 * @brief Main MTC Task Node class
 */
class MTCTaskNode : public rclcpp::Node {
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  void setupPlanningScene();
  void doTask();

private:
  mtc::Task createTask();
  mtc::Task task_;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : Node("mtc_node", options) {
  RCLCPP_INFO(this->get_logger(), "MTC Node initialized");
}

void MTCTaskNode::setupPlanningScene() {
  RCLCPP_INFO(this->get_logger(), "Setting up planning scene...");

  // Wait for planning scene to be available
  RCLCPP_INFO(this->get_logger(), "Waiting for planning scene to be ready...");
  rclcpp::sleep_for(std::chrono::seconds(2));

  // Create planning scene interface
  moveit::planning_interface::PlanningSceneInterface psi;

  // Get parameters
  auto object_name = this->get_parameter("object_name").as_string();
  auto object_type = this->get_parameter("object_type").as_string();
  auto object_dimensions = this->get_parameter("object_dimensions").as_double_array();
  auto object_pose = this->get_parameter("object_pose").as_double_array();
  auto object_reference_frame = this->get_parameter("object_reference_frame").as_string();

  auto surface_name = this->get_parameter("surface_name").as_string();
  auto surface_dimensions = this->get_parameter("surface_dimensions").as_double_array();
  auto surface_pose = this->get_parameter("surface_pose").as_double_array();

  // Create collision objects
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

  // Add support surface (table)
  {
    moveit_msgs::msg::CollisionObject surface;
    surface.id = surface_name;
    surface.header.frame_id = object_reference_frame;
    surface.primitives.resize(1);
    surface.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    surface.primitives[0].dimensions = {surface_dimensions[0], surface_dimensions[1], surface_dimensions[2]};
    surface.primitive_poses.resize(1);
    surface.primitive_poses[0] = vectorToPose(surface_pose);
    surface.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_objects.push_back(surface);
  }

  // Add target object
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = object_name;
    object.header.frame_id = object_reference_frame;
    object.primitives.resize(1);

    if (object_type == "cylinder") {
      object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      object.primitives[0].dimensions = {object_dimensions[0], object_dimensions[1]};  // height, radius
    } else if (object_type == "box") {
      object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
      object.primitives[0].dimensions = {object_dimensions[0], object_dimensions[1], object_dimensions[2]};
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown object type: %s", object_type.c_str());
      return;
    }

    object.primitive_poses.resize(1);
    object.primitive_poses[0] = vectorToPose(object_pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_objects.push_back(object);
  }

  // Apply collision objects to planning scene
  RCLCPP_INFO(this->get_logger(), "Adding %zu collision objects to planning scene", collision_objects.size());
  if (!psi.applyCollisionObjects(collision_objects)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add collision objects to planning scene");
    return;
  }

  // Give time for objects to be added
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Verify objects were added
  auto known_objects = psi.getKnownObjectNames();
  RCLCPP_INFO(this->get_logger(), "Known objects in planning scene: %zu", known_objects.size());
  for (const auto& obj : known_objects) {
    RCLCPP_INFO(this->get_logger(), "  - %s", obj.c_str());
  }

  bool object_found = std::find(known_objects.begin(), known_objects.end(), object_name) != known_objects.end();
  bool surface_found = std::find(known_objects.begin(), known_objects.end(), surface_name) != known_objects.end();

  if (!object_found) {
    RCLCPP_WARN(this->get_logger(), "Target object '%s' not found in planning scene!", object_name.c_str());
  }
  if (!surface_found) {
    RCLCPP_WARN(this->get_logger(), "Support surface '%s' not found in planning scene!", surface_name.c_str());
  }

  if (object_found && surface_found) {
    RCLCPP_INFO(this->get_logger(), "Planning scene setup complete - all objects added successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planning scene setup incomplete - missing objects");
  }
}

mtc::Task MTCTaskNode::createTask() {
  mtc::Task task("pick_place_task");
  task.loadRobotModel(shared_from_this());

  // Enable introspection for RViz Task Monitor
  // This publishes to /pick_place_task/solution
  task.enableIntrospection();

  // Get all parameters
  const auto arm_group_name = this->get_parameter("arm_group_name").as_string();
  const auto gripper_group_name = this->get_parameter("gripper_group_name").as_string();
  const auto gripper_frame = this->get_parameter("gripper_frame").as_string();
  const auto gripper_open_pose = this->get_parameter("gripper_open_pose").as_string();
  const auto gripper_close_pose = this->get_parameter("gripper_close_pose").as_string();
  const auto arm_home_pose = this->get_parameter("arm_home_pose").as_string();
  const auto world_frame = this->get_parameter("world_frame").as_string();
  const auto object_name = this->get_parameter("object_name").as_string();
  const auto support_surface_id = this->get_parameter("surface_name").as_string();

  const auto controller_names = this->get_parameter("controller_names").as_string_array();

  const auto object_dimensions = this->get_parameter("object_dimensions").as_double_array();
  const auto grasp_frame_transform = this->get_parameter("grasp_frame_transform").as_double_array();
  const auto place_pose = this->get_parameter("place_pose").as_double_array();

  const auto approach_object_min_dist = this->get_parameter("approach_object_min_dist").as_double();
  const auto approach_object_max_dist = this->get_parameter("approach_object_max_dist").as_double();
  const auto lift_object_min_dist = this->get_parameter("lift_object_min_dist").as_double();
  const auto lift_object_max_dist = this->get_parameter("lift_object_max_dist").as_double();
  const auto lower_object_min_dist = this->get_parameter("lower_object_min_dist").as_double();
  const auto lower_object_max_dist = this->get_parameter("lower_object_max_dist").as_double();
  const auto retreat_min_distance = this->get_parameter("retreat_min_distance").as_double();
  const auto retreat_max_distance = this->get_parameter("retreat_max_distance").as_double();

  const auto move_to_pick_timeout = this->get_parameter("move_to_pick_timeout").as_double();
  const auto move_to_place_timeout = this->get_parameter("move_to_place_timeout").as_double();

  const auto grasp_pose_angle_delta = this->get_parameter("grasp_pose_angle_delta").as_double();
  const auto grasp_pose_max_ik_solutions = this->get_parameter("grasp_pose_max_ik_solutions").as_int();
  const auto grasp_pose_min_solution_distance = this->get_parameter("grasp_pose_min_solution_distance").as_double();
  const auto place_pose_max_ik_solutions = this->get_parameter("place_pose_max_ik_solutions").as_int();

  const auto cartesian_max_velocity_scaling = this->get_parameter("cartesian_max_velocity_scaling").as_double();
  const auto cartesian_max_acceleration_scaling = this->get_parameter("cartesian_max_acceleration_scaling").as_double();
  const auto cartesian_step_size = this->get_parameter("cartesian_step_size").as_double();

  const auto place_pose_z_offset_factor = this->get_parameter("place_pose_z_offset_factor").as_double();
  const double object_height = object_dimensions[0];  // First dimension is height for cylinder

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", gripper_group_name);
  task.setProperty("ik_frame", gripper_frame);

  // Create planners (Humble MTC API)
  auto ompl_planner_arm = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(cartesian_max_velocity_scaling);
  cartesian_planner->setMaxAccelerationScalingFactor(cartesian_max_acceleration_scaling);
  cartesian_planner->setStepSize(cartesian_step_size);

  // ===========================
  // Current State
  // ===========================
  mtc::Stage* current_state_ptr = nullptr;
  {
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));
  }

  // ===========================
  // Open Gripper
  // ===========================
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
    stage->setGroup(gripper_group_name);
    stage->setGoal(gripper_open_pose);
    task.add(std::move(stage));
  }

  // ===========================
  // Move to Pick
  // ===========================
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{
        {arm_group_name, ompl_planner_arm},
        {gripper_group_name, interpolation_planner}
      });
    stage->setTimeout(move_to_pick_timeout);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // ===========================
  // Pick Object Container
  // ===========================
  mtc::Stage* attach_object_stage = nullptr;
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // Approach Object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", gripper_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(approach_object_min_dist, approach_object_max_dist);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = gripper_frame;
      vec.vector.z = 1.0;  // Approach along gripper's z-axis
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // Generate Grasp Pose
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->setPreGraspPose(gripper_open_pose);
      stage->setObject(object_name);
      stage->setAngleDelta(grasp_pose_angle_delta);
      stage->setMonitoredStage(current_state_ptr);

      // Compute IK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(grasp_pose_max_ik_solutions);
      wrapper->setMinSolutionDistance(grasp_pose_min_solution_distance);
      wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), gripper_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    // Allow Collision (gripper-object)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,object)");
      stage->allowCollisions(
        object_name,
        task.getRobotModel()
          ->getJointModelGroup(gripper_group_name)
          ->getLinkModelNamesWithCollisionGeometry(),
        true);
      grasp->insert(std::move(stage));
    }

    // Close Gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
      stage->setGroup(gripper_group_name);
      stage->setGoal(gripper_close_pose);
      grasp->insert(std::move(stage));
    }

    // Allow Collision (object-surface)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,support)");
      stage->allowCollisions({object_name}, {support_surface_id}, true);
      grasp->insert(std::move(stage));
    }

    // Attach Object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject(object_name, gripper_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    // Re-allow Collision (gripper-object) after attachment
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("re-allow collision (gripper,attached object)");
      stage->allowCollisions(
        object_name,
        task.getRobotModel()
          ->getJointModelGroup(gripper_group_name)
          ->getLinkModelNamesWithCollisionGeometry(),
        true);
      grasp->insert(std::move(stage));
    }

    // Lift Object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(lift_object_min_dist, lift_object_max_dist);
      stage->setIKFrame(gripper_frame);
      stage->properties().set("marker_ns", "lift_object");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame;
      vec.vector.z = 1.0;  // Lift upward
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    // Forbid Collision (object-surface)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,support)");
      stage->allowCollisions({object_name}, {support_surface_id}, false);
      grasp->insert(std::move(stage));
    }

    task.add(std::move(grasp));
  }

  // ===========================
  // Move to Place
  // ===========================
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
      "move to place",
      mtc::stages::Connect::GroupPlannerVector{
        {arm_group_name, ompl_planner_arm},
        {gripper_group_name, interpolation_planner}
      });
    stage->setTimeout(move_to_place_timeout);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // ===========================
  // Place Object Container
  // ===========================
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // Lower Object
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
      stage->properties().set("marker_ns", "lower_object");
      stage->properties().set("link", gripper_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(lower_object_min_dist, lower_object_max_dist);

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame;
      vec.vector.z = -1.0;  // Lower downward
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    // Generate Place Pose
    {
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"ik_frame"});
      stage->setObject(object_name);

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = world_frame;
      target_pose_msg.pose = vectorToPose(place_pose);
      target_pose_msg.pose.position.z += place_pose_z_offset_factor * object_height;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);

      // Compute IK
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(place_pose_max_ik_solutions);
      wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), gripper_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    // Open Gripper
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
      stage->setGroup(gripper_group_name);
      stage->setGoal(gripper_open_pose);
      place->insert(std::move(stage));
    }

    // Forbid Collision (gripper-object)
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (gripper,object)");
      stage->allowCollisions(
        object_name,
        task.getRobotModel()
          ->getJointModelGroup(gripper_group_name)
          ->getLinkModelNamesWithCollisionGeometry(),
        false);
      place->insert(std::move(stage));
    }

    // Detach Object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_name, gripper_frame);
      place->insert(std::move(stage));
    }

    // Retreat
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat after place", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(retreat_min_distance, retreat_max_distance);
      stage->setIKFrame(gripper_frame);
      stage->properties().set("marker_ns", "retreat");

      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = gripper_frame;
      vec.vector.z = -1.0;  // Retreat backward
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    task.add(std::move(place));
  }

  // ===========================
  // Move Home
  // ===========================
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move home", ompl_planner_arm);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal(arm_home_pose);
    task.add(std::move(stage));
  }

  return task;
}

void MTCTaskNode::doTask() {
  RCLCPP_INFO(this->get_logger(), "Starting pick and place task");

  // Create task
  task_ = createTask();

  // Get parameters
  auto execute = this->get_parameter("execute").as_bool();
  auto max_solutions = this->get_parameter("max_solutions").as_int();

  // Initialize task
  try {
    task_.init();
    RCLCPP_INFO(this->get_logger(), "Task initialized successfully");
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR(this->get_logger(), "Task initialization failed: %s", e.what());
    return;
  }

  // Plan task
  RCLCPP_INFO(this->get_logger(), "Planning task...");
  if (!task_.plan(max_solutions)) {
    RCLCPP_ERROR(this->get_logger(), "Task planning failed");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Task planning succeeded with %zu solutions", task_.solutions().size());

  // Publish solution for visualization
  task_.introspection().publishSolution(*task_.solutions().front());
  RCLCPP_INFO(this->get_logger(), "Published solution for visualization in RViz");

  // Execute if requested
  if (execute) {
    RCLCPP_INFO(this->get_logger(), "Executing planned task");
    auto result = task_.execute(*task_.solutions().front());

    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Task execution failed with error code: %d", result.val);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Task executed successfully!");
  } else {
    RCLCPP_INFO(this->get_logger(), "Execution skipped (execute parameter is false)");
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  int ret = 0;

  try {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mtc_task_node);

    try {
      RCLCPP_INFO(mtc_task_node->get_logger(), "Setting up planning scene");
      mtc_task_node->setupPlanningScene();

      RCLCPP_INFO(mtc_task_node->get_logger(), "Executing task");
      mtc_task_node->doTask();

      RCLCPP_INFO(mtc_task_node->get_logger(),
        "Task completed. Keeping node alive for visualization. Press Ctrl+C to exit.");

      executor.spin();
    } catch (const std::runtime_error& e) {
      RCLCPP_ERROR(mtc_task_node->get_logger(), "Runtime error: %s", e.what());
      ret = 1;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Error during node setup: %s", e.what());
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}
