# MoveIt Task Constructor (MTC) Implementation Guide

This document provides a comprehensive guide to implementing MoveIt Task Constructor (MTC) for pick-and-place operations, extracted from a working myCobot 280 ROS2 project.

## Table of Contents
1. [Architecture Overview](#architecture-overview)
2. [Core MTC Concepts](#core-mtc-concepts)
3. [Pick and Place Implementation](#pick-and-place-implementation)
4. [Motion Planners](#motion-planners)
5. [Fallback Strategies](#fallback-strategies)
6. [Configuration System](#configuration-system)
7. [Perception Integration](#perception-integration)
8. [Implementation Patterns](#implementation-patterns)
9. [Code Examples](#code-examples)

---

## Architecture Overview

The implementation uses **MoveIt Task Constructor (MTC)** to create complex manipulation tasks by composing sequential stages.

### Two Main Implementations

1. **Pick and Place Demo**: Full pick-and-place pipeline with perception integration
2. **Fallback Strategies Demo**: Demonstrates multiple planning strategies with fallbacks

### Key Files Structure
```
mycobot_mtc_pick_place_demo/
├── src/
│   ├── mtc_node.cpp                        # Main pick-and-place implementation
│   ├── get_planning_scene_server.cpp       # Perception service for scene generation
│   └── get_planning_scene_client.cpp       # Service client
├── include/
│   └── get_planning_scene_client.h
├── config/
│   └── mtc_node_params.yaml               # All tunable parameters
└── launch/
    └── pick_place_demo.launch.py          # Launch configuration

mycobot_mtc_demos/
├── src/
│   └── fallbacks_move_to.cpp              # Fallback strategies demo
└── launch/
    └── mtc_demos.launch.py
```

---

## Core MTC Concepts

### 1. Task Structure

MTC tasks are built using a **stage-based architecture**:
- **Stages**: Individual motion planning steps
- **Containers**: Organize stages (SerialContainer, Alternatives, Fallbacks)
- **Solvers**: Handle motion planning (Cartesian, OMPL, Pilz)

### 2. Stage Types

- **Generator stages**: Create states independently (e.g., `GenerateGraspPose`)
- **Propagator stages**: Plan from/to states (e.g., `MoveTo`, `MoveRelative`)
- **Connect stages**: Bridge between states from different planning groups
- **ModifyPlanningScene stages**: Update collision objects, attach/detach objects

### 3. Property System

Properties are inherited through the stage hierarchy:
```cpp
task.setProperty("group", arm_group_name);              // Planning group
task.setProperty("eef", gripper_group_name);            // End-effector
task.setProperty("ik_frame", gripper_frame);            // IK reference frame
stage->properties().configureInitFrom(mtc::Stage::PARENT);  // Inherit from parent
```

---

## Pick and Place Implementation

### Node Structure

```cpp
class MTCTaskNode : public rclcpp::Node {
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  void doTask();              // Main execution method
  void setupPlanningScene();  // Sets up collision objects

private:
  mtc::Task task_;
  mtc::Task createTask();     // Builds the task stages
};
```

### Complete Task Pipeline

The pick-and-place task follows this sequential flow:

#### 1. Current State
```cpp
mtc::Stage* current_state_ptr = nullptr;
auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current state");
current_state_ptr = stage_state_current.get();
task.add(std::move(stage_state_current));
```

#### 2. Open Gripper
```cpp
auto stage_open_gripper = std::make_unique<mtc::stages::MoveTo>(
    "open gripper", interpolation_planner);
stage_open_gripper->setGroup(gripper_group_name);
stage_open_gripper->setGoal(gripper_open_pose);
stage_open_gripper->properties().set("trajectory_execution_info",
    mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
task.add(std::move(stage_open_gripper));
```

#### 3. Move to Pick
```cpp
auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
    "move to pick",
    mtc::stages::Connect::GroupPlannerVector{
      {arm_group_name, ompl_planner_arm},
      {gripper_group_name, interpolation_planner}
    });
stage_move_to_pick->setTimeout(move_to_pick_timeout);
stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
task.add(std::move(stage_move_to_pick));
```

#### 4. Pick Object (Serial Container)

The pick operation is a `SerialContainer` with multiple sub-stages:

```cpp
auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
```

**4a. Approach Object**
```cpp
auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
stage->properties().set("marker_ns", "approach_object");
stage->properties().set("link", gripper_frame);
stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
stage->setMinMaxDistance(approach_object_min_dist, approach_object_max_dist);

geometry_msgs::msg::Vector3Stamped vec;
vec.header.frame_id = gripper_frame;
vec.vector.z = 1.0;  // Approach along gripper's z-axis
stage->setDirection(vec);
grasp->insert(std::move(stage));
```

**4b. Generate Grasp Pose**
```cpp
// Generator stage that samples grasp poses around the object
auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
stage->properties().configureInitFrom(mtc::Stage::PARENT);
stage->setPreGraspPose(gripper_open_pose);
stage->setObject(object_name);
stage->setAngleDelta(0.1309);  // ~7.5 degrees
stage->setMonitoredStage(current_state_ptr);

// Wrap with IK computation
auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
wrapper->setMaxIKSolutions(10);
wrapper->setMinSolutionDistance(0.8);
wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), gripper_frame);
wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
grasp->insert(std::move(wrapper));
```

**4c. Allow Collision (Gripper-Object)**
```cpp
auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
    "allow collision (gripper,object)");
stage->allowCollisions(
    object_name,
    task.getRobotModel()
        ->getJointModelGroup(gripper_group_name)
        ->getLinkModelNamesWithCollisionGeometry(),
    true);
grasp->insert(std::move(stage));
```

**4d. Close Gripper**
```cpp
auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
stage->setGroup(gripper_group_name);
stage->setGoal(gripper_close_pose);
grasp->insert(std::move(stage));
```

**4e. Allow Collision (Object-Surface)**
```cpp
auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
    "allow collision (object,support_surface)");
stage->allowCollisions({ object_name }, {support_surface_id}, true);
grasp->insert(std::move(stage));
```

**4f. Attach Object**
```cpp
auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
stage->attachObject(object_name, gripper_frame);
attach_object_stage = stage.get();  // Save pointer for place pose generation
grasp->insert(std::move(stage));
```

**4g. Lift Object**
```cpp
auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
stage->setMinMaxDistance(lift_object_min_dist, lift_object_max_dist);
stage->setIKFrame(gripper_frame);

geometry_msgs::msg::Vector3Stamped vec;
vec.header.frame_id = world_frame;
vec.vector.z = 1.0;  // Lift straight up
stage->setDirection(vec);
grasp->insert(std::move(stage));
```

**4h. Forbid Collision (Object-Surface)**
```cpp
auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
    "forbid collision (object,support_surface)");
stage->allowCollisions({ object_name }, {support_surface_id}, false);
grasp->insert(std::move(stage));
```

Add the grasp container to task:
```cpp
task.add(std::move(grasp));
```

#### 5. Move to Place
```cpp
auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
    "move to place",
    mtc::stages::Connect::GroupPlannerVector{
      {arm_group_name, ompl_planner_arm},
      {gripper_group_name, interpolation_planner}
    });
stage_move_to_place->setTimeout(move_to_place_timeout);
stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
task.add(std::move(stage_move_to_place));
```

#### 6. Place Object (Serial Container)

```cpp
auto place = std::make_unique<mtc::SerialContainer>("place object");
task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
place->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });
```

**6a. Lower Object**
```cpp
auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
stage->properties().set("link", gripper_frame);
stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
stage->setMinMaxDistance(lower_object_min_dist, lower_object_max_dist);

geometry_msgs::msg::Vector3Stamped vec;
vec.header.frame_id = world_frame;
vec.vector.z = -1.0;  // Lower downward
stage->setDirection(vec);
place->insert(std::move(stage));
```

**6b. Generate Place Pose**
```cpp
auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
stage->properties().configureInitFrom(mtc::Stage::PARENT, { "ik_frame" });
stage->setObject(object_name);

// Set target pose
geometry_msgs::msg::PoseStamped target_pose_msg;
target_pose_msg.header.frame_id = world_frame;
target_pose_msg.pose = vectorToPose(place_pose);
target_pose_msg.pose.position.z += 0.5 * object_height;  // Offset for object height
stage->setPose(target_pose_msg);
stage->setMonitoredStage(attach_object_stage);  // Monitor successful grasps

// Compute IK
auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
wrapper->setMaxIKSolutions(10);
wrapper->setIKFrame(vectorToEigen(grasp_frame_transform), gripper_frame);
wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
place->insert(std::move(wrapper));
```

**6c. Open Gripper**
```cpp
auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
stage->setGroup(gripper_group_name);
stage->setGoal(gripper_open_pose);
place->insert(std::move(stage));
```

**6d. Forbid Collision (Gripper-Object)**
```cpp
auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
    "forbid collision (gripper,object)");
stage->allowCollisions(object_name,
    *task.getRobotModel()->getJointModelGroup(gripper_group_name), false);
place->insert(std::move(stage));
```

**6e. Detach Object**
```cpp
auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
stage->detachObject(object_name, gripper_frame);
place->insert(std::move(stage));
```

**6f. Retreat**
```cpp
auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat after place", cartesian_planner);
stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
stage->setMinMaxDistance(retreat_min_distance, retreat_max_distance);
stage->setIKFrame(gripper_frame);

geometry_msgs::msg::Vector3Stamped vec;
vec.header.frame_id = gripper_frame;
vec.vector.z = -1.0;  // Retreat along gripper's negative z-axis
stage->setDirection(vec);
place->insert(std::move(stage));
```

Add the place container:
```cpp
task.add(std::move(place));
```

#### 7. Move Home
```cpp
auto stage = std::make_unique<mtc::stages::MoveTo>("move home", ompl_planner_arm);
stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
stage->setGoal(arm_home_pose);
task.add(std::move(stage));
```

---

## Motion Planners

### Three Planner Types

#### 1. OMPL Planner (Complex paths with obstacles)
```cpp
std::unordered_map<std::string, std::string> ompl_map_arm = {
  {"ompl", arm_group_name + "[RRTConnectkConfigDefault]"}
};
auto ompl_planner_arm = std::make_shared<mtc::solvers::PipelinePlanner>(
    this->shared_from_this(), ompl_map_arm);
```

**Use for**:
- Move to pick
- Move to place
- Move home
- Any motion requiring obstacle avoidance

#### 2. Joint Interpolation Planner (Simple movements)
```cpp
auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
```

**Use for**:
- Gripper open/close
- Simple joint space movements without obstacles

#### 3. Cartesian Planner (Straight-line motions)
```cpp
auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
cartesian_planner->setMaxVelocityScalingFactor(1.0);
cartesian_planner->setMaxAccelerationScalingFactor(1.0);
cartesian_planner->setStepSize(0.00025);
```

**Use for**:
- Approach object
- Lift object
- Lower object
- Retreat motions

---

## Fallback Strategies

### Alternatives Container

Used to try multiple initial states:

```cpp
auto initial_alternatives = std::make_unique<Alternatives>("initial states");

// Option 1
{
  auto fixed = std::make_unique<stages::FixedState>("state 1");
  auto scene = initial_scene->diff();
  scene->getCurrentStateNonConst().setVariablePositions({{"joint1", value1}});
  fixed->setState(scene);
  initial_alternatives->add(std::move(fixed));
}

// Option 2
{
  auto fixed = std::make_unique<stages::FixedState>("state 2");
  auto scene = initial_scene->diff();
  scene->getCurrentStateNonConst().setVariablePositions({{"joint1", value2}});
  fixed->setState(scene);
  initial_alternatives->add(std::move(fixed));
}

task.add(std::move(initial_alternatives));
```

### Fallbacks Container

Tries planners in sequence until one succeeds:

```cpp
auto fallbacks = std::make_unique<Fallbacks>("move to target");

// Try Cartesian first (fastest)
{
  auto move_to = std::make_unique<stages::MoveTo>("Cartesian path", cartesian_planner);
  move_to->setGroup("arm");
  move_to->setGoal(target_state);
  fallbacks->add(std::move(move_to));
}

// Try Pilz second (moderate complexity)
{
  auto move_to = std::make_unique<stages::MoveTo>("Pilz path", pilz_planner);
  move_to->setGroup("arm");
  move_to->setGoal(target_state);
  fallbacks->add(std::move(move_to));
}

// Try OMPL last (most robust but slowest)
{
  auto move_to = std::make_unique<stages::MoveTo>("OMPL path", ompl_planner);
  move_to->setGroup("arm");
  move_to->setGoal(target_state);
  fallbacks->add(std::move(move_to));
}

task.add(std::move(fallbacks));
```

---

## Configuration System

### Parameter Categories

**mtc_node_params.yaml structure**:

```yaml
mtc_node:
  ros__parameters:
    # General parameters
    execute: false
    max_solutions: 25

    # Controller parameters
    controller_names:
      - "arm_controller"
      - "grip_action_controller"

    # Robot configuration
    arm_group_name: "arm"
    gripper_group_name: "gripper"
    gripper_frame: "link6_flange"
    gripper_open_pose: "open"
    gripper_close_pose: "half_closed"
    arm_home_pose: "home"

    # Scene frame
    world_frame: "base_link"

    # Object parameters
    object_name: "object"
    object_type: "cylinder"
    object_dimensions: [0.35, 0.0125]  # [height, radius] for cylinder
    object_pose: [0.22, 0.12, 0.0, 0.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]

    # Grasp and place parameters
    grasp_frame_transform: [0.0, 0.0, 0.096, 1.5708, 0.0, 0.0]
    place_pose: [-0.183, -0.14, 0.0, 0.0, 0.0, 0.0]

    # Motion planning parameters
    approach_object_min_dist: 0.0015
    approach_object_max_dist: 0.3
    lift_object_min_dist: 0.005
    lift_object_max_dist: 0.3
    lower_object_min_dist: 0.005
    lower_object_max_dist: 0.4

    # Timeout parameters
    move_to_pick_timeout: 10.0
    move_to_place_timeout: 10.0

    # Grasp generation parameters
    grasp_pose_angle_delta: 0.1309  # ~7.5 degrees
    grasp_pose_max_ik_solutions: 10
    grasp_pose_min_solution_distance: 0.8

    # Place generation parameters
    place_pose_max_ik_solutions: 10

    # Cartesian planner parameters
    cartesian_max_velocity_scaling: 1.0
    cartesian_max_acceleration_scaling: 1.0
    cartesian_step_size: 0.00025

    # Direction vectors
    approach_object_direction_z: 1.0
    lift_object_direction_z: 1.0
    lower_object_direction_z: -1.0
    retreat_direction_z: -1.0

    # Other parameters
    place_pose_z_offset_factor: 0.5
    retreat_min_distance: 0.025
    retreat_max_distance: 0.25
```

### Launch File Configuration

```python
def generate_launch_description():
    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=package_name)
        .trajectory_execution(file_path=moveit_controllers_file_path)
        .robot_description_semantic(file_path=srdf_model_path)
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
            default_planning_pipeline="ompl"
        )
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
        .to_moveit_configs()
    )

    # Create MTC node
    mtc_demo_node = Node(
        package="mycobot_mtc_pick_place_demo",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
            {'start_state': {'content': initial_positions_file_path}},
            mtc_node_params_file_path,
        ],
    )

    return LaunchDescription([mtc_demo_node])
```

---

## Perception Integration

### GetPlanningScene Service

This service processes point clouds to create collision objects dynamically.

**Service Definition (custom)**:
```
# Request
string target_shape                  # "cylinder" or "box"
float64[] target_dimensions          # [height, radius] for cylinder or [x, y, z] for box

---

# Response
moveit_msgs/PlanningSceneWorld scene_world
sensor_msgs/PointCloud2 full_cloud
sensor_msgs/Image rgb_image
string target_object_id
string support_surface_id
bool success
```

### Service Workflow

1. **Transform Point Cloud**: Convert to target frame and optionally crop
2. **Segment Plane**: Extract support surface using RANSAC
3. **Segment Objects**: Remove plane to isolate objects
4. **Feature Extraction**: Compute normals, curvature, RSD (Radius-based Surface Descriptor)
5. **Clustering**: Region growing segmentation
6. **Shape Fitting**: Fit primitives (cylinder/box) to clusters using RANSAC
7. **Target Identification**: Match detected objects to target specifications
8. **Create Collision Objects**: Generate MoveIt collision objects

### Using the Service in MTC Node

```cpp
// In MTCTaskNode constructor
planning_scene_client = std::make_shared<GetPlanningSceneClient>();

// In setupPlanningScene()
void MTCTaskNode::setupPlanningScene() {
  moveit::planning_interface::PlanningSceneInterface psi;

  auto object_type = this->get_parameter("object_type").as_string();
  auto object_dimensions = this->get_parameter("object_dimensions").as_double_array();

  // Call the service
  auto response = planning_scene_client->call_service(object_type, object_dimensions);

  // Apply collision objects to planning scene
  if (!psi.applyCollisionObjects(response.scene_world.collision_objects)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add collision objects");
  }

  // Update object parameters based on detected object
  target_object_id_ = response.target_object_id;
  support_surface_id_ = response.support_surface_id;

  // Update node parameters with actual detected object properties
  for (const auto& collision_object : response.scene_world.collision_objects) {
    if (collision_object.id == target_object_id_) {
      updateObjectParameters(collision_object);
      break;
    }
  }
}
```

---

## Implementation Patterns

### Pattern 1: Property Inheritance

```cpp
// Set at task level
task.setProperty("group", arm_group_name);
task.setProperty("eef", gripper_group_name);
task.setProperty("ik_frame", gripper_frame);

// Stage inherits from parent
stage->properties().configureInitFrom(mtc::Stage::PARENT);

// Container exposes properties to children
container->properties().exposeTo(stage->properties(), {"eef", "group"});
```

### Pattern 2: Collision Management

```cpp
// Allow collision between gripper and object for grasping
auto allow_stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
    "allow collision (gripper,object)");
allow_stage->allowCollisions(
    object_name,
    gripper_link_names,
    true);  // true = allow

// Forbid collision after lifting
auto forbid_stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
    "forbid collision (object,surface)");
forbid_stage->allowCollisions(
    {object_name},
    {support_surface_id},
    false);  // false = forbid
```

### Pattern 3: Attach/Detach Objects

```cpp
// Attach object to gripper
auto attach_stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
attach_stage->attachObject(object_name, gripper_frame);
attach_object_stage_ptr = attach_stage.get();  // Save for monitoring

// Detach object at place
auto detach_stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
detach_stage->detachObject(object_name, gripper_frame);
```

### Pattern 4: Stage Monitoring

```cpp
// Monitor current state for grasp pose generation
grasp_generator_stage->setMonitoredStage(current_state_ptr);

// Monitor attach stage for place pose generation
place_generator_stage->setMonitoredStage(attach_object_stage_ptr);
```

### Pattern 5: IK Computation Wrapper

```cpp
// Create generator stage
auto generator = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
generator->setObject(object_name);
generator->setAngleDelta(angle_delta);

// Wrap with IK computation
auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(generator));
ik_wrapper->setMaxIKSolutions(max_solutions);
ik_wrapper->setMinSolutionDistance(min_distance);
ik_wrapper->setIKFrame(transform, reference_frame);
ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});

container->insert(std::move(ik_wrapper));
```

### Pattern 6: Controller Specification

```cpp
// Set controller names for trajectory execution
std::vector<std::string> controller_names = {"arm_controller", "grip_action_controller"};

// At task level
task.setProperty("trajectory_execution_info",
    mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));

// Per-stage override if needed
stage->properties().set("trajectory_execution_info",
    mtc::TrajectoryExecutionInfo().set__controller_names(controller_names));
```

---

## Code Examples

### Complete Main Function

```cpp
int main(int argc, char** argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  int ret = 0;

  try {
    // Set up node options
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    // Create the MTC task node
    auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

    // Set up a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mtc_task_node);

    // Set up the planning scene and execute the task
    try {
      RCLCPP_INFO(mtc_task_node->get_logger(), "Setting up planning scene");
      mtc_task_node->setupPlanningScene();

      RCLCPP_INFO(mtc_task_node->get_logger(), "Executing task");
      mtc_task_node->doTask();

      RCLCPP_INFO(mtc_task_node->get_logger(),
          "Task execution completed. Keeping node alive for visualization.");

      // Keep the node running for visualization
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
```

### Complete doTask() Method

```cpp
void MTCTaskNode::doTask() {
  RCLCPP_INFO(this->get_logger(), "Starting the pick and place task");

  // Create the task
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

  // Plan the task
  if (!task_.plan(max_solutions)) {
    RCLCPP_ERROR(this->get_logger(), "Task planning failed");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Task planning succeeded");

  // Publish solution for visualization in RViz
  task_.introspection().publishSolution(*task_.solutions().front());
  RCLCPP_INFO(this->get_logger(), "Published solution for visualization");

  // Execute if requested
  if (execute) {
    RCLCPP_INFO(this->get_logger(), "Executing the planned task");
    auto result = task_.execute(*task_.solutions().front());

    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Task execution failed with error code: %d", result.val);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Task executed successfully");
  } else {
    RCLCPP_INFO(this->get_logger(), "Execution skipped as per configuration");
  }
}
```

### Helper Function: Vector to Eigen Transform

```cpp
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
  return Eigen::Translation3d(values[0], values[1], values[2]) *
         Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}

geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values) {
  return tf2::toMsg(vectorToEigen(values));
}
```

---

## Critical Dependencies

### Required ROS 2 Packages
```xml
<!-- package.xml -->
<depend>rclcpp</depend>
<depend>moveit_ros_planning_interface</depend>
<depend>moveit_task_constructor_core</depend>
<depend>moveit_task_constructor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>shape_msgs</depend>
<depend>moveit_msgs</depend>
<depend>tf2_geometry_msgs</depend>
<depend>tf2_eigen</depend>

<!-- For perception (optional) -->
<depend>sensor_msgs</depend>
<depend>pcl_ros</depend>
<depend>pcl_conversions</depend>
```

### CMake Configuration
```cmake
find_package(moveit_task_constructor_core REQUIRED)

add_executable(mtc_node src/mtc_node.cpp)
ament_target_dependencies(mtc_node
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning_interface
  geometry_msgs
  shape_msgs
  moveit_msgs
)
```

---

## Execution Flow Summary

1. **Initialize Node**: Declare parameters from YAML config
2. **Setup Planning Scene**:
   - Call perception service (optional)
   - Add collision objects to planning scene
   - Update parameters based on detected objects
3. **Create Task**: Build stage hierarchy with planners
4. **Initialize Task**: `task.init()` validates all stages
5. **Plan**: `task.plan(max_solutions)` generates trajectories
6. **Publish**: Visualize solution in RViz via `task.introspection()`
7. **Execute** (optional): `task.execute()` sends trajectories to controllers
8. **Spin**: Keep node alive for continuous visualization

---

## Best Practices

1. **Modular Stage Design**: Each stage has a single, clear responsibility
2. **Proper Collision Management**: Explicitly allow/forbid collisions at each stage
3. **Property Inheritance**: Set common properties at task/container level
4. **Error Handling**: Comprehensive logging and try-catch blocks
5. **Parameterization**: All tunable values in YAML config files
6. **Visualization**: Always publish solutions for debugging
7. **Controller Specification**: Set trajectory execution info per stage or globally
8. **Fallback Planning**: Use Alternatives/Fallbacks for robustness
9. **Stage Monitoring**: Link generator stages to ensure consistency
10. **IK Solutions**: Configure max solutions and min distance for diversity

---

## Troubleshooting Tips

### Task Initialization Fails
- Check that all required properties are set (group, eef, ik_frame)
- Verify stage property inheritance with `configureInitFrom()`
- Ensure container properties are exposed to children

### Planning Fails
- Increase timeout values
- Try different planners (fallback strategy)
- Check collision objects in planning scene
- Verify IK solutions exist for target poses
- Reduce `min_solution_distance` for IK

### Execution Fails
- Verify controller names match actual controllers
- Check trajectory execution info is set
- Ensure controllers are running and accepting goals
- Validate joint limits in SRDF

### No Solutions Generated
- Increase `max_solutions` parameter
- Check grasp/place pose generation parameters
- Verify object dimensions and poses
- Review approach/lift/lower distance min/max values

---

## Additional Resources

- [MoveIt Task Constructor Tutorial](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)
- [MTC API Documentation](https://moveit.picknik.ai/main/api/html/namespace_moveit_1_1task__constructor.html)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/)

---

**End of Guide**

This implementation provides a production-ready template for building complex manipulation tasks with MTC. Adapt the parameters, planners, and stage configurations to match your specific robot and application requirements.

---

## Essential MoveIt Configuration Files

MTC requires properly configured MoveIt files to function. Here are the critical configuration files you need:

### 1. SRDF File (Semantic Robot Description Format)

**File**: `config/<robot_name>/<robot_name>.srdf`

This is **the most critical file** for MTC. It defines:

#### Planning Groups
```xml
<!-- Arm planning group - used for arm movements -->
<group name="arm">
    <joint name="link1_to_link2"/>
    <joint name="link2_to_link3"/>
    <joint name="link3_to_link4"/>
    <joint name="link4_to_link5"/>
    <joint name="link5_to_link6"/>
    <joint name="link6_to_link6_flange"/>
</group>

<!-- Gripper planning group - used for gripper control -->
<group name="gripper">
    <joint name="gripper_controller"/>
</group>

<!-- Combined group (optional but useful) -->
<group name="arm_with_gripper">
    <group name="arm" />
    <group name="gripper" />
</group>
```

**Why it matters for MTC**:
- MTC uses `task.setProperty("group", "arm")` which references the SRDF group
- Group names must match your MTC parameter configuration

#### End Effector Definition
```xml
<end_effector name="gripper" parent_link="link6_flange" group="gripper"/>
```

**Why it matters for MTC**:
- MTC uses `task.setProperty("eef", "gripper")` which references this
- The `parent_link` defines where objects attach during grasping
- Must match your `gripper_frame` parameter

#### Named Poses (Group States)
```xml
<!-- Arm poses -->
<group_state name="home" group="arm">
    <joint name="link1_to_link2" value="0"/>
    <joint name="link2_to_link3" value="0"/>
    <joint name="link3_to_link4" value="0"/>
    <joint name="link4_to_link5" value="0"/>
    <joint name="link5_to_link6" value="0"/>
    <joint name="link6_to_link6_flange" value="0"/>
</group_state>

<group_state name="ready" group="arm">
    <joint name="link1_to_link2" value="0"/>
    <joint name="link2_to_link3" value="0"/>
    <joint name="link3_to_link4" value="1.5708"/>
    <joint name="link4_to_link5" value="1.5708"/>
    <joint name="link5_to_link6" value="0"/>
    <joint name="link6_to_link6_flange" value="0"/>
</group_state>

<!-- Gripper poses -->
<group_state name="open" group="gripper">
    <joint name="gripper_controller" value="0.0"/>
</group_state>

<group_state name="half_closed" group="gripper">
    <joint name="gripper_controller" value="-0.34"/>
</group_state>

<group_state name="closed" group="gripper">
    <joint name="gripper_controller" value="-0.50"/>
</group_state>
```

**Why it matters for MTC**:
- MTC stages use these named poses: `stage->setGoal("home")`, `stage->setGoal("open")`
- Must match your parameters:
  - `arm_home_pose: "home"`
  - `gripper_open_pose: "open"`
  - `gripper_close_pose: "half_closed"`

#### Collision Checking
```xml
<!-- Disable collision checking between adjacent links -->
<disable_collisions link1="base_link" link2="link1" reason="Adjacent"/>
<disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
<!-- ... more pairs ... -->

<!-- Disable collision checking between gripper and arm links -->
<disable_collisions link1="link6_flange" link2="gripper_base" reason="Adjacent"/>
<disable_collisions link1="link6_flange" link2="gripper_left1" reason="Never"/>
<!-- ... more gripper pairs ... -->
```

**Why it matters for MTC**:
- Prevents false collision detections during planning
- MTC stages use `getLinkModelNamesWithCollisionGeometry()` which respects these settings

---

### 2. MoveIt Controllers Configuration

**File**: `config/<robot_name>/moveit_controllers.yaml`

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_action_controller

  arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - link1_to_link2
      - link2_to_link3
      - link3_to_link4
      - link4_to_link5
      - link5_to_link6
      - link6_to_link6_flange

  gripper_action_controller:
    action_ns: gripper_cmd
    type: GripperCommand
    default: true
    joints:
      - gripper_controller
```

**Why it matters for MTC**:
- Controller names must match your MTC parameters:
  ```yaml
  controller_names:
    - "arm_controller"
    - "gripper_action_controller"
  ```
- MTC uses these to execute trajectories: `mtc::TrajectoryExecutionInfo().set__controller_names(controller_names)`
- Joint lists must match SRDF groups

---

### 3. Kinematics Configuration

**File**: `config/<robot_name>/kinematics.yaml`

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  position_only_ik: false
  kinematics_solver_attempts: 5

gripper:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  position_only_ik: false
  kinematics_solver_attempts: 3
```

**Why it matters for MTC**:
- MTC's `ComputeIK` stages require an IK solver
- Group names must match SRDF groups
- `kinematics_solver_timeout` affects MTC planning speed
- `kinematics_solver_attempts` affects success rate for difficult poses

**Common IK Solvers**:
- `kdl_kinematics_plugin/KDLKinematicsPlugin` - Standard, works for most robots
- `srv_kinematics_plugin/SrvKinematicsPlugin` - Fast analytical solver for specific robot types
- `bio_ik/BioIKKinematicsPlugin` - Advanced, handles constraints well

---

### 4. Pilz Cartesian Limits (if using Pilz planner)

**File**: `config/<robot_name>/pilz_cartesian_limits.yaml`

```yaml
cartesian_limits:
  max_trans_vel: 1.0      # m/s
  max_trans_acc: 2.25     # m/s²
  max_trans_dec: -5.0     # m/s²
  max_rot_vel: 1.57       # rad/s (≈90°/s)
```

**Why it matters for MTC**:
- Required if you use Pilz planner in fallback strategies
- Limits affect trajectory smoothness and safety
- Must be within robot's physical capabilities

---

### 5. Initial Positions

**File**: `config/<robot_name>/initial_positions.yaml`

```yaml
initial_positions:
  link1_to_link2: 0.0
  link2_to_link3: 0.0
  link3_to_link4: 0.0
  link4_to_link5: 0.0
  link5_to_link6: 0.0
  link6_to_link6_flange: 0.0
  gripper_controller: 0.0
```

**Why it matters for MTC**:
- Sets the starting state for planning
- Used by `CurrentState` stage in MTC
- Should match a safe, collision-free configuration

---

### 6. Joint Limits (Important for Safety)

**File**: `config/<robot_name>/joint_limits.yaml`

```yaml
joint_limits:
  link1_to_link2:
    has_position_limits: true
    min_position: -2.879793
    max_position: 2.879793
    has_velocity_limits: true
    max_velocity: 3.0
    has_acceleration_limits: true
    max_acceleration: 2.0

  # ... similar for all joints ...

  gripper_controller:
    has_position_limits: true
    min_position: -0.7
    max_position: 0.05
    has_velocity_limits: true
    max_velocity: 0.5
```

**Why it matters for MTC**:
- Prevents planning outside robot's physical limits
- MTC respects these limits during trajectory generation
- Affects planning success rate

---

## MoveIt Configuration Checklist for MTC

Use this checklist when setting up a new robot for MTC:

### SRDF File (`<robot>.srdf`)
- [ ] Define `arm` planning group with all arm joints
- [ ] Define `gripper` planning group with gripper joints
- [ ] Define end effector with correct `parent_link`
- [ ] Create named poses: `home`, `ready` for arm
- [ ] Create named poses: `open`, `closed`, `half_closed` for gripper
- [ ] Disable collisions between adjacent links
- [ ] Disable collisions between gripper and arm links

### Controllers (`moveit_controllers.yaml`)
- [ ] Define arm controller with type `FollowJointTrajectory`
- [ ] Define gripper controller with type `GripperCommand`
- [ ] List all joints for each controller
- [ ] Verify action namespaces match hardware interface

### Kinematics (`kinematics.yaml`)
- [ ] Configure IK solver for arm group
- [ ] Set reasonable timeout (0.05-0.1s typical)
- [ ] Set search resolution (0.005 typical)
- [ ] Configure gripper group (if needed for complex grippers)

### Cartesian Limits (`pilz_cartesian_limits.yaml`)
- [ ] Set max velocities within robot specs
- [ ] Set max accelerations conservatively
- [ ] Test with Pilz planner before using in MTC

### Initial Positions (`initial_positions.yaml`)
- [ ] Set safe starting positions
- [ ] Verify no self-collisions
- [ ] Test that robot can reach this pose

### Joint Limits (`joint_limits.yaml`)
- [ ] Set position limits from robot datasheet
- [ ] Set velocity limits (use 70-80% of max for safety)
- [ ] Set acceleration limits conservatively

---

## How Launch File Uses These Configs

The launch file ties everything together:

```python
moveit_config = (
    MoveItConfigsBuilder(robot_name, package_name=package_name)
    .trajectory_execution(file_path=moveit_controllers_file_path)      # Uses moveit_controllers.yaml
    .robot_description_semantic(file_path=srdf_model_path)             # Uses .srdf
    .joint_limits(file_path=joint_limits_file_path)                    # Uses joint_limits.yaml
    .robot_description_kinematics(file_path=kinematics_file_path)      # Uses kinematics.yaml
    .planning_pipelines(
        pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
        default_planning_pipeline="ompl"
    )
    .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path) # Uses pilz_cartesian_limits.yaml
    .to_moveit_configs()
)

# Pass to MTC node
mtc_node = Node(
    package="your_mtc_package",
    executable="mtc_node",
    parameters=[
        moveit_config.to_dict(),                      # All MoveIt configs
        {'start_state': {'content': initial_positions_file_path}},  # Initial positions
        mtc_params_file_path,                         # Your MTC-specific params
    ],
)
```

---

## Parameter Name Mapping (Critical!)

Your MTC parameters **must match** the SRDF and controller definitions:

| MTC Parameter | Must Match | File |
|---------------|------------|------|
| `arm_group_name: "arm"` | `<group name="arm">` | SRDF |
| `gripper_group_name: "gripper"` | `<group name="gripper">` | SRDF |
| `gripper_frame: "link6_flange"` | `<end_effector parent_link="link6_flange">` | SRDF |
| `arm_home_pose: "home"` | `<group_state name="home" group="arm">` | SRDF |
| `gripper_open_pose: "open"` | `<group_state name="open" group="gripper">` | SRDF |
| `gripper_close_pose: "half_closed"` | `<group_state name="half_closed" group="gripper">` | SRDF |
| `controller_names: ["arm_controller", ...]` | `controller_names: [arm_controller, ...]` | moveit_controllers.yaml |

**If these don't match, MTC will fail with cryptic errors like**:
- "Unknown group: arm"
- "No IK solver found for group"
- "Failed to find controller for group"
- "Named target 'home' not found"

---

## Quick Start: Minimal Required Files

If starting from scratch, you **must have** these 4 files configured:

1. **SRDF** - Groups, end effector, named poses, collision pairs
2. **moveit_controllers.yaml** - Controller definitions
3. **kinematics.yaml** - IK solvers
4. **initial_positions.yaml** - Starting configuration

The others are optional but recommended:
- **joint_limits.yaml** - Safety (highly recommended)
- **pilz_cartesian_limits.yaml** - Only if using Pilz planner

---

## Troubleshooting MoveIt Config Issues

### "Unknown group 'arm'"
- Check SRDF has `<group name="arm">`
- Verify SRDF is loaded in launch file: `.robot_description_semantic(file_path=srdf_model_path)`
- Check parameter: `arm_group_name: "arm"` matches SRDF

### "No IK solver found"
- Check `kinematics.yaml` has entry for group name
- Verify kinematics file loaded: `.robot_description_kinematics(file_path=kinematics_file_path)`
- Check IK plugin is installed: `ros2 pkg list | grep kdl`

### "Named target 'home' not found"
- Check SRDF has `<group_state name="home" group="arm">`
- Verify group name matches: pose must be for the same group you're commanding

### "Controller not found"
- Check `moveit_controllers.yaml` has controller defined
- Verify controller name matches MTC parameter: `controller_names`
- Check controller is running: `ros2 control list_controllers`

### Planning always fails
- Check joint limits aren't too restrictive
- Verify no self-collisions in initial pose
- Test IK solver works: `ros2 run moveit_ros_move_group test_move_group`
