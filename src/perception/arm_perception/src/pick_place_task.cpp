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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <Eigen/Geometry>

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
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : Node("mtc_pick_place", options)
{
  node_base_interface_ = this->get_node_base_interface();

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
    RCLCPP_ERROR(LOGGER, "   Make sure test_environment_publisher.py is running:");
    RCLCPP_ERROR(LOGGER, "   ros2 run arm_perception test_environment_publisher.py");
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

  const double goal_joint_tolerance = this->get_parameter("planner.goal_joint_tolerance").as_double();
  const double cartesian_step_size = this->get_parameter("planner.cartesian_step_size").as_double();
  const double max_velocity_scaling = this->get_parameter("planner.max_velocity_scaling").as_double();
  const double max_acceleration_scaling = this->get_parameter("planner.max_acceleration_scaling").as_double();

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
    mtc::stages::Connect::GroupPlannerVector planners = {
        { "arm", sampling_planner },
        { "hand", sampling_planner }
    };
    auto stage = std::make_unique<mtc::stages::Connect>("move to pick", planners);
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
  ---- *               Approach Object                    *
     ***************************************************/
    {
      // Approach distance should be small since grasp poses are already near the object
      // Cylinder radius = 3cm, palm thickness ~2-3cm, so minimum clearance ~1cm
      const double approach_min = 0.0;    // Allow zero approach if already at grasp pose
      const double approach_max = 0.10;   // Maximum 10cm approach distance
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(approach_min, approach_max);

      // Approach direction in hand_frame (following tutorial pattern)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;  // approach along hand Z axis
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

      // ComputeIK wrapper - try without transform first to verify IK works
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(hand_frame);  // Direct frame, no transform

      wrapper->setIgnoreCollisions(true);
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
  ---- *               Close Hand                      *
     ***************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", sampling_planner);
      stage->setGroup(hand_group);
      stage->setGoal(hand_close_pose);
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
      vec.vector.y = -1.0;  // UP is -Y direction in base_link!
      vec.vector.z = 0.0;
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
  ---- *          Generate Place Pose (fixed world)        *
     *****************************************************/
    {
      const int max_ik_solutions = this->get_parameter("grasp.max_ik_solutions").as_int();
      const double min_solution_distance = this->get_parameter("grasp.min_solution_distance").as_double();

      // Destination pose: above destination table top
      // Adjusted position for better reachability (flexibility of ~10cm)
      const double dest_table_x = this->get_parameter("destination_table.position.x").as_double();
      const double dest_table_y = this->get_parameter("destination_table.position.y").as_double();
      const double dest_table_z = this->get_parameter("destination_table.position.z").as_double();
      const double dest_table_thickness = this->get_parameter("destination_table.dimensions.thickness").as_double();
      const double cylinder_height = this->get_parameter("cylinder.dimensions.height").as_double();
      const double dest_table_top_z = dest_table_z + (dest_table_thickness / 2.0);
      const double target_z = dest_table_top_z + (cylinder_height / 2.0);  // cylinder center at table top

      auto stage = std::make_unique<mtc::stages::GeneratePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "ik_frame" });
      stage->properties().set("marker_ns", "place_pose");
      stage->setMonitoredStage(pick_stage_ptr);

      geometry_msgs::msg::PoseStamped place_pose;
      place_pose.header.frame_id = world_frame;
      // Try position closer to robot base (10cm forward in Z) for better reach
      place_pose.pose.position.x = dest_table_x + 0.10;  // 10cm forward (closer to robot)
      place_pose.pose.position.y = dest_table_y;
      place_pose.pose.position.z = target_z + 0.05;  // Only 5cm above (not 10cm)

      // Rotate 180° around Z axis (blue/up) to flip the hand orientation
      // This makes: Green → back, Red → left, Blue → up
      Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
      place_pose.pose.orientation.x = q.x();
      place_pose.pose.orientation.y = q.y();
      place_pose.pose.orientation.z = q.z();
      place_pose.pose.orientation.w = q.w();

      stage->setPose(place_pose);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(max_ik_solutions);
      wrapper->setMinSolutionDistance(min_solution_distance);
      wrapper->setIKFrame(hand_frame);
      wrapper->setIgnoreCollisions(true);
      wrapper->setGroup("arm");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    /******************************************************
  ---- *          Lower Object to Table                   *
     *****************************************************/
    {
      // Lower the object from approach height (10cm above) down to table surface
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.08, 0.12);  // Lower 8-12cm (covers the +0.10 offset)
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lower_object");

      // Move downward (-Y direction in base_link)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame;  // base_link
      vec.vector.x = 0.0;
      vec.vector.y = 1.0;   // DOWN is +Y direction (opposite of lift)
      vec.vector.z = 0.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Open Hand                              *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", sampling_planner);
      stage->setGroup(hand_group);
      stage->setGoal(hand_open_pose);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Detach Object                          *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("test_cylinder", hand_frame);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Retreat Motion                            *
     *****************************************************/
    {
      // Stronger retreat upward; allow zero minimum to avoid failures when already clear
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat after place", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.30);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          Clear Object (extra lift)             *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("clear object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.0, 0.15);  // allow zero if already clear
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "clear_object");
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = world_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    /******************************************************
  ---- *          allow collision (hand, object)        *
     *****************************************************/
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("test_cylinder",
                           *task_.getRobotModel()->getJointModelGroup("hand"),
                           true);
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

  // Publish task solution for visualization
  task_.introspection().publishSolution(*task_.solutions().front());

  // Execute task
  RCLCPP_INFO(LOGGER, "Executing task solution...");
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(LOGGER, "Task execution failed with error code: %d", result.val);
    return;
  }

  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "✅ Task completed successfully!");
  RCLCPP_INFO(LOGGER, "========================================");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

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
