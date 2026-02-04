#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class JointTeleopNode : public rclcpp::Node
{
public:
  JointTeleopNode()
      : rclcpp::Node("joint_teleop_node")
  {
    command_topic_ = declare_parameter<std::string>("command_topic", "teleop/joint_commands");
    controller_topic_ = declare_parameter<std::string>(
        "controller_topic", "/arm_controller/joint_trajectory");
    action_name_ = declare_parameter<std::string>(
        "action_name", "/arm_controller/follow_joint_trajectory");
    execution_time_sec_ = declare_parameter<double>("execution_time_sec", 1.0);
    joint_order_ = declare_parameter<std::vector<std::string>>(
        "controller_joint_names",
        {"shoulder_pitch_joint",
         "shoulder_roll_joint",
         "shoulder_yaw_joint",
         "elbow_pitch_joint",
         "elbow_yaw_joint",
         "wrist_roll_joint"});

    if (joint_order_.empty())
    {
      RCLCPP_FATAL(get_logger(), "Parameter 'controller_joint_names' must not be empty.");
      rclcpp::shutdown();
      return;
    }

    current_positions_.assign(joint_order_.size(), 0.0);
    joint_index_.clear();
    for (size_t idx = 0; idx < joint_order_.size(); ++idx)
    {
      joint_index_[joint_order_[idx]] = idx;
    }

    controller_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(controller_topic_, 10);
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, action_name_);
    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        command_topic_, 10,
        std::bind(&JointTeleopNode::handle_command, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Joint teleop ready. Listening on '%s' and publishing trajectories to '%s'",
        command_topic_.c_str(), controller_topic_.c_str());
  }

private:
  void handle_command(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.empty() || msg->position.empty())
    {
      RCLCPP_WARN(get_logger(), "Received empty joint command, ignoring.");
      return;
    }

    if (msg->name.size() != msg->position.size())
    {
      RCLCPP_ERROR(
          get_logger(),
          "Joint command name/position size mismatch (%zu vs %zu).",
          msg->name.size(), msg->position.size());
      return;
    }

    auto traj = trajectory_msgs::msg::JointTrajectory();
    traj.header.stamp.sec = 0;
    traj.header.stamp.nanosec = 0;
    traj.joint_names = joint_order_;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = current_positions_;
    bool velocity_used = false;
    if (!point.positions.empty())
    {
      point.velocities.assign(point.positions.size(), 0.0);
    }

    bool any_valid = false;
    double total_delta = 0.0;
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      const auto &joint_name = msg->name[i];
      const auto idx_it = joint_index_.find(joint_name);
      if (idx_it == joint_index_.end())
      {
        RCLCPP_WARN(get_logger(), "Joint '%s' is not controlled by this teleop node.", joint_name.c_str());
        continue;
      }

      if (i >= msg->position.size())
      {
        RCLCPP_ERROR(get_logger(), "Missing position entry for joint '%s'.", joint_name.c_str());
        continue;
      }

      const auto idx = idx_it->second;
      const double previous = current_positions_[idx];
      const double requested = msg->position[i];
      current_positions_[idx] = requested;
      point.positions[idx] = requested;
      total_delta += std::fabs(requested - previous);

      if (!msg->velocity.empty())
      {
        if (msg->velocity.size() == msg->name.size())
        {
          point.velocities[idx] = msg->velocity[i];
          velocity_used = true;
        }
        else
        {
          RCLCPP_WARN_ONCE(
              get_logger(), "Velocity vector size mismatch. Velocities will be ignored.");
        }
      }
      any_valid = true;
    }

    if (!any_valid)
    {
      RCLCPP_WARN(get_logger(), "No valid joints found in the received command.");
      return;
    }

    if (total_delta < 1e-4)
    {
      RCLCPP_WARN(get_logger(), "Joint command matches current target positions; no motion will occur.");
    }

    if (!velocity_used)
    {
      point.velocities.clear();
    }

    const int64_t duration_ns = static_cast<int64_t>(execution_time_sec_ * 1e9);
    point.time_from_start.sec = static_cast<int32_t>(duration_ns / 1000000000LL);
    point.time_from_start.nanosec = static_cast<uint32_t>(duration_ns % 1000000000LL);
    traj.points.push_back(point);

    auto subscriber_count = controller_pub_->get_subscription_count();
    if (subscriber_count == 0)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "No subscribers on %s. Check that the trajectory controller is running.",
          controller_topic_.c_str());
    }

    controller_pub_->publish(traj);
    send_follow_joint_trajectory_goal(traj);

    std::ostringstream summary;
    summary.setf(std::ios::fixed);
    summary.precision(3);
    for (size_t i = 0; i < joint_order_.size(); ++i)
    {
      summary << joint_order_[i] << "=" << point.positions[i];
      if (i + 1 < joint_order_.size())
      {
        summary << ", ";
      }
    }

    RCLCPP_DEBUG(
        get_logger(),
        "Sent trajectory for %zu joints (%.2f s). Targets: [%s]",
        traj.joint_names.size(), execution_time_sec_, summary.str().c_str());
  }

  std::string command_topic_;
  std::string controller_topic_;
  std::string action_name_;
  double execution_time_sec_{1.0};
  std::vector<std::string> joint_order_;
  std::vector<double> current_positions_;
  std::unordered_map<std::string, size_t> joint_index_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr controller_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

  using GoalHandleFollowJointTrajectory =
      rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  static const char *follow_joint_error_to_string(int32_t code)
  {
    using control_msgs::action::FollowJointTrajectory_Result;
    switch (code)
    {
    case FollowJointTrajectory_Result::SUCCESSFUL:
      return "SUCCESSFUL";
    case FollowJointTrajectory_Result::INVALID_GOAL:
      return "INVALID_GOAL";
    case FollowJointTrajectory_Result::INVALID_JOINTS:
      return "INVALID_JOINTS";
    case FollowJointTrajectory_Result::OLD_HEADER_TIMESTAMP:
      return "OLD_HEADER_TIMESTAMP";
    case FollowJointTrajectory_Result::PATH_TOLERANCE_VIOLATED:
      return "PATH_TOLERANCE_VIOLATED";
    case FollowJointTrajectory_Result::GOAL_TOLERANCE_VIOLATED:
      return "GOAL_TOLERANCE_VIOLATED";
    default:
      return "UNKNOWN";
    }
  }

  void send_follow_joint_trajectory_goal(const trajectory_msgs::msg::JointTrajectory &traj)
  {
    if (!action_client_)
    {
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(100)))
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "FollowJointTrajectory action server '%s' is not available.",
          action_name_.c_str());
      return;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory = traj;
    const int64_t tol_ns = static_cast<int64_t>(execution_time_sec_ * 1e9);
    goal.goal_time_tolerance.sec = static_cast<int32_t>(tol_ns / 1000000000LL);
    goal.goal_time_tolerance.nanosec = static_cast<uint32_t>(tol_ns % 1000000000LL);

    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        [this](GoalHandleFollowJointTrajectory::SharedPtr handle)
    {
      if (!handle)
      {
        RCLCPP_ERROR(get_logger(), "FollowJointTrajectory goal was rejected.");
      }
      else
      {
        RCLCPP_INFO(get_logger(), "FollowJointTrajectory goal accepted.");
      }
    };
    send_goal_options.feedback_callback =
        [this](
            GoalHandleFollowJointTrajectory::SharedPtr,
            const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
    {
      if (!feedback || feedback->joint_names.empty())
      {
        return;
      }
      RCLCPP_DEBUG(
          get_logger(),
          "Controller feedback for %zu joints, remaining %.3f s",
          feedback->joint_names.size(),
          rclcpp::Duration(feedback->desired.time_from_start).seconds() -
              rclcpp::Duration(feedback->actual.time_from_start).seconds());
    };
    send_goal_options.result_callback =
        [this](const GoalHandleFollowJointTrajectory::WrappedResult &result)
    {
      const auto &status = result.result;
      const auto error_msg = status ? status->error_string : std::string();
      const auto error_code = status ? status->error_code : 0;
      const char *error_code_text = follow_joint_error_to_string(error_code);
      switch (result.code)
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "FollowJointTrajectory goal succeeded.");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(
            get_logger(),
            "FollowJointTrajectory goal aborted. Error code: %d (%s). Reason: %s",
            error_code, error_code_text,
            error_msg.empty() ? "unspecified" : error_msg.c_str());
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(
            get_logger(),
            "FollowJointTrajectory goal canceled. Error code: %d (%s). Reason: %s",
            error_code, error_code_text,
            error_msg.empty() ? "unspecified" : error_msg.c_str());
        break;
      default:
        RCLCPP_ERROR(get_logger(), "FollowJointTrajectory goal returned unknown result.");
        break;
      }
    };

    action_client_->async_send_goal(goal, send_goal_options);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
