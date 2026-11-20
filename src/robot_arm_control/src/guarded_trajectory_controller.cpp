#include <cmath>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace robot_arm_control
{

// Fortress can latch a velocity-driven joint at a hard stop. Reserve an interior
// command range and validate both ingress paths before handing commands to JTC.
class GuardedTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override
  {
    const auto result = JointTrajectoryController::on_configure(previous_state);
    if (result != controller_interface::CallbackReturn::SUCCESS) {
      return result;
    }

    action_server_.reset();
    joint_command_subscriber_.reset();
    limits_.clear();
    for (const auto & joint : params_.joints) {
      const auto bounds = auto_declare<std::vector<double>>("position_bounds." + joint, {});
      if (bounds.size() != 2 || !std::isfinite(bounds[0]) || !std::isfinite(bounds[1]) ||
        bounds[0] >= bounds[1])
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Missing or invalid command bounds for %s", joint.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      limits_.emplace(joint, std::make_pair(bounds[0], bounds[1]));
    }

    auto node = get_node();
    const std::string action_name = std::string(node->get_name()) + "/follow_joint_trajectory";
    action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
      node->get_node_base_interface(), node->get_node_clock_interface(),
      node->get_node_logging_interface(), node->get_node_waitables_interface(), action_name,
      [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowJTrajAction::Goal> goal) {
        if (!within_command_range(goal->trajectory)) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return goal_received_callback(uuid, goal);
      },
      [this](auto goal_handle) {return goal_cancelled_callback(goal_handle);},
      [this](auto goal_handle) {goal_accepted_callback(goal_handle);});

    joint_command_subscriber_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
      [this](std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory) {
        if (within_command_range(*trajectory)) {
          topic_callback(trajectory);
        }
      });
    return controller_interface::CallbackReturn::SUCCESS;
  }

private:
  bool within_command_range(const trajectory_msgs::msg::JointTrajectory & trajectory) const
  {
    const auto logger = get_node()->get_logger();
    const std::set<std::string> unique_joints(
      trajectory.joint_names.begin(), trajectory.joint_names.end());
    if (unique_joints.size() != trajectory.joint_names.size()) {
      RCLCPP_WARN(logger, "Rejected trajectory with duplicate joint names");
      return false;
    }
    for (const auto & point : trajectory.points) {
      // Position-only waypoints use linear interpolation. Derivative inputs
      // enable splines whose extrema can exceed the checked waypoint bounds.
      if (point.positions.size() != trajectory.joint_names.size() ||
        !point.velocities.empty() || !point.accelerations.empty() || !point.effort.empty())
      {
        RCLCPP_WARN(logger, "Expected a position-only waypoint with one value per joint");
        return false;
      }
      for (std::size_t index = 0; index < point.positions.size(); ++index) {
        const auto & name = trajectory.joint_names[index];
        const auto bound = limits_.find(name);
        const double position = point.positions[index];
        if (bound == limits_.end() || !std::isfinite(position) ||
          position < bound->second.first || position > bound->second.second)
        {
          RCLCPP_WARN(logger, "Rejected waypoint position %.6f for joint %s", position, name.c_str());
          return false;
        }
      }
    }
    return true;
  }

  std::map<std::string, std::pair<double, double>> limits_;
};

}  // namespace robot_arm_control

PLUGINLIB_EXPORT_CLASS(
  robot_arm_control::GuardedTrajectoryController, controller_interface::ControllerInterface)
