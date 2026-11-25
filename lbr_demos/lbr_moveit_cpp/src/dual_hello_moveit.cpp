#include <memory>
#include <string>
#include <future>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/position_constraint.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "moveit_msgs/msg/bounding_volume.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::placeholders;
using moveit_msgs::action::MoveGroup;

class MoveGroupActionClientNode : public rclcpp::Node {
public:
  using GoalHandleMoveGroup = rclcpp_action::ClientGoalHandle<MoveGroup>;

  MoveGroupActionClientNode(const std::string & node_name)
  : Node(node_name) {
    action_client_ = rclcpp_action::create_client<MoveGroup>(
      this, "/move_action");

    RCLCPP_INFO(this->get_logger(), "Waiting for action server /move_action...");
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      throw std::runtime_error("Couldn't connect to action server /move_action.");
    }
    RCLCPP_INFO(this->get_logger(), "Connected to action server.");
  }

  void send_goal_async(
    const geometry_msgs::msg::Pose & target,
    const std::string & move_group_name,
    const std::string & base,
    const std::string & end_effector,
    std::function<void(GoalHandleMoveGroup::WrappedResult)> done_callback)
  {
    auto goal_msg = MoveGroup::Goal();
    goal_msg.request.allowed_planning_time = 1.0;
    goal_msg.request.group_name = move_group_name;
    goal_msg.request.max_acceleration_scaling_factor = 0.1;
    goal_msg.request.max_velocity_scaling_factor = 0.1;
    goal_msg.request.num_planning_attempts = 1;

    moveit_msgs::msg::Constraints constraint;
    moveit_msgs::msg::PositionConstraint pc;
    moveit_msgs::msg::OrientationConstraint oc;

    std_msgs::msg::Header header;
    header.frame_id = base;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    primitive.dimensions = {0.0001};

    moveit_msgs::msg::BoundingVolume bounding_volume;
    bounding_volume.primitives.push_back(primitive);
    bounding_volume.primitive_poses.push_back(target);

    pc.header = header;
    pc.link_name = end_effector;
    pc.constraint_region = bounding_volume;
    pc.weight = 1.0;

    oc.header = header;
    oc.link_name = end_effector;
    oc.orientation = target.orientation;
    oc.absolute_x_axis_tolerance = 0.001;
    oc.absolute_y_axis_tolerance = 0.001;
    oc.absolute_z_axis_tolerance = 0.001;
    oc.weight = 1.0;

    constraint.position_constraints.push_back(pc);
    constraint.orientation_constraints.push_back(oc);

    goal_msg.request.goal_constraints.push_back(constraint);

    auto send_goal_options = rclcpp_action::Client<MoveGroup>::SendGoalOptions();
    send_goal_options.result_callback = [done_callback](const GoalHandleMoveGroup::WrappedResult & result) {
      done_callback(result);
    };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<MoveGroup>::SharedPtr action_client_;
};

std::string result_code_to_string(rclcpp_action::ResultCode code) {
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return "SUCCEEDED";
    case rclcpp_action::ResultCode::ABORTED:
      return "ABORTED";
    case rclcpp_action::ResultCode::CANCELED:
      return "CANCELED";
    default:
      return "UNKNOWN";
  }
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveGroupActionClientNode>("hello_moveit_cpp");

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  bool right_done = false;
  bool left_done = false;

  node->send_goal_async(pose, "lbr_right", "lbr_right_link_0", "lbr_right_link_ee",
    [&right_done](rclcpp_action::ClientGoalHandle<MoveGroup>::WrappedResult result) {
      std::cout << "Right goal result received. Status: " << result_code_to_string(result.code) << std::endl;
      right_done = true;
    });
  
  node->send_goal_async(pose, "lbr_left", "lbr_left_link_0", "lbr_left_link_ee",
    [&left_done](rclcpp_action::ClientGoalHandle<MoveGroup>::WrappedResult result) {
      std::cout << "Left goal result received. Status: " << result_code_to_string(result.code) << std::endl;
      left_done = true;
    });

  // Spin until both are done
  while (!(right_done && left_done)) {
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}