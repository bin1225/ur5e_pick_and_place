#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_test_cpp");
  // /move_group must already be running (via your launch file)
  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");

  // Build a simple goal pose
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = "base_link";
  target.pose.position.x = 0.4;
  target.pose.position.y = 0.0;
  target.pose.position.z = 0.4;
  target.pose.orientation.w = 1.0;
  move_group.setPoseTarget(target);

  // Prepare a Plan object and call plan(plan)
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto ec = move_group.plan(plan);

  // Check both the error code and that we got a trajectory
  if (ec == moveit::core::MoveItErrorCode::SUCCESS
      && !plan.trajectory.joint_trajectory.points.empty())
  {
    RCLCPP_INFO(node->get_logger(), "Plan found, executing");
    move_group.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Planning failed (code: %d)", 
      static_cast<int>(ec.val));
  }

  rclcpp::shutdown();
  return 0;
}
