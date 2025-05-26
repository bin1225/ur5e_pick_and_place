#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>

#include <tf2/LinearMath/Quaternion.h>


using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_and_place_cpp");

  // MoveGroupInterface for UR5e manipulator
  moveit::planning_interface::MoveGroupInterface move_group(node, "ur1_manipulator");
  move_group.setPlanningTime(10.0);
  move_group.setStartStateToCurrentState();
  move_group.setGoalPositionTolerance(0.01);   // 1cm
  move_group.setGoalOrientationTolerance(0.05); // 약 3°
  move_group.allowReplanning(true);
  // 여러 번 시도
  move_group.setNumPlanningAttempts(5);
  move_group.setMaxVelocityScalingFactor(0.5);   // 속도의 50%
  move_group.setMaxAccelerationScalingFactor(0.5);

  // Gripper command publisher
  auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/ur1/forward_position_gripper_controller/commands", 10);

  // Open gripper
  std_msgs::msg::Float64MultiArray open_cmd;
  open_cmd.data = {0.1};  // adjust value for open position
  gripper_pub->publish(open_cmd);
  RCLCPP_INFO(node->get_logger(), "Gripper opened");


  // --- PICK ---
  geometry_msgs::msg::PoseStamped pick_pose;
  pick_pose.header.frame_id = "ur1_base_link";
  
  // 위치 설정
  pick_pose.pose.position.x = 0.5;
  pick_pose.pose.position.y = 0.0;
  pick_pose.pose.position.z = 0.2;
  
  // tf2 쿼터니언 생성
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, 0.0);
  // geometry_msgs::msg::Quaternion 으로 직접 복사
  pick_pose.pose.orientation.x = q.x();
  pick_pose.pose.orientation.y = q.y();
  pick_pose.pose.orientation.z = q.z();
  pick_pose.pose.orientation.w = q.w();
  
  // 플래닝
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(pick_pose);

  moveit::planning_interface::MoveGroupInterface::Plan pick_plan;
  if (move_group.plan(pick_plan) == moveit::core::MoveItErrorCode::SUCCESS 
      && !pick_plan.trajectory.joint_trajectory.points.empty()) {
    RCLCPP_INFO(node->get_logger(), "Pick plan found, executing");
    move_group.execute(pick_plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Pick planning failed");
    return 1;
  }

  // Close gripper
  std_msgs::msg::Float64MultiArray close_cmd;
  close_cmd.data = {0.7};  // adjust value for closed position
  gripper_pub->publish(close_cmd);
  RCLCPP_INFO(node->get_logger(), "Gripper closed");
  rclcpp::sleep_for(1s);

  // --- PLACE ---
  geometry_msgs::msg::PoseStamped place_pose;
  place_pose.header.frame_id = "ur1_base_link";
  place_pose.pose.position.x = 0.3;
  place_pose.pose.position.y = -0.3;
  place_pose.pose.position.z = 0.0;

  place_pose.pose.orientation.x = q.x();
  place_pose.pose.orientation.y = q.y();
  place_pose.pose.orientation.z = q.z();
  place_pose.pose.orientation.w = q.w();

  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(place_pose);
  moveit::planning_interface::MoveGroupInterface::Plan place_plan;
  if (move_group.plan(place_plan) == moveit::core::MoveItErrorCode::SUCCESS 
      && !place_plan.trajectory.joint_trajectory.points.empty()) {
    RCLCPP_INFO(node->get_logger(), "Place plan found, executing");
    move_group.execute(place_plan);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Place planning failed");
    return 1;
  }

  // Open gripper
  open_cmd.data = {0.1};  // adjust value for open position
  gripper_pub->publish(open_cmd);
  RCLCPP_INFO(node->get_logger(), "Gripper opened");

  rclcpp::shutdown();
  return 0;
}
