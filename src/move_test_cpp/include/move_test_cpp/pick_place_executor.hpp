#pragma once
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "move_test_cpp/pick_place_executor.hpp"
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <thread>
namespace pick_place
{

/**
 * @brief 한 번의 Pick-and-Place 시퀀스를 실행한다.
 *        동일 노드/MoveGroup 재사용을 위해 클래스로 묶음.
 */
class Executor
{
public:
  explicit Executor(const rclcpp::Node::SharedPtr &node,
                    const std::string &planning_group = "ur_manipulator",
                    const std::string &gripper_topic  = "/forward_position_gripper_controller/commands");

  /** @return true if both pick & place succeeded */
  bool run(const geometry_msgs::msg::PoseStamped &pick_pose,
           const geometry_msgs::msg::PoseStamped &place_pose);

private:
  // 내부 유틸
  void publishGripper(double position);
  bool goToPose(const geometry_msgs::msg::PoseStamped &target);
  bool goCartesian(double delta_z);
  bool isObjectGrasped(double cmd_pos, double thresh = 0.02);
  bool pickOnce(const geometry_msgs::msg::PoseStamped &pick_pose);

  rclcpp::Node::SharedPtr                  node_;
  rclcpp::Logger                           log_;
  moveit::planning_interface::MoveGroupInterface mg_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr grip_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  double current_gripper_pos_{0.0};
};

} // namespace pick_place
