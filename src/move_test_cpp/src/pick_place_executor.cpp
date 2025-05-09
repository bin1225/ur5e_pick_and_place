#include "move_test_cpp/pick_place_executor.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>

using namespace std::chrono_literals;
namespace pick_place
{

Executor::Executor(const rclcpp::Node::SharedPtr &node,
                   const std::string &planning_group,
                   const std::string &gripper_topic)
  : node_(node),
    log_(node_->get_logger()),
    mg_(node_, planning_group)          // MoveGroupInterface 생성
{
  // MoveGroup 기본 파라미터
  mg_.setPlanningTime(10.0);
  mg_.setNumPlanningAttempts(5);
  mg_.setGoalPositionTolerance(0.01);
  mg_.setGoalOrientationTolerance(0.05);
  mg_.setPoseReferenceFrame("base_link");
  mg_.allowReplanning(true);
  mg_.setMaxVelocityScalingFactor(0.5);
  mg_.setMaxAccelerationScalingFactor(0.5);

  // 그리퍼 퍼블리셔
  grip_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(gripper_topic, 10);
  rclcpp::sleep_for(200ms);
}

void Executor::publishGripper(double pos)
{
  std_msgs::msg::Float64MultiArray m;  m.data = {pos};
  grip_pub_->publish(m);
}

bool Executor::goToPose(const geometry_msgs::msg::PoseStamped &target)
{
  mg_.setStartStateToCurrentState();
  mg_.clearPoseTargets();
  mg_.setPoseTarget(target, mg_.getEndEffectorLink());

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  constexpr int max_try = 3;
  for (int i = 0; i < max_try; ++i)
  {
    if (mg_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
      return mg_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }
  return false;
}

bool Executor::run(const geometry_msgs::msg::PoseStamped &pick_pose,
                   const geometry_msgs::msg::PoseStamped &place_pose)
{
  // 1. 오픈
  publishGripper(0.1);
  rclcpp::sleep_for(500ms);

  // 2. 픽
  if (!goToPose(pick_pose))     {RCLCPP_ERROR(log_, "Pick pose failed");  return false;}
  publishGripper(0.7);          // grip
  rclcpp::sleep_for(1s);

  // 3. 플레이스
  if (!goToPose(place_pose))    {RCLCPP_ERROR(log_, "Place pose failed"); return false;}
  publishGripper(0.1);          // release

  RCLCPP_INFO(log_, "Pick-and-Place done");
  return true;
}

} // namespace pick_place
