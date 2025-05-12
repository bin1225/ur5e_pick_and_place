#include "move_test_cpp/pick_place_executor.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <sensor_msgs/msg/joint_state.hpp>

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
  mg_.setPlannerId("RRTConnectkConfigDefault");
  mg_.setGoalJointTolerance(0.01);
  mg_.setGoalPositionTolerance(0.02);
  mg_.setGoalOrientationTolerance(0.05);
  mg_.setPlanningTime(5.0);
  mg_.setNumPlanningAttempts(30);
  mg_.setMaxVelocityScalingFactor(0.5);
  mg_.setMaxAccelerationScalingFactor(0.5);
  mg_.allowReplanning(true);

  // 그리퍼 퍼블리셔
  grip_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(gripper_topic, 10);
  rclcpp::sleep_for(200ms);

  state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      const std::string joint = "gripper_robotiq_85_left_knuckle_joint";
      for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == joint && i < msg->position.size()) {
          current_gripper_pos_ = msg->position[i];
          break;
        }
      }
    });
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
  constexpr int MAX_RETRY = 2;
  bool grasp_ok = false;
  for (int attempt = 0; attempt < MAX_RETRY; attempt++)    
  {
    RCLCPP_INFO(log_, "Pick attempt %d ...", attempt + 1);
    grasp_ok = pickOnce(pick_pose);
    if (grasp_ok) break;

    RCLCPP_WARN(log_, "Pick failed (no object). Retrying...");
    // 물체 놓친 경우 자리 정렬 시간을 조금 줌
    rclcpp::sleep_for(300ms);
  }

  if (!grasp_ok) {
    RCLCPP_ERROR(log_, "Pick failed after %d attempts", MAX_RETRY + 1);
    return false;
  }

  // 6. place pose 로 이동
  if (!goToPose(place_pose)) {
    RCLCPP_ERROR(log_, "Place pose failed");
    return false;
  }

  // 7. Release
  publishGripper(0.1);
  rclcpp::sleep_for(500ms);

  RCLCPP_INFO(log_, "Pick-and-Place done");
  return true;
}

bool Executor::isObjectGrasped(double cmd_pos, double thresh)
{
  rclcpp::spin_some(node_);
  double delta = std::abs(cmd_pos - current_gripper_pos_);
  RCLCPP_INFO(log_, "[isObjectGrasped] cmd=%.3f, actual=%.3f, Δ=%.3f",
              cmd_pos, current_gripper_pos_, delta);
  return delta > thresh;
}

bool Executor::pickOnce(const geometry_msgs::msg::PoseStamped &pick_pose)
{
  geometry_msgs::msg::PoseStamped above = pick_pose;
  above.pose.position.z += 0.12;

  isObjectGrasped(0.7);
  // 0. 먼저 그리퍼 열기
  publishGripper(0.1);
  rclcpp::sleep_for(500ms);
  
  if (!goToPose(above)   || !goToPose(pick_pose))
    return false;

  const double close_cmd = 0.7;
  publishGripper(close_cmd);          // 닫기
  rclcpp::sleep_for(1s);

  if (!goToPose(above))               
    return false;

  return isObjectGrasped(close_cmd);  // 집힘 감지
}
}