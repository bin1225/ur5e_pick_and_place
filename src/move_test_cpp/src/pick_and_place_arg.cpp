#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <string>

using namespace std::chrono_literals;

/**
 * @brief Helper to publish a simple Robotiq 2F‑85 gripper command.
 *        Adjust the data length/values if your controller expects something different.
 */
void publishGripperCmd(const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &pub,
                       double position)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {position}; // {left_knuckle_pos, …} if multi‑joint controller
  pub->publish(msg);
}

/**
 * @brief Plan & execute a cartesian pose target.
 * @return true if execution succeeded.
 */
bool goToPose(moveit::planning_interface::MoveGroupInterface &mg,
              const geometry_msgs::msg::PoseStamped &target,
              rclcpp::Logger logger)
{
  mg.setStartStateToCurrentState();
  mg.clearPoseTargets();
  mg.setPoseTarget(target, mg.getEndEffectorLink());

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  constexpr int max_attempts = 3;
  for (int i = 0; i < max_attempts; ++i)
  {
    if (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(logger, "Plan attempt %d succeeded – executing", i + 1);
      return mg.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
    RCLCPP_WARN(logger, "Plan attempt %d failed", i + 1);
  }
  return false;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_and_place_arg_cpp_node");
  auto logger = node->get_logger();

  // --------------------------------------------------------------------------
  // 1) Parse CLI arguments  (pick_x pick_y pick_z place_x place_y place_z)
  // --------------------------------------------------------------------------
  double pick_x{0.50}, pick_y{0.00}, pick_z{0.20};
  double place_x{0.30}, place_y{-0.30}, place_z{0.20};

  if (argc == 7)
  {
    try
    {
      pick_x  = std::stod(argv[1]);
      pick_y  = std::stod(argv[2]);
      pick_z  = std::stod(argv[3]);
      place_x = std::stod(argv[4]);
      place_y = std::stod(argv[5]);
      place_z = std::stod(argv[6]);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(logger, "Argument conversion failed: %s", e.what());
      return EXIT_FAILURE;
    }
  }
  else
  {
    RCLCPP_WARN(logger, "Expected 6 args – using defaults");
  }
  RCLCPP_INFO(logger, "Pick  (%.3f, %.3f, %.3f)", pick_x, pick_y, pick_z);
  RCLCPP_INFO(logger, "Place (%.3f, %.3f, %.3f)", place_x, place_y, place_z);

  // --------------------------------------------------------------------------
  // 2) MoveGroupInterface setup
  // --------------------------------------------------------------------------
  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(5);
  move_group.setGoalPositionTolerance(0.01);   // 1 cm
  move_group.setGoalOrientationTolerance(0.05);// ≈3°
  move_group.setPoseReferenceFrame("base_link");
  move_group.allowReplanning(true);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // --------------------------------------------------------------------------
  // 3) Gripper publisher
  // --------------------------------------------------------------------------
  auto gripper_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_gripper_controller/commands", 10);
  rclcpp::sleep_for(200ms); // give publisher time to connect

  // --------------------------------------------------------------------------
  // 4) Open gripper
  // --------------------------------------------------------------------------
  publishGripperCmd(gripper_pub, 0.1);
  RCLCPP_INFO(logger, "Gripper opened");
  rclcpp::sleep_for(500ms);

  // --------------------------------------------------------------------------
  // 5) Build pick pose
  // --------------------------------------------------------------------------
  geometry_msgs::msg::PoseStamped pick_pose;
  pick_pose.header.frame_id = "base_link";
  pick_pose.pose.position.x = pick_x;
  pick_pose.pose.position.y = pick_y;
  pick_pose.pose.position.z = pick_z;

  tf2::Quaternion q_pick;
  q_pick.setRPY(M_PI, 0.0, 0.0); // TCP pointing down
  q_pick.normalize();
  pick_pose.pose.orientation.x = q_pick.x();
  pick_pose.pose.orientation.y = q_pick.y();
  pick_pose.pose.orientation.z = q_pick.z();
  pick_pose.pose.orientation.w = q_pick.w();

  if (!goToPose(move_group, pick_pose, logger))
  {
    RCLCPP_ERROR(logger, "Failed to reach pick pose");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // --------------------------------------------------------------------------
  // 6) Close gripper
  // --------------------------------------------------------------------------
  publishGripperCmd(gripper_pub, 0.7);
  RCLCPP_INFO(logger, "Gripper closed");
  rclcpp::sleep_for(1s);

  // --------------------------------------------------------------------------
  // 7) Build place pose (same orientation)
  // --------------------------------------------------------------------------
  geometry_msgs::msg::PoseStamped place_pose = pick_pose;
  place_pose.pose.position.x = place_x;
  place_pose.pose.position.y = place_y;
  place_pose.pose.position.z = place_z;

  if (!goToPose(move_group, place_pose, logger))
  {
    RCLCPP_ERROR(logger, "Failed to reach place pose");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // --------------------------------------------------------------------------
  // 8) Open gripper to release
  // --------------------------------------------------------------------------
  publishGripperCmd(gripper_pub, 0.1);
  RCLCPP_INFO(logger, "Object released – task complete");

  geometry_msgs::msg::PoseStamped home_pose = pick_pose;
  place_pose.pose.position.x = place_x;
  place_pose.pose.position.y = place_y;
  place_pose.pose.position.z = place_z+0.5;
  if (!goToPose(move_group, place_pose, logger))
  {
    RCLCPP_ERROR(logger, "Failed to reach place pose");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
