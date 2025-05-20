#include "move_test_cpp/pick_place_executor.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  

geometry_msgs::msg::PoseStamped makePose(
    double x, double y, double z,
    const tf2::Quaternion &q,
    const std::string &frame = "base_link")
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  p.pose.orientation = tf2::toMsg(q);
  return p;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"use_sim_time", rclcpp::ParameterValue(true)}});
  auto node = rclcpp::Node::make_shared("demo_controller", opts);

  pick_place::Executor executor(node);
  

  // ───────────── 1) 좌표 목록 정의 ─────────────
  tf2::Quaternion q;  q.setRPY(M_PI, 0.0, 0.0);   // TCP ↓

  struct TargetPair {geometry_msgs::msg::PoseStamped pick, place;};
  std::vector<TargetPair> goals;

  goals.push_back({
    makePose(0.35, 0.158, 0.181, q),   // pick
    makePose(0.35, 0.358,  0.181, q)    // place (고정)
  });
  goals.push_back({
    makePose(0.7, 0.158, 0.181, q),   // pick
    makePose(0.7, 0.358,  0.181, q)    // place (고정)
  });
  goals.push_back({
    makePose(0.55, 0.158, 0.181, q),   // pick
    makePose(0.55, 0.358, 0.181, q)    // place (고정)
  });

  // ───────────── 2) 순차 실행 ─────────────
  for (size_t idx = 0; idx < goals.size(); ++idx)
  {
    RCLCPP_INFO(node->get_logger(), "=== Goal %zu / %zu ===", idx + 1, goals.size());
    if (!executor.run(goals[idx].pick, goals[idx].place))
    {
      RCLCPP_ERROR(node->get_logger(), "Goal %zu failed – 중단", idx + 1);
      continue;   // 실패 시 중단, 필요하면 continue 로 재시도
    }
  }

  rclcpp::shutdown();
  return 0;
}