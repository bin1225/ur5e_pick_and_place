#include "move_test_cpp/pick_place_executor.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("demo_pick_place_controller");

  pick_place::Executor executor(node);   // 재사용 가능한 객체 하나 생성

  // ▶︎ 여러 목표를 순차 실행
  for (int i = 0; i < 3; ++i)
  {
    geometry_msgs::msg::PoseStamped pick, place;
    pick.header.frame_id  = place.header.frame_id = "base_link";

    // 임시 좌표 예시
    pick.pose.position.x  = 0.50; pick.pose.position.y = 0.00 + 0.05*i; pick.pose.position.z = 0.20;
    place.pose.position.x = 0.30; place.pose.position.y = -0.30;        place.pose.position.z = 0.20;

    tf2::Quaternion q; q.setRPY(M_PI, 0.0, 0.0);
    geometry_msgs::msg::Quaternion ori = tf2::toMsg(q); // ★ 변환
    pick.pose.orientation  = ori;
    place.pose.orientation = ori;

    executor.run(pick, place);           // 라이브러리 함수 호출
  }

  rclcpp::shutdown();
  return 0;
}
