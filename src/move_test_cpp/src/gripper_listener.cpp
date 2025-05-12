#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class GripperListener : public rclcpp::Node
{
public:
  GripperListener()
  : Node("gripper_listener")
  {
    // 1) /joint_states 구독
    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&GripperListener::jointStateCallback, this, std::placeholders::_1));
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // 2) 원하는 그리퍼 조인트 이름 찾기
    const std::string target = "gripper_robotiq_85_left_knuckle_joint";
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == target) {
        double pos = 0.0;
        if (i < msg->position.size()) {
          pos = msg->position[i];
        }
        RCLCPP_INFO(this->get_logger(),
                    "[/joint_states] %s position = %.3f",
                    target.c_str(), pos);
        return;
      }
    }
    // (선택) 조인트가 없으면
    RCLCPP_WARN_ONCE(this->get_logger(),
                     "Joint '%s' not found in /joint_states", target.c_str());
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
