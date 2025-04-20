#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from moveit.planning import MoveItPy, PlanningComponent

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')

        # ▶ 파라미터 파일 없이 생성
        self.moveit_py = MoveItPy(
            node_name='pick_and_place',            # rclpy 노드 이름
            provide_planning_service=True          # planning 서비스 제공
        )
        self.arm: PlanningComponent = self.moveit_py.get_planning_component('ur_manipulator')
        self.get_logger().info('MoveItPy initialized (parameters from launch)')

        # 그리퍼 제어 퍼블리셔
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_gripper_controller/commands',
            10
        )

    def go_to_pose(self, pose: PoseStamped) -> bool:
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=pose, pose_link='ee_link')
        plan = self.arm.plan()
        if plan.joint_trajectory.points:
            self.get_logger().info('Plan found; executing...')
            self.arm.execute(plan.joint_trajectory)
            return True
        else:
            self.get_logger().error('Planning failed!')
            return False

    def control_gripper(self, position: float):
        msg = Float64MultiArray(data=[position])
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'Gripper cmd: {position}')

    def run(self):
        # --- PICK ---
        pick = PoseStamped()
        pick.header.frame_id = 'base_link'
        pick.pose.position.x = 0.5
        pick.pose.position.y = 0.0
        pick.pose.position.z = 0.2
        pick.pose.orientation.w = 1.0

        if not self.go_to_pose(pick):
            return
        self.control_gripper(0.0)  # close
        self.get_logger().info('Picked, waiting 1s...')
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))

        # --- PLACE ---
        place = PoseStamped()
        place.header.frame_id = 'base_link'
        place.pose.position.x = 0.3
        place.pose.position.y = -0.3
        place.pose.position.z = 0.2
        place.pose.orientation.w = 1.0

        if not self.go_to_pose(place):
            return
        self.control_gripper(0.8)  # open
        self.get_logger().info('Placed, done.')

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
