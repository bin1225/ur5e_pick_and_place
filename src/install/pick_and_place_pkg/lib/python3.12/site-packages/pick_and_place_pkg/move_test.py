#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy, PlanningComponent

class MoveTest(Node):
    def __init__(self):
        super().__init__('move_test')
        
        moveit_cpp_yaml = '<path-to>/moveit_cpp.yaml'
        self.moveit_py = MoveItPy(
        node_name='move_test',
        launch_params_filepaths=[moveit_cpp_yaml],
        provide_planning_service=True
        )

        self.get_logger().info('MoveItPy client initialized, connected to existing MoveItCpp')

        # Get the planning component for the UR5e manipulator group
        self.arm: PlanningComponent = self.moveit_py.get_planning_component('ur_manipulator')

    def go_to_pose(self, pose: PoseStamped) -> bool:
        # Set current state as start
        self.arm.set_start_state_to_current_state()
        # Define goal pose for end effector
        self.arm.set_goal_state(pose_stamped_msg=pose, pose_link='ee_link')
        # Plan
        plan = self.arm.plan()
        if plan.joint_trajectory.points:
            self.get_logger().info('Plan found; executing...')
            self.arm.execute(plan.joint_trajectory)
            return True
        else:
            self.get_logger().error('Planning failed')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = MoveTest()

    # Simple test pose
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'base_link'
    target_pose.pose.position.x = 0.4
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.4
    target_pose.pose.orientation.w = 1.0

    success = node.go_to_pose(target_pose)
    if success:
        node.get_logger().info('Simple move test succeeded')
    else:
        node.get_logger().error('Simple move test failed')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
