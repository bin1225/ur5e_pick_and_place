# ────────────────────────── import ──────────────────────────
import os, yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, RegisterEventHandler, LogInfo
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace
# ────────────────────────────────────────────────────────────


# ───────── Declare 인자는 **리스트만** 반환하도록 변경 ─────────
def declare_arguments():
    return [
        DeclareLaunchArgument("launch_rviz", default_value="true",
                              description="Launch RViz?"),
        DeclareLaunchArgument(
            "ur_type",
            choices=["ur3","ur3e","ur5","ur5e","ur10","ur10e","ur16e","ur20","ur30"],
            description="Type/series of used UR robot."
        ),
        DeclareLaunchArgument("warehouse_sqlite_path",
                              default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite")),
        DeclareLaunchArgument("launch_servo",  default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("publish_robot_description_semantic", default_value="true"),
        DeclareLaunchArgument("tf_prefix",  default_value=""),
        DeclareLaunchArgument("namespace",  default_value="",
                              description="ROS namespace for this MoveIt instance"),
    ]
# ────────────────────────────────────────────────────────────


def generate_launch_description():
    # ── LaunchConfigurations ──
    ns         = LaunchConfiguration("namespace")
    tf_prefix  = LaunchConfiguration("tf_prefix")
    ur_type    = LaunchConfiguration("ur_type")
    use_sim    = LaunchConfiguration("use_sim_time")
    rviz_on    = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")
    publish_sem = LaunchConfiguration("publish_robot_description_semantic")
    sqlite_path = LaunchConfiguration("warehouse_sqlite_path")

    # ── MoveIt config ──
    moveit_cfg = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
          .robot_description(
              Path("urdf") / "ur.urdf.xacro",
              {"name": ur_type, "ur_type": ur_type, "prefix": tf_prefix})
          .robot_description_semantic(
              Path("srdf") / "dual.srdf.xacro",
              {"name": ur_type, "prefix": tf_prefix})
          .to_moveit_configs()
    )
    warehouse_ros = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": sqlite_path,
    }

    # ── LaunchDescription 생성 ──
    ld = LaunchDescription(declare_arguments())

    # ☆ 여기서부터 <ns>/ 로 묶음 – 반드시 첫 줄
    ld.add_action(PushRosNamespace(ns))

    # ── 노드 & 로그 ──
    ld.add_action(LogInfo(msg=["MoveIt ns: ", ns]))

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",              # 네임스페이스 자동 적용, remap 없이 OK
    )

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[
            moveit_cfg.to_dict(),
            warehouse_ros,
            {"use_sim_time": use_sim,
             "publish_robot_description_semantic": publish_sem},
        ],
    )

    servo_yaml = yaml.safe_load(
        open(Path(get_package_share_directory("ur_moveit_config"))
             / "config" / "ur_servo.yaml"))
    servo_node = Node(
        package="moveit_servo", executable="servo_node",
        condition=IfCondition(launch_servo), output="screen",
        parameters=[moveit_cfg.to_dict(), {"moveit_servo": servo_yaml}],
    )

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2_moveit",
        condition=IfCondition(rviz_on), output="log",
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_cfg.robot_description,
            moveit_cfg.robot_description_semantic,
            moveit_cfg.robot_description_kinematics,
            moveit_cfg.planning_pipelines,
            moveit_cfg.joint_limits,
            warehouse_ros,
            {"use_sim_time": use_sim},
        ],
    )

    # ── 이벤트: description 도착 후 move_group·rviz·servo 시작 ──
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=wait_robot_description,
                      on_exit=[move_group, rviz_node, servo_node])))

    return ld
