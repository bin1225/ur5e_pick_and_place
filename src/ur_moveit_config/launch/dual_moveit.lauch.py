# ────────────────────────── import ──────────────────────────
import os, yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
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
        DeclareLaunchArgument(
            "warehouse_sqlite_path",
            default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
            description="Path where the warehouse database should be stored",
        ),
        DeclareLaunchArgument("launch_servo",  default_value="false"),
        DeclareLaunchArgument("namespace",  default_value="ur1"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("publish_robot_description_semantic", default_value="true"),
        DeclareLaunchArgument("tf_prefix",  default_value=""),
    ]

# ────────────────────────────────────────────────────────────

def launch_setup(context, *args, **kwargs):
    
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    ur_type = LaunchConfiguration("ur_type").perform(context)
    rviz_on = LaunchConfiguration("launch_rviz").perform(context)
    launch_servo = LaunchConfiguration("launch_servo").perform(context)
    publish_sem = LaunchConfiguration("publish_robot_description_semantic").perform(context)
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path").perform(context)
    ns=LaunchConfiguration("namespace").perform(context)
    
    # MoveIt config - context-aware!
    moveit_cfg = (
        MoveItConfigsBuilder(robot_name="ur5e", package_name="ur_moveit_config")
            .robot_description(
                "/home/bin1225/workspaces/ur_gz/src/ur_simulation_gz/ur_simulation_gz/urdf/dual_gz.urdf.xacro",
                {"name": "ur5e", "ur_type": ur_type, "tf_prefix": tf_prefix}
            )
            .robot_description_semantic(
                "/home/bin1225/workspaces/ur_gz/src/ur_moveit_config/srdf/dual.srdf.xacro",
                {"name": "ur5e", "tf_prefix": tf_prefix}
            )
            # .planning_pipelines(
            #     default_planning_pipeline="ompl",
            #     pipelines=["ompl"]
            # )
            .to_moveit_configs()
    )
    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # Servo YAML (context-safe)
    servo_yaml_path = Path(get_package_share_directory("ur_moveit_config")) / "config" / "ur_servo.yaml"
    with open(servo_yaml_path, 'r') as f:
        servo_yaml = yaml.safe_load(f)


    controller_yaml_filename = f"{ns}_moveit_controllers.yaml"  # 예: ur1_moveit_controllers.yaml
    controller_yaml_path = (
        Path(get_package_share_directory("ur_moveit_config")) / "config" / controller_yaml_filename
    )
    if not controller_yaml_path.exists():
        raise RuntimeError(f"{controller_yaml_path} 파일이 존재하지 않습니다. 네임스페이스와 yaml 이름을 확인하세요!")

    with open(controller_yaml_path, "r") as f:
        controller_yaml = yaml.safe_load(f)

    
    # 노드들 선언 (모두 context-aware!)
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        namespace=ns,
        parameters=[
            moveit_cfg.to_dict(),
            warehouse_ros_config,
            controller_yaml,
            {"use_sim_time": True, "publish_robot_description_semantic": publish_sem},
        ],
    )

    servo_node = Node(
        package="moveit_servo", executable="servo_node",
        condition=IfCondition(launch_servo), output="screen",
        parameters=[moveit_cfg.to_dict(), {"moveit_servo": servo_yaml, "use_sim_time": True}],
    )


    rviz_cfg = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"),
        "config",
        TextSubstitution(text=ns + "_moveit.rviz")
    ])
    
    rviz_node = Node(
        namespace=ns,
        package="rviz2", executable="rviz2", name=(ns+"_rvize"),
        condition=IfCondition(rviz_on), output="log",
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_cfg.robot_description,
            moveit_cfg.robot_description_semantic,
            moveit_cfg.robot_description_kinematics,
            moveit_cfg.planning_pipelines,
            moveit_cfg.joint_limits,
            warehouse_ros_config,
            {"use_sim_time": True},
        ],
    )

    wait_robot_description = Node(
        namespace=ns,
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    # RegisterEventHandler 등은 context-safe하게 등록
    return [
        wait_robot_description,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group, rviz_node, servo_node],
            )
        ),
    ]
    #return [move_group, rviz_node, servo_node]

def generate_launch_description():
    ld = LaunchDescription(declare_arguments())
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
