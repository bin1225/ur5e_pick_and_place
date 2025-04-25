from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type         = LaunchConfiguration("ur_type")
    safety_limits   = LaunchConfiguration("safety_limits")
    controllers_file= LaunchConfiguration("controllers_file")
    description_file= LaunchConfiguration("description_file")
    moveit_launch_file = LaunchConfiguration("moveit_launch_file")

    # 1) Gazebo 시뮬레이션 및 robot_state_publisher 포함
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "controllers_file": controllers_file,
            "description_file": description_file,
            "launch_rviz": "false",
        }.items(),
    )

    # 2) MoveIt 플러그인 실행 시 name:=ur_type 인자를 추가
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": "true",
            # ↓ SRDF xacro에 name 인자를 넘겨줘야 'ur5e'로 설정됩니다.
            "name": ur_type,
        }.items(),
    )

    return [ur_control_launch, ur_moveit_launch]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e", description="UR robot type"),
        DeclareLaunchArgument("safety_limits", default_value="true", description="Enable safety limits"),
        DeclareLaunchArgument("controllers_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "config", "ur_controllers.yaml"]),
            description="Path to controllers yaml"),
        DeclareLaunchArgument("description_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"]),
            description="URDF/xacro file"),
        DeclareLaunchArgument("moveit_launch_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]),
            description="Path to MoveIt config launch"),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
