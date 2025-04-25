from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # --- 기존 인자
    ur_type                = LaunchConfiguration("ur_type")
    safety_limits          = LaunchConfiguration("safety_limits")
    safety_pos_margin      = LaunchConfiguration("safety_pos_margin")
    safety_k_position      = LaunchConfiguration("safety_k_position")
    controllers_file       = LaunchConfiguration("controllers_file")
    tf_prefix              = LaunchConfiguration("tf_prefix")
    activate_joint_ctrl    = LaunchConfiguration("activate_joint_controller")
    initial_joint_ctrl     = LaunchConfiguration("initial_joint_controller")
    description_file       = LaunchConfiguration("description_file")
    launch_rviz            = LaunchConfiguration("launch_rviz")
    rviz_config_file       = LaunchConfiguration("rviz_config_file")
    gazebo_gui             = LaunchConfiguration("gazebo_gui")
    world_file             = LaunchConfiguration("world_file")

    # ▶ robot_description_xacro 호출 시 name:=ur_type 로 넘깁니다.
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        description_file, " ",
        "safety_limits:=", safety_limits, " ",
        "safety_pos_margin:=", safety_pos_margin, " ",
        "safety_k_position:=", safety_k_position, " ",
        # ↓ 여기서 기존에 "name:=ur" 이던 것을 "name:=<ur_type>" 으로 변경
        "name:=", ur_type, " ",
        "ur_type:=", ur_type, " ",
        "tf_prefix:=", tf_prefix, " ",
        "simulation_controllers:=", controllers_file
    ])
    robot_description = {"robot_description": robot_description_content}

    # ▶ robot_state_publisher 로 URDF(/robot_description) 올리기
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # ▶ 이하 기존과 동일
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )
    initial_joint_ctrl_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=[initial_joint_ctrl, "-c", "/controller_manager"],
        condition=IfCondition(activate_joint_ctrl),
    )
    initial_joint_ctrl_stopped = Node(
        package="controller_manager", executable="spawner",
        arguments=[initial_joint_ctrl, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(activate_joint_ctrl),
    )
    gripper_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["forward_position_gripper_controller", "-c", "/controller_manager"],
        output="screen",
    )

    gz_spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-string", robot_description_content, "-name", ur_type, "-allow_renaming", "true"],
    )
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": IfElseSubstitution(
                gazebo_gui,
                if_value=[" -r -v 4 ", world_file],
                else_value=[" -s -r -v 4 ", world_file],
            )
        }.items(),
    )
    gz_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    return [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz,
        initial_joint_ctrl_stopped,
        initial_joint_ctrl_spawner,
        gz_spawn,
        gz_launch,
        gz_bridge,
        gripper_controller_spawner,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e",
            description="Type/series of used UR robot."),
        DeclareLaunchArgument("safety_limits", default_value="true", description="Enable safety limits"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15", description="Safety margin"),
        DeclareLaunchArgument("safety_k_position", default_value="20", description="Safety k-position"),
        DeclareLaunchArgument("controllers_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "config", "ur_controllers.yaml"]),
            description="Path to controller YAML"),
        DeclareLaunchArgument("tf_prefix", default_value='""', description="TF prefix for multi-robot"),
        DeclareLaunchArgument("activate_joint_controller", default_value="true", description="Activate joint ctrl"),
        DeclareLaunchArgument("initial_joint_controller", default_value="scaled_joint_trajectory_controller",
            description="Initial robot controller"),
        DeclareLaunchArgument("description_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "urdf", "ur_gz.urdf.xacro"]),
            description="URDF/XACRO path"),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"),
        DeclareLaunchArgument("rviz_config_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]),
            description="RViz config file"),
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Gazebo GUI?"),
        DeclareLaunchArgument("world_file", default_value="empty.sdf", description="Gazebo world file"),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
