import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    hand_side = LaunchConfiguration("hand", default="right")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    ros2_control_hardware_type = LaunchConfiguration("ros2_control_hardware_type", default="hardware")
    io_interface_descriptor = LaunchConfiguration("io_interface_descriptor", default="modbus_tcp:192.168.1.100:502")
    hand_id = LaunchConfiguration("hand_id", default="1")
    # v6_1: joint_states topic that robot_state_publisher (and thus RViz) renders.
    # The teleop bridge republishes /joint_states in URDF coordinates (left MCP motor offset removed).
    joint_states_topic = LaunchConfiguration("joint_states_topic", default="/joint_states")

    # Robot Description
    robot_description_str = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [
                    FindPackageShare("allegro_hand_v6_bringup"),
                    "config",
                    "single_hand",
                    "allegro_hand_v6_1.urdf.xacro",
                ]
            ),
            " hand:=",
            hand_side,
            " ros2_control_hardware_type:=",
            ros2_control_hardware_type,
            " io_interface_descriptor:=",
            io_interface_descriptor,
            " hand_id:=",
            hand_id,
        ]
    )
    robot_description = {"robot_description": robot_description_str}

    # Path to controller configuration
    ros2_controllers_path = PathJoinSubstitution(
        [
            FindPackageShare("allegro_hand_v6_bringup"),
            "config",
            "single_hand",
            "ros2_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[("joint_states", joint_states_topic)],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    allegro_hand_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["allegro_hand_position_controller", "-c", "/controller_manager"],
    )

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("allegro_hand_v6_description"), "rviz", "allegro_hand_v6.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
        condition=IfCondition(use_rviz),
    )

    static_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link_publisher',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'base_link'
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2 if true",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "hand",
                default_value="right",
                description="Specify which hand to use: right or left",
            ),
            DeclareLaunchArgument(
                "ros2_control_hardware_type",
                default_value="hardware",
                description="ROS 2 control hardware interface type: mock_components (standalone RViz), isaac (Isaac Sim bridge), hardware/physical_device (physical V6 serial hand)",
            ),
            DeclareLaunchArgument(
                "io_interface_descriptor",
                default_value="modbus_tcp:192.168.1.100:502",
                description="Hardware communication interface descriptor (e.g. modbus_tcp:192.168.1.100:502 or serial:/dev/ttyUSB0,2000000)",
            ),
            DeclareLaunchArgument(
                "hand_id",
                default_value="1",
                description="Modbus slave / hand ID",
            ),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/joint_states",
                description="JointState topic rendered by robot_state_publisher / RViz (e.g. /allegro/joint_states_urdf)",
            ),
            static_tf_publisher_node,
            control_node,
            robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            allegro_hand_position_controller_spawner,
            rviz_node, 
        ]
    )
