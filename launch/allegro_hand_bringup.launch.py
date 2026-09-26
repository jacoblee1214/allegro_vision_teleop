#!/usr/bin/env python3
"""
allegro_hand_bringup.launch.py — ros2_control bring-up for the Allegro Hand V6 on ROS 2 Jazzy.

This is a teleop-specific bring-up. It reuses the Wonik driver packages that are already
built in ~/v6f_manuse_teleoperation (allegro_hand_v6_bringup / _description / _hardware)
but leaves them untouched: no patch file has to be copied into the driver package.

Differences from allegro_hand_v6_bringup/launch/allegro_hand.launch.py:
  * `rviz` argument really turns RViz2 on/off (the cockpit UI draws its own 3D hand).
  * `joint_states_topic` lets robot_state_publisher read URDF-coordinate joint states
    (/allegro/joint_states_urdf from sim_bridge_node) instead of raw motor states.
    Only matters for a left hand, where the MCP motors are zeroed at -pi/2.
  * No tactile relay / dashboard / pose GUI nodes: those belong to the MANUS demo.
  * Controller spawners wait up to 60 s, because Modbus hardware init takes ~9 s and the
    default 10 s spawner timeout drops joint_state_broadcaster on a cold start.

Arguments:
  hand:=right|left                      hand model (default right)
  hardware:=hardware|mock_components|isaac   ros2_control plugin (default hardware)
  descriptor:=modbus_tcp:<ip>:<port>    io interface (default modbus_tcp:192.168.1.100:502)
  hand_id:=1                            Modbus slave id
  rviz:=true|false                      open RViz2 (default true)
  joint_states_topic:=/joint_states     topic robot_state_publisher subscribes to

Example:
  ros2 launch launch/allegro_hand_bringup.launch.py \
      hardware:=hardware descriptor:=modbus_tcp:192.168.1.100:502 hand:=right rviz:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    hand = LaunchConfiguration("hand")
    hardware = LaunchConfiguration("hardware")
    descriptor = LaunchConfiguration("descriptor")
    hand_id = LaunchConfiguration("hand_id")
    rviz = LaunchConfiguration("rviz")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    bringup_share = FindPackageShare("allegro_hand_v6_bringup")
    description_share = FindPackageShare("allegro_hand_v6_description")

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                PathJoinSubstitution(
                    [bringup_share, "config", "single_hand", "allegro_hand.urdf.xacro"]
                ),
                " hand:=", hand,
                " ros2_control_hardware_type:=", hardware,
                " io_interface_descriptor:=", descriptor,
                " hand_id:=", hand_id,
            ]
        )
    }

    controllers_yaml = PathJoinSubstitution(
        [bringup_share, "config", "single_hand", "ros2_controllers.yaml"]
    )
    rviz_config = PathJoinSubstitution([description_share, "rviz", "allegro_hand_v6.rviz"])

    # RViz's fixed frame is `world`; the hand URDF root is `base_link`.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_link_publisher",
        output="log",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "-1.5707963", "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "base_link",
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[robot_description],
        remappings=[("joint_states", joint_states_topic)],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "allegro_hand_position_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hand", default_value="right",
                                  description="right or left"),
            DeclareLaunchArgument("hardware", default_value="hardware",
                                  description="hardware (Modbus TCP), mock_components, or isaac"),
            DeclareLaunchArgument("descriptor", default_value="modbus_tcp:192.168.1.100:502",
                                  description="io interface descriptor; the driver default is "
                                              "192.168.40.100, so pass this explicitly"),
            DeclareLaunchArgument("hand_id", default_value="1",
                                  description="Modbus slave / hand id"),
            DeclareLaunchArgument("rviz", default_value="true",
                                  description="open RViz2"),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states",
                                  description="joint states topic for robot_state_publisher "
                                              "(/allegro/joint_states_urdf for a left hand)"),
            static_tf,
            control_node,
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            position_controller_spawner,
            rviz_node,
        ]
    )
