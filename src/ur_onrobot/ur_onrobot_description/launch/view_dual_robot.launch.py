import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur3e", description="Type/series of UR robot."),
        DeclareLaunchArgument("onrobot_type", default_value="rg2", description="Type of OnRobot gripper."),
    ]

    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")

    # Biên dịch xacro sang URDF string text
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("ur_onrobot_description"), "urdf", "dual_ur_onrobot.urdf.xacro"]), " ",
        "ur_type:=", ur_type, " ",
        "onrobot_type:=", onrobot_type, " ",
        "use_fake_hardware:=true",
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # KHỞI ĐỘNG RVIZ2: Ép tự động nhận gốc world và map đúng topic hiển thị robot luôn!
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-f", "world", 
            "-d", "/opt/ros/humble/share/ur_description/rviz/view_robot.rviz"
        ],
        remappings=[
            ("/robot_description", "robot_description")
        ]
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)