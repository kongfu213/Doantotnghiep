from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("frame_id", default_value="world"),
        DeclareLaunchArgument("table_top", default_value="-0.003"),
        DeclareLaunchArgument("pcb_x", default_value="0.0"),
        DeclareLaunchArgument("pcb_y", default_value="0.0"),

        Node(
            package="ur_onrobot_mtc",
            executable="setup_pcb_scene",
            name="setup_pcb_scene",
            output="screen",
            parameters=[{
                "frame_id": ParameterValue(LaunchConfiguration("frame_id"), value_type=str),
                "table_top": ParameterValue(LaunchConfiguration("table_top"), value_type=float),
                "pcb_x": ParameterValue(LaunchConfiguration("pcb_x"), value_type=float),
                "pcb_y": ParameterValue(LaunchConfiguration("pcb_y"), value_type=float),
            }],
        ),
    ])
