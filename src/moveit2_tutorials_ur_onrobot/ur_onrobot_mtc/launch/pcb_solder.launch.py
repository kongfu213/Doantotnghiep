"""PCB scene and solder command server; requires fake controllers and move_group."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("execute", default_value="false",
                              choices=["true", "false"]),
        DeclareLaunchArgument("holder_arm", default_value="left",
                              choices=["left", "right"]),
        DeclareLaunchArgument("dwell_seconds", default_value="1.5"),
        DeclareLaunchArgument("approach_height", default_value="0.06"),
        DeclareLaunchArgument("pcb_x", default_value="0.0"),
        DeclareLaunchArgument("pcb_y", default_value="0.0"),
        DeclareLaunchArgument("table_top", default_value="-0.003"),
        DeclareLaunchArgument("pcb_lift", default_value="0.10"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    scene = Node(
        package="ur_onrobot_mtc",
        executable="setup_pcb_scene",
        output="screen",
        parameters=[{
            "prepare_table": True,
            "show_tool_overlay": False,
            **{
                name: ParameterValue(
                    LaunchConfiguration(name),
                    value_type=float
                )
                for name in ("pcb_x", "pcb_y", "table_top", "pcb_lift")
            },
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"),
                value_type=bool
            ),
        }],
    )

    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_onrobot_mtc"),
                "launch",
                "run_dual_mtc.launch.py",
            ])
        ),
        launch_arguments={
            "task_mode": "command_server",
            "command_initialize_stack_scene": "false",
            **{
                name: LaunchConfiguration(name)
                for name in (
                    "execute",
                    "holder_arm",
                    "dwell_seconds",
                    "approach_height",
                    "use_sim_time",
                )
            },
        }.items(),
    )

    return LaunchDescription(args + [scene, server])