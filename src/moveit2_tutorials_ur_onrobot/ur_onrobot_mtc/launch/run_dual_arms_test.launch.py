from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ur_onrobot_moveit_config.launch_common import load_yaml


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    execute = LaunchConfiguration("execute")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_description"), "urdf", "dual_ur_onrobot.urdf.xacro"]
        ), " ",
        "robot_ip:=xxx.yyy.zzz.www", " ",
        "joint_limit_params:=", joint_limit_params, " ",
        "kinematics_params:=", kinematics_params, " ",
        "physical_params:=", physical_params, " ",
        "visual_params:=", visual_params, " ",
        "safety_limits:=", safety_limits, " ",
        "safety_pos_margin:=", safety_pos_margin, " ",
        "safety_k_position:=", safety_k_position, " ",
        "name:=ur_onrobot", " ",
        "ur_type:=", ur_type, " ",
        "onrobot_type:=", onrobot_type, " ",
        "script_filename:=ros_control.urscript", " ",
        "input_recipe_filename:=rtde_input_recipe.txt", " ",
        "output_recipe_filename:=rtde_output_recipe.txt", " ",
        "prefix:=", prefix, " "
    ])

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution(
            [FindPackageShare("ur_onrobot_moveit_config"), "srdf", "dual_ur_onrobot.srdf.xacro"]
        )
    ])

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "ur_onrobot_moveit_config", "config/kinematics.yaml"
        )
    }

    joint_limits_yaml = load_yaml("ur_onrobot_moveit_config", "config/joint_limits.yaml")
    if joint_limits_yaml is None:
        joint_limits_yaml = {}

    robot_description_planning = {
        "robot_description_planning": joint_limits_yaml
    }

    ompl_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_yaml = load_yaml("ur_onrobot_moveit_config", "config/ompl_planning.yaml")
    if ompl_yaml:
        ompl_config["ompl"].update(ompl_yaml)

    common_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_config,
        {"use_sim_time": use_sim_time},
        {"execute": execute},
    ]

    spawn_env = Node(
        package="ur_onrobot_mtc",
        executable="spawn_environment.py",
        name="spawn_environment_dual_mtc",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    mtc_node = Node(
        package="ur_onrobot_mtc",
        executable="dual_arm_mtc_node",
        name="dual_arm_mtc_node",
        output="screen",
        parameters=common_parameters,
    )

    delayed = TimerAction(period=2.0, actions=[mtc_node])

    return [spawn_env, delayed]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("onrobot_type", default_value="rg2"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "execute",
            default_value="false",
            choices=["true", "false"],
            description="Execute coordinated dual-arm MTC pick + lift after planning"
        ),
        OpaqueFunction(function=launch_setup),
    ])
