from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
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

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_onrobot_description"), "urdf", "dual_ur_onrobot.urdf.xacro"]
            ),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=", joint_limit_params,
            " ",
            "kinematics_params:=", kinematics_params,
            " ",
            "physical_params:=", physical_params,
            " ",
            "visual_params:=", visual_params,
            " ",
            "safety_limits:=", safety_limits,
            " ",
            "safety_pos_margin:=", safety_pos_margin,
            " ",
            "safety_k_position:=", safety_k_position,
            " ",
            "name:=ur_onrobot",
            " ",
            "ur_type:=", ur_type,
            " ",
            "onrobot_type:=", onrobot_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=", prefix,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur_onrobot_moveit_config"), "srdf", "dual_ur_onrobot.srdf.xacro"]
            ),
        ]
    )

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

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "ur_onrobot_moveit_config", "config/joint_limits.yaml"
        )
    }

    ompl_planning_pipeline_config = {
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
        },
    }

    ompl_yaml = load_yaml("ur_onrobot_moveit_config", "config/ompl_planning.yaml")
    if ompl_yaml:
        ompl_planning_pipeline_config["ompl"].update(ompl_yaml)

    common_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning_pipeline_config,
    ]

    right_mtc_node = Node(
        package="ur_onrobot_mtc",
        executable="right_arm_mtc_node",
        name="right_mtc_node",
        output="screen",
        parameters=common_parameters,
    )

    return [right_mtc_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("onrobot_type", default_value="rg2"),
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("prefix", default_value='""'),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])