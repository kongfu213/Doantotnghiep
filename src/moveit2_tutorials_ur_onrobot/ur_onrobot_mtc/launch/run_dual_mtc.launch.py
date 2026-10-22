from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import math

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
    task_mode = LaunchConfiguration("task_mode")

    # Robot-description files. These describe the robot; they do NOT define task poses.
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
        PathJoinSubstitution([
            FindPackageShare("ur_onrobot_description"),
            "urdf",
            "dual_ur_onrobot.urdf.xacro",
        ]), " ",
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
        "prefix:=", prefix, " ",
    ])

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([
            FindPackageShare("ur_onrobot_moveit_config"),
            "srdf",
            "dual_ur_onrobot.srdf.xacro",
        ]),
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

    joint_limits_yaml = load_yaml(
        "ur_onrobot_moveit_config", "config/joint_limits.yaml"
    ) or {}

    # Ruckig needs explicit jerk bounds; use one configurable limit for every
    # revolute joint on both arms in this simulation.
    max_joint_jerk = float(LaunchConfiguration("max_joint_jerk").perform(context))
    if not math.isfinite(max_joint_jerk) or max_joint_jerk <= 0.0:
        raise RuntimeError("max_joint_jerk must be a finite positive value in rad/s^3")
    joint_limits = joint_limits_yaml.setdefault("joint_limits", {})
    arm_joint_names = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    )
    for arm in ("left", "right"):
        for joint_name in arm_joint_names:
            limits = joint_limits.setdefault(f"{arm}_{joint_name}", {})
            limits["has_jerk_limits"] = True
            limits["max_jerk"] = max_joint_jerk

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
        },
    }

    ompl_yaml = load_yaml("ur_onrobot_moveit_config", "config/ompl_planning.yaml")
    if ompl_yaml:
        ompl_config["ompl"].update(ompl_yaml)

    # In the MoveIt request-adapter chain, the first entry runs last on the
    # trajectory, after time parameterization has assigned timestamps.
    ruckig_adapter = "default_planner_request_adapters/AddRuckigTrajectorySmoothing"
    adapters = ompl_config["ompl"].get("request_adapters", "").split()
    adapters = [adapter for adapter in adapters if adapter != ruckig_adapter]
    adapters.insert(0, ruckig_adapter)
    ompl_config["ompl"]["request_adapters"] = " ".join(adapters)

    # IMPORTANT:
    # No stack_center_x/y, no A/B/C/D task poses, no left/right place target,
    # no swap-buffer pose. Task targets come from /robot_command + PlanningScene.
    # Load mission parameters as a normal Python dict instead of passing
    # missions.yaml as an extra ROS params file. This avoids rcl YAML parser
    # failures such as "No value at line ..." while keeping the same
    # mission_names and missions.<name> parameters.
    missions_yaml = load_yaml("ur_onrobot_mtc", "config/missions.yaml") or {}
    mission_parameters = (
        missions_yaml
        .get("dual_arm_mtc_node", {})
        .get("ros__parameters", {})
    )

    common_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_config,
        {"use_sim_time": use_sim_time},
        {"execute": ParameterValue(execute, value_type=bool)},
        {"command_initialize_stack_scene": ParameterValue(
            LaunchConfiguration("command_initialize_stack_scene"), value_type=bool)},
        {"solder.holder_arm": LaunchConfiguration("holder_arm")},
        {"solder.dwell_seconds": ParameterValue(LaunchConfiguration("dwell_seconds"), value_type=float)},
        {"solder.approach_height": ParameterValue(LaunchConfiguration("approach_height"), value_type=float)},
        {"task_mode": task_mode},
        mission_parameters,
    ]

    mtc_node = Node(
        package="ur_onrobot_mtc",
        executable="dual_arm_mtc_node",
        name="dual_arm_mtc_node",
        output="screen",
        parameters=common_parameters,
    )

    # dual_arm_mtc_node initializes its own command-server scene.
    # Do not launch spawn_environment.py here; this avoids two scene owners.
    return [mtc_node]


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
            description="Execute planned motions",
        ),
        DeclareLaunchArgument(
            "task_mode",
            default_value="command_server",
            choices=[
                "command_server",
                "pick_place",
                "primitive_demo",
                "pose_demo",
                "dual_pick_demo",
                "coordination_demo",
                "stack4_demo",
            ],
            description="command_server keeps the node alive and accepts /robot_command",
        ),
        DeclareLaunchArgument("command_initialize_stack_scene", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("holder_arm", default_value="left", choices=["left", "right"]),
        DeclareLaunchArgument("dwell_seconds", default_value="1.5"),
        DeclareLaunchArgument("approach_height", default_value="0.06"),
        DeclareLaunchArgument(
            "max_joint_jerk",
            default_value="20.0",
            description="Maximum joint jerk for Ruckig smoothing in rad/s^3",
        ),
        OpaqueFunction(function=launch_setup),
    ])