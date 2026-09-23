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
    task_mode = LaunchConfiguration("task_mode")
    demo_parallel_dz = LaunchConfiguration("demo_parallel_dz")
    demo_single_dx = LaunchConfiguration("demo_single_dx")
    demo_joint_delta = LaunchConfiguration("demo_joint_delta")
    coord_center_shift = LaunchConfiguration("coord_center_shift")
    coord_both_dz = LaunchConfiguration("coord_both_dz")
    coord_rotate_deg = LaunchConfiguration("coord_rotate_deg")
    stack_center_x = LaunchConfiguration("stack_center_x")
    stack_center_y = LaunchConfiguration("stack_center_y")
    stack_object_size = LaunchConfiguration("stack_object_size")
    stack_gap = LaunchConfiguration("stack_gap")
    stack_pre_dz = LaunchConfiguration("stack_pre_dz")
    stack_transit_height = LaunchConfiguration("stack_transit_height")
    stack_entry_offset = LaunchConfiguration("stack_entry_offset")
    stack_a_x = LaunchConfiguration("stack_a_x")
    stack_a_y = LaunchConfiguration("stack_a_y")
    stack_b_x = LaunchConfiguration("stack_b_x")
    stack_b_y = LaunchConfiguration("stack_b_y")
    stack_c_x = LaunchConfiguration("stack_c_x")
    stack_c_y = LaunchConfiguration("stack_c_y")
    stack_d_x = LaunchConfiguration("stack_d_x")
    stack_d_y = LaunchConfiguration("stack_d_y")
    left_place_x = LaunchConfiguration("left_place_x")
    left_place_y = LaunchConfiguration("left_place_y")
    left_place_z = LaunchConfiguration("left_place_z")
    right_place_x = LaunchConfiguration("right_place_x")
    right_place_y = LaunchConfiguration("right_place_y")
    right_place_z = LaunchConfiguration("right_place_z")
    left_place_roll_deg = LaunchConfiguration("left_place_roll_deg")
    left_place_pitch_deg = LaunchConfiguration("left_place_pitch_deg")
    left_place_yaw_deg = LaunchConfiguration("left_place_yaw_deg")
    right_place_roll_deg = LaunchConfiguration("right_place_roll_deg")
    right_place_pitch_deg = LaunchConfiguration("right_place_pitch_deg")
    right_place_yaw_deg = LaunchConfiguration("right_place_yaw_deg")
    command = LaunchConfiguration("command")
    command_initialize_stack_scene = LaunchConfiguration("command_initialize_stack_scene")
    swap_buffer_x = LaunchConfiguration("swap_buffer_x")
    swap_buffer_y = LaunchConfiguration("swap_buffer_y")

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
        {"task_mode": task_mode},
        {"demo_parallel_dz": demo_parallel_dz},
        {"demo_single_dx": demo_single_dx},
        {"demo_joint_delta": demo_joint_delta},
        {"coord_center_shift": coord_center_shift},
        {"coord_both_dz": coord_both_dz},
        {"coord_rotate_deg": coord_rotate_deg},
        {"stack_center_x": stack_center_x},
        {"stack_center_y": stack_center_y},
        {"stack_object_size": stack_object_size},
        {"stack_gap": stack_gap},
        {"stack_pre_dz": stack_pre_dz},
        {"stack_transit_height": stack_transit_height},
        {"stack_entry_offset": stack_entry_offset},
        {"stack_a_x": stack_a_x},
        {"stack_a_y": stack_a_y},
        {"stack_b_x": stack_b_x},
        {"stack_b_y": stack_b_y},
        {"stack_c_x": stack_c_x},
        {"stack_c_y": stack_c_y},
        {"stack_d_x": stack_d_x},
        {"stack_d_y": stack_d_y},
        {"left_place_x": left_place_x},
        {"left_place_y": left_place_y},
        {"left_place_z": left_place_z},
        {"right_place_x": right_place_x},
        {"right_place_y": right_place_y},
        {"right_place_z": right_place_z},
        {"left_place_roll_deg": left_place_roll_deg},
        {"left_place_pitch_deg": left_place_pitch_deg},
        {"left_place_yaw_deg": left_place_yaw_deg},
        {"right_place_roll_deg": right_place_roll_deg},
        {"right_place_pitch_deg": right_place_pitch_deg},
        {"right_place_yaw_deg": right_place_yaw_deg},
        {"command": command},
        {"command_initialize_stack_scene": command_initialize_stack_scene},
        {"swap_buffer_x": swap_buffer_x},
        {"swap_buffer_y": swap_buffer_y},
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
        executable="mtc_persistent_demo",
        name="mtc_persistent_demo",
        output="screen",
        parameters=common_parameters,
    )

    delayed = TimerAction(period=2.0, actions=[mtc_node])

    return [delayed]


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
            description="Execute coordinated dual-arm task"
        ),
        DeclareLaunchArgument(
            "task_mode",
            default_value="command_server",
            choices=["command_server", "pick_place", "primitive_demo", "pose_demo", "dual_pick_demo", "coordination_demo", "stack4_demo"],
            description="command_server=keep node alive and accept pick/place/move/stack/swap from robot_cli on /robot_command"
        ),
        DeclareLaunchArgument("demo_parallel_dz", default_value="0.05"),
        DeclareLaunchArgument("demo_single_dx", default_value="0.05"),
        DeclareLaunchArgument("demo_joint_delta", default_value="0.08"),
        DeclareLaunchArgument("coord_center_shift", default_value="0.05"),
        DeclareLaunchArgument("coord_both_dz", default_value="0.03"),
        DeclareLaunchArgument("coord_rotate_deg", default_value="20.0"),
        DeclareLaunchArgument("stack_center_x", default_value="0.0"),
        DeclareLaunchArgument("stack_center_y", default_value="0.0"),
        DeclareLaunchArgument("stack_object_size", default_value="0.04"),
        DeclareLaunchArgument("stack_gap", default_value="0.003"),
        DeclareLaunchArgument("stack_pre_dz", default_value="0.08"),
        DeclareLaunchArgument("stack_transit_height", default_value="0.22"),
        DeclareLaunchArgument("stack_entry_offset", default_value="0.10"),
        DeclareLaunchArgument("stack_a_x", default_value="0.30"),
        DeclareLaunchArgument("stack_a_y", default_value="0.25"),
        DeclareLaunchArgument("stack_b_x", default_value="-0.25"),
        DeclareLaunchArgument("stack_b_y", default_value="0.25"),
        DeclareLaunchArgument("stack_c_x", default_value="0.30"),
        DeclareLaunchArgument("stack_c_y", default_value="-0.25"),
        DeclareLaunchArgument("stack_d_x", default_value="-0.25"),
        DeclareLaunchArgument("stack_d_y", default_value="-0.25"),
        DeclareLaunchArgument("left_place_x", default_value="-0.25"),
        DeclareLaunchArgument("left_place_y", default_value="0.10"),
        DeclareLaunchArgument("left_place_z", default_value="0.052"),
        DeclareLaunchArgument("right_place_x", default_value="-0.25"),
        DeclareLaunchArgument("right_place_y", default_value="-0.10"),
        DeclareLaunchArgument("right_place_z", default_value="0.052"),
        DeclareLaunchArgument("left_place_roll_deg", default_value="0.0"),
        DeclareLaunchArgument("left_place_pitch_deg", default_value="0.0"),
        DeclareLaunchArgument("left_place_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("right_place_roll_deg", default_value="0.0"),
        DeclareLaunchArgument("right_place_pitch_deg", default_value="0.0"),
        DeclareLaunchArgument("right_place_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("command", default_value=""),
        DeclareLaunchArgument(
            "command_initialize_stack_scene",
            default_value="true",
            choices=["true", "false"],
            description="Create table + stack_A/B/C/D automatically when command_server starts"
        ),
        DeclareLaunchArgument("swap_buffer_x", default_value="0.0"),
        DeclareLaunchArgument("swap_buffer_y", default_value="0.36"),
        OpaqueFunction(function=launch_setup),
    ])
