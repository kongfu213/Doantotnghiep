from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
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
    use_sim_time = LaunchConfiguration("use_sim_time")

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

    # 1. URDF Content
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
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=ur_onrobot",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "onrobot_type:=",
            onrobot_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # 2. SRDF Content
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

    # 3. Kinematics Config
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "ur_onrobot_moveit_config", "config/kinematics.yaml"
        )
    }

    # 4. Joint Limits Config (Khai báo giới hạn gia tốc)
    joint_limits_yaml = load_yaml("ur_onrobot_moveit_config", "config/joint_limits.yaml")
    if joint_limits_yaml is None:
        joint_limits_yaml = {}
    robot_description_planning = {
        "robot_description_planning": joint_limits_yaml
    }

    # 5. OMPL Planning Pipeline
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

    # Gom toàn bộ thông số cấu hình lại
    common_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning_pipeline_config,
        {"use_sim_time": use_sim_time},
    ]

    # 1. NODE SPAWN MÔI TRƯỜNG (Bàn & Vật thể)
    spawn_env_node = Node(
        package="ur_onrobot_mtc",
        executable="spawn_environment.py",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 2. NODE TAY TRÁI
    left_mtc_node = Node(
        package="ur_onrobot_mtc",
        executable="left_arm_mtc_node",
        name="left_mtc_node",
        output="screen",
        parameters=common_parameters,
    )

    # 3. NODE TAY PHẢI
    right_mtc_node = Node(
        package="ur_onrobot_mtc",
        executable="right_arm_mtc_node",
        name="right_mtc_node",
        output="screen",
        parameters=common_parameters,
    )

    # Trễ 2.0 giây cho MTC Nodes để đảm bảo Environment đã spawn thành công
    delayed_mtc_nodes = TimerAction(
        period=2.0,
        actions=[left_mtc_node, right_mtc_node]
    )

    return [spawn_env_node, delayed_mtc_nodes]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3e",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            description="Type/series of used UR robot.",
        ),
        DeclareLaunchArgument(
            "onrobot_type",
            default_value="rg2",
            choices=["rg2", "rg6"],
            description="Type of the OnRobot gripper.",
        ),
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        ),
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        ),
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])