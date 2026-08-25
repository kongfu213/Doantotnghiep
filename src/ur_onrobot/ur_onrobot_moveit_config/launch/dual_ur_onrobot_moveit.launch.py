import os

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ur_onrobot_moveit_config.launch_common import load_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def launch_setup(context, *args, **kwargs):

    # Khởi tạo các tham số
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")

    ur_description_package = LaunchConfiguration("ur_description_package")
    description_file = LaunchConfiguration("description_file")
    _publish_robot_description_semantic = LaunchConfiguration("publish_robot_description_semantic")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    warehouse_sqlite_path = LaunchConfiguration("warehouse_sqlite_path")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_servo = LaunchConfiguration("launch_servo")

    # 1. Robot Description (Đã bọc ParameterValue để ép kiểu chuỗi)
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution([FindPackageShare(ur_description_package), "urdf", description_file]),
                " ",
                "ur_type:=", ur_type, " ",
                "onrobot_type:=", onrobot_type, " ",
                "safety_limits:=", safety_limits, " ",
                "safety_pos_margin:=", safety_pos_margin, " ",
                "safety_k_position:=", safety_k_position, " ",
                "use_fake_hardware:=true", " ",
                "prefix:=", prefix,
            ]
        ),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # 2. Semantic Description (SRDF - Đã bọc ParameterValue để sửa lỗi YAML)
    robot_description_semantic_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
                ),
                " ",
                "name:=", "ur_onrobot", " ",
                "prefix:=", prefix, " "
            ]
        ),
        value_type=str
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    
    publish_robot_description_semantic = {
        "publish_robot_description_semantic": _publish_robot_description_semantic
    }

    # 3. Kinematics & Planning
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "ur_onrobot_moveit_config", "config/kinematics.yaml"
        )
    }

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", str(moveit_joint_limits_file.perform(context))),
        )
    }

    # 4. OMPL Planning Pipeline
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_onrobot_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # 5. MoveIt Controllers
    controllers_yaml = load_yaml("ur_onrobot_moveit_config", "config/controllers.yaml")
    
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # 6. Trajectory Execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    warehouse_ros_config = {
        "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
        "warehouse_host": warehouse_sqlite_path,
    }

    # --- CÁC NODE KHỞI CHẠY ---

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 2. ROS 2 Control Node (Cơ bắp ảo)
    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "ros2_controllers.yaml"]
    )

    # 3. Các Spawner kích hoạt động cơ cho 2 tay
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    left_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_joint_trajectory_controller", "-c", "/controller_manager"],
    )
    right_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_joint_trajectory_controller", "-c", "/controller_manager"],
    )
    left_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_finger_width_trajectory_controller", "-c", "/controller_manager"],
    )
    right_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_finger_width_trajectory_controller", "-c", "/controller_manager"],
    )

    # 4. MoveIt Node (Bộ não)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
            warehouse_ros_config,
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"}
        ],
    )

    # 5. RViz (Giao diện)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            warehouse_ros_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # 6. Servo Node
    servo_yaml = load_yaml("ur_onrobot_moveit_config", "config/ur_onrobot_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        output="screen",
    )

    # Trả về tất cả các node để chạy đồng thời
    return [
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
        servo_node
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("ur_type", default_value="ur3e", description="Type of UR robot."))
    declared_arguments.append(DeclareLaunchArgument("onrobot_type", default_value="rg2", description="Type of OnRobot gripper."))
    declared_arguments.append(DeclareLaunchArgument("safety_limits", default_value="true", description="Enables safety limits."))
    declared_arguments.append(DeclareLaunchArgument("safety_pos_margin", default_value="0.15", description="Margin for shoulder joints."))
    declared_arguments.append(DeclareLaunchArgument("safety_k_position", default_value="20", description="k-position factor."))
    declared_arguments.append(DeclareLaunchArgument("ur_description_package", default_value="ur_onrobot_description", description="Package for description."))
    declared_arguments.append(DeclareLaunchArgument("description_file", default_value="dual_ur_onrobot.urdf.xacro", description="URDF file."))
    declared_arguments.append(DeclareLaunchArgument("publish_robot_description_semantic", default_value="true", description="Publish SRDF."))
    declared_arguments.append(DeclareLaunchArgument("moveit_config_package", default_value="ur_onrobot_moveit_config", description="MoveIt package."))
    declared_arguments.append(DeclareLaunchArgument("moveit_config_file", default_value="dual_ur_onrobot.srdf.xacro", description="SRDF file."))
    declared_arguments.append(DeclareLaunchArgument("moveit_joint_limits_file", default_value="joint_limits.yaml", description="Limits file."))
    declared_arguments.append(DeclareLaunchArgument("warehouse_sqlite_path", default_value=os.path.expanduser("~/.ros/warehouse_ros.sqlite"), description="Path for DB."))
    declared_arguments.append(DeclareLaunchArgument("use_sim_time", default_value="false", description="Use sim time."))
    declared_arguments.append(DeclareLaunchArgument("prefix", default_value="", description="Prefix."))
    
    declared_arguments.append(DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?"))
    declared_arguments.append(DeclareLaunchArgument("launch_servo", default_value="false", description="Launch Servo?"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])