import os
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

def launch_setup(context, *args, **kwargs):
    # Lấy các tham số runtime
    ur_type = LaunchConfiguration("ur_type")
    onrobot_type = LaunchConfiguration("onrobot_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    
    # IP riêng biệt cho hệ 2 tay thực tế
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")

    # --- BỔ SUNG CÁC THAM SỐ FILE MẶC ĐỊNH CHO ROBOT UR ---
    ur_description_package = FindPackageShare("ur_description")
    
    joint_limits_parameters_file = PathJoinSubstitution([ur_description_package, "config", ur_type, "joint_limits.yaml"])
    kinematics_parameters_file = PathJoinSubstitution([ur_description_package, "config", ur_type, "default_kinematics.yaml"])
    physical_parameters_file = PathJoinSubstitution([ur_description_package, "config", ur_type, "physical_parameters.yaml"])
    visual_parameters_file = PathJoinSubstitution([ur_description_package, "config", ur_type, "visual_parameters.yaml"])

    # Biên dịch file Xacro gộp với ĐẦY ĐỦ tham số để tránh lỗi gãy cấu trúc phần cứng
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare('ur_onrobot_description'), "urdf", 'dual_ur_onrobot.urdf.xacro']), " ",
        "ur_type:=", ur_type, " ",
        "onrobot_type:=", onrobot_type, " ",
        "use_fake_hardware:=", use_fake_hardware, " ",
        "joint_limits_parameters_file:=", joint_limits_parameters_file, " ",
        "kinematics_parameters_file:=", kinematics_parameters_file, " ",
        "physical_parameters_file:=", physical_parameters_file, " ",
        "visual_parameters_file:=", visual_parameters_file, " ",
        "left_robot_ip:=", left_robot_ip, " ",
        "right_robot_ip:=", right_robot_ip, " ",
    ])
    robot_description = {"robot_description": ParameterValue(value=robot_description_content, value_type=str)}

    # File cấu hình controller tổng cho cả 2 tay
    initial_joint_controllers = PathJoinSubstitution([
        FindPackageShare('ur_onrobot_control'), "config", 'dual_ur_onrobot_controllers.yaml'
    ])

    # Node quản lý trung tâm (Sửa lại nạp tham số phẳng để tránh lỗi sập exit code -6)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description, 
            ParameterFile(initial_joint_controllers, allow_substs=True)
        ],
        output="screen",
        condition=IfCondition(use_fake_hardware),
    )

    # Node Robot State Publisher chung cập nhật TF tree cho hệ thống
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # --- HÀM TẠO SPAWNER CONTROLLER ---
    def make_spawner(controller_name, activate=True):
        arguments = [controller_name, "-c", "/controller_manager"]
        if not activate:
            arguments.append("--inactive")
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=arguments,
            output="screen"
        )

    # Phân nhóm controller active/inactive động theo tham số đầu vào
    active_list = ["joint_state_broadcaster"]
    inactive_list = []

    selected_controller = initial_joint_controller.perform(context)
    
    # Tay trái
    if selected_controller == "scaled_joint_trajectory_controller":
        active_list.extend(["left_scaled_joint_trajectory_controller", "left_finger_width_trajectory_controller"])
        inactive_list.append("left_joint_trajectory_controller")
    else:
        active_list.extend(["left_joint_trajectory_controller", "left_finger_width_trajectory_controller"])
        inactive_list.append("left_scaled_joint_trajectory_controller")

    # Tay phải
    if selected_controller == "scaled_joint_trajectory_controller":
        active_list.extend(["right_scaled_joint_trajectory_controller", "right_finger_width_trajectory_controller"])
        inactive_list.append("right_joint_trajectory_controller")
    else:
        active_list.extend(["right_joint_trajectory_controller", "right_finger_width_trajectory_controller"])
        inactive_list.append("right_scaled_joint_trajectory_controller")

    # Khởi tạo toàn bộ các node spawner
    spawner_nodes = []
    for c in active_list:
        spawner_nodes.append(make_spawner(c, activate=True))
    for c in inactive_list:
        spawner_nodes.append(make_spawner(c, activate=False))

    # Áp dụng kĩ thuật hoãn binh 6.0 giây để hệ thống gộp ổn định luồng dịch vụ trước khi nạp controller
    delayed_spawners = TimerAction(period=6.0, actions=spawner_nodes)

    # Node hiển thị RViz2
    rviz_config_file = PathJoinSubstitution([FindPackageShare('ur_onrobot_description'), "rviz", "view_robot.rviz"])
    rviz_node = Node(package="rviz2", executable="rviz2", output="log", arguments=["-d", rviz_config_file])

    return [control_node, robot_state_publisher_node, delayed_spawners, rviz_node]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("onrobot_type", default_value="rg2"),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.56.101"),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.56.102"),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("initial_joint_controller", default_value="joint_trajectory_controller"),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])