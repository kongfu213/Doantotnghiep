import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dual_ur_onrobot", package_name="ur_onrobot_moveit_config")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("ur_onrobot_description"),
                "urdf",
                "dual_ur_onrobot.urdf.xacro"
            )
        )
        .robot_description_semantic(
            file_path="srdf/dual_ur_onrobot.srdf.xacro"
        )
        .to_moveit_configs()
    )

    dual_arm_controller_node = Node(
        package="ur_onrobot_mtc",
        executable="dual_arm_controller",
        output="screen",
        prefix=["xterm -e"],  # Sử dụng xterm ổn định cho ROS CLI
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([dual_arm_controller_node])