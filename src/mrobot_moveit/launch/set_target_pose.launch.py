from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Set MoveIt configuration
    moveit_config = MoveItConfigsBuilder("MRobot", package_name="mrobot_moveit").to_moveit_configs()

    # Define hello_moveit node
    set_target_pose_node = Node(
        package="mrobot_moveit",
        executable="set_target_pose",
        output="screen",
        parameters=[
            moveit_config.robot_description,  # Load URDF
            moveit_config.robot_description_semantic,  # Load SRDF
            moveit_config.robot_description_kinematics,  # Load kinematics.yaml
        ],
    )

    return LaunchDescription([set_target_pose_node])