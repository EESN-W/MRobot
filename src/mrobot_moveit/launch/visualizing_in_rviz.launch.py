from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Set MoveIt configuration
    moveit_config = MoveItConfigsBuilder("MRobot", package_name="mrobot_moveit").to_moveit_configs()

    # Define visualizing_in_rviz node
    visualizing_in_rviz = Node(
        package="mrobot_moveit",
        executable="visualizing_in_rviz",
        output="screen",
        parameters=[
            moveit_config.robot_description,  # Load URDF
            moveit_config.robot_description_semantic,  # Load SRDF
            moveit_config.robot_description_kinematics,  # Load kinematics.yaml
        ],
    )

    return LaunchDescription([visualizing_in_rviz])