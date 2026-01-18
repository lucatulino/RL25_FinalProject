import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('iiwa_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'collaborative_setup.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'dual_arm.rviz')

    use_save = LaunchConfiguration('use_save')
    use_save_arg = DeclareLaunchArgument('use_save', default_value='false')

    robot_description = {"robot_description": Command([FindExecutable(name="xacro"), " ", xacro_file])}

    return LaunchDescription([
        use_save_arg,
        

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description]
        ),


        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui"
        ),


        Node(
            package="rviz2",
            executable="rviz2",
            arguments=['-d', rviz_config_file],
            condition=IfCondition(use_save)
        ),


        Node(
            package="rviz2",
            executable="rviz2",
            condition=UnlessCondition(use_save)
        )
    ])
