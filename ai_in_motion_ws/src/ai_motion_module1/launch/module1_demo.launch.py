"""
Module 1 Demo Launch File

Launch file for demonstrating Module 1 ROS 2 concepts:
- Joint control
- Topic communication
- Service communication
- URDF model loading
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    # Get the launch directory
    pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Joint control demo node
    joint_control_demo = Node(
        package='ai_motion_module1',
        executable='joint_control_demo',
        name='joint_control_demo',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Topic demo node (publisher and subscriber)
    topic_demo = Node(
        package='ai_motion_module1',
        executable='topic_demo',
        name='topic_demo',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Joint service node
    joint_service = Node(
        package='ai_motion_module1',
        executable='joint_service',
        name='joint_service',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='ai_motion_module1',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Robot state publisher to publish the URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description':
                f'$(find ai_motion_module1)/urdf/humanoid_robot.urdf'}
        ],
        output='screen'
    )

    # RViz2 for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'module1_demo.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)

    # Add the actions to launch
    ld.add_action(joint_control_demo)
    ld.add_action(topic_demo)
    ld.add_action(joint_service)
    ld.add_action(joint_state_publisher)
    ld.add_action(robot_state_publisher)
    # ld.add_action(rviz_node)  # Commented out as rviz config doesn't exist yet

    return ld