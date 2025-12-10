"""
Isaac Sim Launch File

Launch file for Isaac Sim environment with VSLAM and navigation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package names
    pkg_ai_motion_module3 = get_package_share_directory('ai_motion_module3')
    pkg_ai_motion_module1 = get_package_share_directory('ai_motion_module1')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup').find('nav2_bringup')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=PathJoinSubstitution([
        FindPackageShare('ai_motion_module3'), 'config', 'nav2_config.yaml'
    ]))
    world = LaunchConfiguration('world', default=PathJoinSubstitution([
        FindPackageShare('ai_motion_module3'), 'worlds', 'obstacle_course.usd'
    ]))

    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'
    )

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ai_motion_module3'), 'config', 'nav2_config.yaml'
        ]),
        description='Full path to the navigation parameters file to use'
    )

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('ai_motion_module3'), 'worlds', 'obstacle_course.usd'
        ]),
        description='Full path to the Isaac Sim USD world file'
    )

    # Set parameters for Nav2
    set_params = SetParameter(name='use_sim_time', value=use_sim_time)

    # Robot State Publisher node to publish the robot URDF
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[os.path.join(pkg_ai_motion_module1, 'urdf', 'humanoid_robot.urdf')]
    )

    # Joint State Publisher node
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Isaac Sim VSLAM node
    vslam_node = Node(
        package='ai_motion_module3',
        executable='vslam_node',
        name='vslam_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([FindPackageShare('ai_motion_module3'), 'config', 'vslam_config.yaml'])
        ],
        output='screen'
    )

    # Isaac Navigation node
    isaac_nav_node = Node(
        package='ai_motion_module3',
        executable='isaac_nav_node',
        name='isaac_nav_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([FindPackageShare('ai_motion_module3'), 'config', 'nav2_config.yaml'])
        ],
        output='screen'
    )

    # Bipedal Navigation controller
    bipedal_controller = Node(
        package='ai_motion_module3',
        executable='nav2_bipedal_controller',
        name='nav2_bipedal_controller',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([FindPackageShare('ai_motion_module3'), 'config', 'nav2_config.yaml'])
        ],
        output='screen'
    )

    # Obstacle course demo node
    obstacle_demo = Node(
        package='ai_motion_module3',
        executable='obstacle_course_demo',
        name='obstacle_course_demo',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Navigation 2 bringup (this would include map server, localization, planning, etc.)
    navigation2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(declare_world_arg)

    # Add parameter setting
    ld.add_action(set_params)

    # Add robot state publishing
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)

    # Add Isaac-specific nodes
    ld.add_action(vslam_node)
    ld.add_action(isaac_nav_node)
    ld.add_action(bipedal_controller)
    ld.add_action(obstacle_demo)

    # Add navigation bringup
    ld.add_action(navigation2_bringup_cmd)

    return ld