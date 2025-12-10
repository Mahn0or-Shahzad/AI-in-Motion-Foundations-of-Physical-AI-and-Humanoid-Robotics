"""
Gazebo Simulation Launch File

Launch file for Gazebo simulation environment with physics, sensors,
and humanoid robot model.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package names
    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')
    pkg_ai_motion_module2 = get_package_share_directory('ai_motion_module2')
    pkg_ai_motion_module1 = get_package_share_directory('ai_motion_module1')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=PathJoinSubstitution([
        FindPackageShare('ai_motion_module2'), 'worlds', 'simple_room.world'
    ]))

    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('ai_motion_module2'), 'worlds', 'simple_room.world'
        ]),
        description='SDF world file'
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

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

    # Gazebo ROS spawn entity node to spawn the robot in Gazebo
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Sensor simulation nodes
    lidar_sim_node = Node(
        package='ai_motion_module2',
        executable='lidar_sim',
        name='lidar_sim',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    camera_sim_node = Node(
        package='ai_motion_module2',
        executable='camera_sim',
        name='camera_sim',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    imu_sim_node = Node(
        package='ai_motion_module2',
        executable='imu_sim',
        name='imu_sim',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_world_arg)

    # Add simulation launch commands
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)  # Comment out for headless simulation
    ld.add_action(spawn_entity_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(joint_state_publisher_cmd)

    # Add sensor simulation nodes
    ld.add_action(lidar_sim_node)
    ld.add_action(camera_sim_node)
    ld.add_action(imu_sim_node)

    return ld