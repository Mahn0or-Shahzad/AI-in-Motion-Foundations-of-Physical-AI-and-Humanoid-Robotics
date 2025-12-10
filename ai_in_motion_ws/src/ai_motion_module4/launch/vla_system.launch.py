"""
VLA System Launch File

Launch file for Vision-Language-Action system with voice processing,
cognitive planning, and action execution.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package names
    pkg_ai_motion_module4 = get_package_share_directory('ai_motion_module4')
    pkg_ai_motion_module1 = get_package_share_directory('ai_motion_module1')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_voice_processing = LaunchConfiguration('enable_voice_processing', default='true')
    enable_cognitive_planning = LaunchConfiguration('enable_cognitive_planning', default='true')
    enable_action_execution = LaunchConfiguration('enable_action_execution', default='true')
    openai_api_key = LaunchConfiguration('openai_api_key', default='')

    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_enable_voice_arg = DeclareLaunchArgument(
        'enable_voice_processing',
        default_value='true',
        description='Enable voice processing components'
    )

    declare_enable_cognitive_arg = DeclareLaunchArgument(
        'enable_cognitive_planning',
        default_value='true',
        description='Enable cognitive planning components'
    )

    declare_enable_action_arg = DeclareLaunchArgument(
        'enable_action_execution',
        default_value='true',
        description='Enable action execution components'
    )

    declare_openai_api_key_arg = DeclareLaunchArgument(
        'openai_api_key',
        default_value='',
        description='OpenAI API key for Whisper and LLM integration'
    )

    # Voice interface node
    voice_interface_node = Node(
        package='ai_motion_module4',
        executable='voice_interface_node',
        name='voice_interface_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([FindPackageShare('ai_motion_module4'), 'config', 'vla_config.yaml'])
        ],
        condition=IfCondition(enable_voice_processing),
        output='screen'
    )

    # Cognitive planner node
    cognitive_planner_node = Node(
        package='ai_motion_module4',
        executable='cognitive_planner_node',
        name='cognitive_planner_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([FindPackageShare('ai_motion_module4'), 'config', 'vla_config.yaml'])
        ],
        condition=IfCondition(enable_cognitive_planning),
        output='screen'
    )

    # Action executor node
    action_executor_node = Node(
        package='ai_motion_module4',
        executable='action_executor_node',
        name='action_executor_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([FindPackageShare('ai_motion_module4'), 'config', 'vla_config.yaml'])
        ],
        condition=IfCondition(enable_action_execution),
        output='screen'
    )

    # Intent classifier node
    intent_classifier_node = Node(
        package='ai_motion_module4',
        executable='intent_classifier_node',
        name='intent_classifier_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([FindPackageShare('ai_motion_module4'), 'config', 'vla_config.yaml'])
        ],
        condition=IfCondition(enable_voice_processing),
        output='screen'
    )

    # VLA integration node
    vla_integration_node = Node(
        package='ai_motion_module4',
        executable='vla_integration_node',
        name='vla_integration_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([FindPackageShare('ai_motion_module4'), 'config', 'vla_config.yaml'])
        ],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_enable_voice_arg)
    ld.add_action(declare_enable_cognitive_arg)
    ld.add_action(declare_enable_action_arg)
    ld.add_action(declare_openai_api_key_arg)

    # Add nodes
    ld.add_action(voice_interface_node)
    ld.add_action(cognitive_planner_node)
    ld.add_action(action_executor_node)
    ld.add_action(intent_classifier_node)
    ld.add_action(vla_integration_node)

    return ld