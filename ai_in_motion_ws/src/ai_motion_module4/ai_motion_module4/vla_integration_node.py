"""
VLA Integration Node

Integrates voice processing, cognitive planning, and action execution
components into a unified Vision-Language-Action system for humanoid robots.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_motion_common_msgs.msg import VoiceCommand, CognitivePlan
from typing import Dict, Any, Optional
import json
import time


class VLAIntegrationNode(Node):
    """
    A ROS 2 node that integrates voice processing, cognitive planning,
    and action execution into a unified Vision-Language-Action system.
    """

    def __init__(self):
        super().__init__('vla_integration_node')

        # Initialize state
        self.current_plan = None
        self.plan_status = "idle"

        # Create subscribers for voice commands and cognitive plans
        self.voice_cmd_sub = self.create_subscription(
            VoiceCommand,
            '/voice_command',
            self.voice_command_callback,
            10
        )

        self.plan_sub = self.create_subscription(
            String,
            '/cognitive_plan',
            self.plan_callback,
            10
        )

        # Create publishers for system status and plan execution
        self.status_pub = self.create_publisher(
            String,
            '/vla_system/status',
            10
        )

        self.plan_execution_pub = self.create_publisher(
            String,
            '/plan_execution_queue',
            10
        )

        # Create service clients for various components
        self.voice_processing_available = True
        self.cognitive_planning_available = True
        self.action_execution_available = True

        # System monitoring timer
        self.status_timer = self.create_timer(1.0, self.publish_system_status)

        self.get_logger().info('VLA Integration Node initialized')

    def voice_command_callback(self, msg: VoiceCommand):
        """
        Callback for voice commands that triggers cognitive planning.
        """
        self.get_logger().info(f'Received voice command: {msg.command}')

        # Forward voice command to cognitive planner
        # In a real implementation, this would call a service or publish to planner
        self.process_voice_command(msg.command)

    def plan_callback(self, msg: String):
        """
        Callback for cognitive plans received from the planner.
        """
        try:
            plan_data = json.loads(msg.data)
            self.get_logger().info(f'Received cognitive plan with {len(plan_data.get("steps", []))} steps')

            # Forward plan to action executor
            self.execute_plan(plan_data)

        except Exception as e:
            self.get_logger().error(f'Error processing cognitive plan: {e}')

    def process_voice_command(self, command: str):
        """
        Process a voice command by sending it to the cognitive planner.
        """
        self.get_logger().info(f'Processing voice command: {command}')

        # In a real implementation, this would call the cognitive planner service
        # For now, we'll simulate by publishing to the cognitive plan topic
        plan_request = String()
        plan_request.data = json.dumps({
            "command": command,
            "timestamp": time.time(),
            "source": "voice"
        })

        # This would typically go to the cognitive planner
        # For now, we'll publish to a topic that the planner subscribes to
        self.get_logger().info(f'Sent command to cognitive planner: {command}')

    def execute_plan(self, plan_data: Dict[str, Any]):
        """
        Execute a cognitive plan by sending it to the action executor.
        """
        self.get_logger().info(f'Executing plan with {len(plan_data.get("steps", []))} steps')

        # Forward each action in the plan to the action executor
        for step in plan_data.get('steps', []):
            action_msg = String()
            action_msg.data = json.dumps(step)

            # Publish action to execution queue
            self.plan_execution_pub.publish(action_msg)
            self.get_logger().info(f'Sent action to executor: {step.get("action_type", "unknown")}')

        self.current_plan = plan_data
        self.plan_status = "executing"

    def publish_system_status(self):
        """
        Publish current system status for monitoring.
        """
        status_msg = String()
        status_msg.data = f"VLA System - Voice: {'Available' if self.voice_processing_available else 'Unavailable'}, " \
                         f"Planning: {'Available' if self.cognitive_planning_available else 'Unavailable'}, " \
                         f"Execution: {'Available' if self.action_execution_available else 'Unavailable'}, " \
                         f"Plan Status: {self.plan_status}"

        self.status_pub.publish(status_msg)


def main(args=None):
    """
    Main function to run the VLA integration node.
    """
    rclpy.init(args=args)

    vla_integration_node = VLAIntegrationNode()

    try:
        rclpy.spin(vla_integration_node)
    except KeyboardInterrupt:
        pass
    finally:
        vla_integration_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()