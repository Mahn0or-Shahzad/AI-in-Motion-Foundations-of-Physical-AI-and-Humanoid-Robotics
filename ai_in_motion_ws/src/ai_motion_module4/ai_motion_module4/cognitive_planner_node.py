"""
Cognitive Planner Node

Implements cognitive planning using LLMs to convert natural language
to executable robot actions for humanoid robots.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_motion_common_msgs.msg import VoiceCommand, NavigationGoal
from ai_motion_common_msgs.srv import ProcessVoiceCommand
from geometry_msgs.msg import PoseStamped, Point
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from typing import Dict, List, Any, Optional
import openai
import json
import re
import time
from enum import Enum


class TaskStatus(Enum):
    PENDING = "pending"
    PLANNING = "planning"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"


class PlanStep:
    """
    Represents a single step in a cognitive plan.
    """
    def __init__(self, action_type: str, parameters: Dict[str, Any], description: str = ""):
        self.action_type = action_type
        self.parameters = parameters
        self.description = description
        self.status = TaskStatus.PENDING
        self.execution_time = None
        self.error_message = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "action_type": self.action_type,
            "parameters": self.parameters,
            "description": self.description,
            "status": self.status.value,
            "execution_time": self.execution_time,
            "error_message": self.error_message
        }


class CognitivePlan:
    """
    Represents a complete cognitive plan generated from natural language.
    """
    def __init__(self, natural_language: str):
        self.natural_language = natural_language
        self.steps: List[PlanStep] = []
        self.status = TaskStatus.PENDING
        self.confidence = 0.0
        self.created_time = time.time()
        self.completed_time = None

    def add_step(self, step: PlanStep):
        self.steps.append(step)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "natural_language": self.natural_language,
            "steps": [step.to_dict() for step in self.steps],
            "status": self.status.value,
            "confidence": self.confidence,
            "created_time": self.created_time,
            "completed_time": self.completed_time
        }


class CognitivePlannerNode(Node):
    """
    A ROS 2 node that implements cognitive planning using LLMs.
    """

    def __init__(self):
        super().__init__('cognitive_planner_node')

        # Initialize OpenAI API key from parameter or environment
        self.declare_parameter('openai_api_key', '')
        self.api_key = self.get_parameter('openai_api_key').value or os.getenv('OPENAI_API_KEY')

        if not self.api_key:
            self.get_logger().warn('OpenAI API key not set. Cognitive planning will not work.')
        else:
            openai.api_key = self.api_key

        # LLM settings
        self.llm_model = "gpt-3.5-turbo"  # Can be upgraded to gpt-4 if needed
        self.temperature = 0.3
        self.max_tokens = 500
        self.top_p = 1.0
        self.frequency_penalty = 0.0
        self.presence_penalty = 0.0

        # Supported intents and action mappings
        self.supported_intents = [
            "navigation", "manipulation", "perception", "status_request",
            "stop", "help", "pickup_object", "place_object", "follow_me",
            "go_to_location", "inspect_object", "report_status"
        ]

        self.action_mappings = {
            "navigation": {
                "keywords": ["go to", "move to", "navigate to", "walk to", "travel to", "kitchen", "bedroom", "office"],
                "parameters": ["location", "destination", "target"]
            },
            "manipulation": {
                "keywords": ["pick up", "grab", "take", "place", "put down", "drop", "lift", "hold"],
                "parameters": ["object", "location", "action"]
            },
            "perception": {
                "keywords": ["look at", "find", "detect", "identify", "what is", "describe", "see", "show me"],
                "parameters": ["object", "location", "attribute"]
            }
        }

        # Create subscriber for voice commands
        self.voice_command_sub = self.create_subscription(
            VoiceCommand,
            '/voice_commands',
            self.voice_command_callback,
            10
        )

        # Create publisher for navigation goals
        self.navigation_goal_pub = self.create_publisher(
            NavigationGoal,
            '/navigation/goal',
            10
        )

        # Create publisher for plan status
        self.plan_status_pub = self.create_publisher(
            String,
            '/cognitive_plan/status',
            10
        )

        # Navigation action client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Current active plan
        self.current_plan: Optional[CognitivePlan] = None
        self.active_plan_lock = threading.Lock()

        # Planning timer
        self.planning_timer = self.create_timer(0.1, self.planning_loop)

        self.get_logger().info('Cognitive Planner Node initialized')

    def voice_command_callback(self, msg: VoiceCommand):
        """
        Callback for incoming voice commands to generate cognitive plans.
        """
        self.get_logger().info(f'Received voice command: "{msg.transcription}" with intent "{msg.intent}"')

        # Generate cognitive plan based on voice command
        plan = self.generate_cognitive_plan(msg.transcription, msg.intent, msg.parameters)

        if plan:
            self.execute_plan(plan)
        else:
            self.get_logger().error(f'Failed to generate plan for command: {msg.transcription}')

    def generate_cognitive_plan(self, natural_language: str, intent: str, parameters: Dict[str, Any]) -> Optional[CognitivePlan]:
        """
        Generate a cognitive plan from natural language using LLM.
        """
        if not self.api_key:
            self.get_logger().error('OpenAI API key not set, cannot generate plan')
            return None

        try:
            # Create the plan
            plan = CognitivePlan(natural_language)

            # Generate plan using LLM
            if intent == "navigation" or "go to" in natural_language.lower():
                plan = self.generate_navigation_plan(plan, natural_language, parameters)
            elif intent == "manipulation" or any(kw in natural_language.lower() for kw in self.action_mappings["manipulation"]["keywords"]):
                plan = self.generate_manipulation_plan(plan, natural_language, parameters)
            elif intent == "perception" or any(kw in natural_language.lower() for kw in self.action_mappings["perception"]["keywords"]):
                plan = self.generate_perception_plan(plan, natural_language, parameters)
            else:
                # Use LLM to determine the best plan for unknown intents
                plan = self.generate_general_plan(plan, natural_language)

            # Calculate confidence based on how well we understood the command
            plan.confidence = self.calculate_plan_confidence(plan, natural_language)

            # Validate the plan
            if self.validate_plan(plan):
                plan.status = TaskStatus.PLANNING
                self.get_logger().info(f'Generated plan with {len(plan.steps)} steps for: {natural_language}')
                return plan
            else:
                self.get_logger().error(f'Generated plan failed validation: {natural_language}')
                return None

        except Exception as e:
            self.get_logger().error(f'Error generating cognitive plan: {e}')
            return None

    def generate_navigation_plan(self, plan: CognitivePlan, natural_language: str, parameters: Dict[str, Any]) -> CognitivePlan:
        """
        Generate a navigation plan from natural language.
        """
        # Extract destination from natural language or parameters
        destination = self.extract_destination(natural_language, parameters)

        if destination:
            # Create navigation step
            nav_step = PlanStep(
                action_type="navigation",
                parameters={"destination": destination},
                description=f"Navigate to {destination}"
            )
            plan.add_step(nav_step)
        else:
            self.get_logger().warn(f'Could not extract destination from: {natural_language}')

        return plan

    def generate_manipulation_plan(self, plan: CognitivePlan, natural_language: str, parameters: Dict[str, Any]) -> CognitivePlan:
        """
        Generate a manipulation plan from natural language.
        """
        # Extract object and action from natural language or parameters
        obj = parameters.get("object", self.extract_object(natural_language))
        action = parameters.get("action", self.extract_action(natural_language))
        location = parameters.get("location", self.extract_location(natural_language))

        if obj and action:
            # Create manipulation step
            manip_step = PlanStep(
                action_type="manipulation",
                parameters={"object": obj, "action": action, "location": location},
                description=f"{action.capitalize()} {obj}"
            )
            plan.add_step(manip_step)
        else:
            self.get_logger().warn(f'Could not extract complete manipulation from: {natural_language}')

        return plan

    def generate_perception_plan(self, plan: CognitivePlan, natural_language: str, parameters: Dict[str, Any]) -> CognitivePlan:
        """
        Generate a perception plan from natural language.
        """
        # Extract target from natural language or parameters
        target = parameters.get("object", self.extract_object(natural_language))
        attribute = parameters.get("attribute", self.extract_attribute(natural_language))

        if target:
            # Create perception step
            percep_step = PlanStep(
                action_type="perception",
                parameters={"target": target, "attribute": attribute},
                description=f"Perceive {target} ({attribute if attribute else 'all attributes'})"
            )
            plan.add_step(percep_step)
        else:
            self.get_logger().warn(f'Could not extract perception target from: {natural_language}')

        return plan

    def generate_general_plan(self, plan: CognitivePlan, natural_language: str) -> CognitivePlan:
        """
        Use LLM to generate a general plan for complex or unknown commands.
        """
        try:
            prompt = f"""
            Given the following natural language command, generate a sequence of robot actions that would fulfill the request.
            The command is: "{natural_language}"

            Please return a JSON list of steps, where each step has:
            - action_type: one of ["navigation", "manipulation", "perception", "status_request", "stop", "help"]
            - parameters: a dictionary of parameters needed for the action
            - description: a human-readable description of the step

            Example response format:
            [
                {{
                    "action_type": "navigation",
                    "parameters": {{"destination": "kitchen"}},
                    "description": "Navigate to kitchen"
                }},
                {{
                    "action_type": "perception",
                    "parameters": {{"target": "apple"}},
                    "description": "Look for apple"
                }}
            ]
            """

            response = openai.ChatCompletion.create(
                model=self.llm_model,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant that converts natural language commands into robot action sequences."},
                    {"role": "user", "content": prompt}
                ],
                temperature=self.temperature,
                max_tokens=self.max_tokens,
                response_format={"type": "json_object"}
            )

            steps_json = response.choices[0].message.content
            steps = json.loads(steps_json)

            for step_data in steps:
                step = PlanStep(
                    action_type=step_data["action_type"],
                    parameters=step_data["parameters"],
                    description=step_data["description"]
                )
                plan.add_step(step)

            return plan

        except Exception as e:
            self.get_logger().error(f'Error generating general plan with LLM: {e}')
            # Fallback: try to parse common patterns
            return self.fallback_plan_generation(plan, natural_language)

    def fallback_plan_generation(self, plan: CognitivePlan, natural_language: str) -> CognitivePlan:
        """
        Fallback plan generation when LLM fails.
        """
        # Simple pattern matching as fallback
        if "go to" in natural_language.lower() or "navigate to" in natural_language.lower():
            destination = self.extract_destination(natural_language, {})
            if destination:
                step = PlanStep(
                    action_type="navigation",
                    parameters={"destination": destination},
                    description=f"Navigate to {destination}"
                )
                plan.add_step(step)

        return plan

    def calculate_plan_confidence(self, plan: CognitivePlan, natural_language: str) -> float:
        """
        Calculate confidence in the generated plan based on various factors.
        """
        confidence = 0.5  # Base confidence

        # Increase confidence if we have clear destination/object
        for step in plan.steps:
            if step.action_type == "navigation" and "destination" in step.parameters:
                confidence += 0.2
            elif step.action_type == "manipulation" and "object" in step.parameters:
                confidence += 0.2
            elif step.action_type == "perception" and "target" in step.parameters:
                confidence += 0.1

        # Cap confidence at 1.0
        return min(1.0, confidence)

    def validate_plan(self, plan: CognitivePlan) -> bool:
        """
        Validate the generated plan for feasibility and safety.
        """
        if not plan.steps:
            return False

        # Check that all steps have required parameters
        for step in plan.steps:
            if not step.action_type or not step.parameters:
                return False

        # Additional validation rules can be added here
        return True

    def execute_plan(self, plan: CognitivePlan):
        """
        Execute the cognitive plan step by step.
        """
        with self.active_plan_lock:
            self.current_plan = plan
            self.current_plan.status = TaskStatus.EXECUTING

        self.get_logger().info(f'Executing plan with {len(plan.steps)} steps')

        # Execute each step in the plan
        for i, step in enumerate(plan.steps):
            self.get_logger().info(f'Executing step {i+1}/{len(plan.steps)}: {step.description}')

            success = self.execute_plan_step(step)
            if success:
                step.status = TaskStatus.COMPLETED
                step.execution_time = time.time()
            else:
                step.status = TaskStatus.FAILED
                step.error_message = "Step execution failed"
                plan.status = TaskStatus.FAILED
                break

        # Update plan status
        with self.active_plan_lock:
            if all(step.status == TaskStatus.COMPLETED for step in plan.steps):
                plan.status = TaskStatus.COMPLETED
                self.get_logger().info('Plan completed successfully')
            else:
                plan.status = TaskStatus.FAILED
                self.get_logger().error('Plan execution failed')

            plan.completed_time = time.time()

        # Publish plan status
        status_msg = String()
        status_msg.data = json.dumps({
            "plan_id": id(plan),
            "status": plan.status.value,
            "steps_completed": len([s for s in plan.steps if s.status == TaskStatus.COMPLETED]),
            "total_steps": len(plan.steps)
        })
        self.plan_status_pub.publish(status_msg)

    def execute_plan_step(self, step: PlanStep) -> bool:
        """
        Execute a single step of the plan.
        """
        try:
            if step.action_type == "navigation":
                return self.execute_navigation_step(step)
            elif step.action_type == "manipulation":
                return self.execute_manipulation_step(step)
            elif step.action_type == "perception":
                return self.execute_perception_step(step)
            else:
                self.get_logger().warn(f'Unknown action type: {step.action_type}')
                return False
        except Exception as e:
            self.get_logger().error(f'Error executing plan step: {e}')
            return False

    def execute_navigation_step(self, step: PlanStep) -> bool:
        """
        Execute a navigation step.
        """
        destination = step.parameters.get("destination", "")

        # Convert destination to coordinates (in a real system, this would come from a map)
        # For simulation, we'll use some predefined locations
        location_coords = {
            "kitchen": (3.0, 2.0, 0.0),
            "bedroom": (5.0, -1.0, 0.0),
            "office": (-2.0, 4.0, 0.0),
            "living room": (0.0, 0.0, 0.0),
            "front door": (8.0, 0.0, 0.0)
        }

        if destination.lower() in location_coords:
            x, y, theta = location_coords[destination.lower()]
            return self.send_navigation_goal(x, y, theta)
        else:
            self.get_logger().warn(f'Unknown destination: {destination}')
            return False

    def execute_manipulation_step(self, step: PlanStep) -> bool:
        """
        Execute a manipulation step.
        """
        obj = step.parameters.get("object", "")
        action = step.parameters.get("action", "")
        location = step.parameters.get("location", "")

        self.get_logger().info(f'Executing manipulation: {action} {obj} at {location}')
        # In a real system, this would interface with manipulation controllers
        # For now, we'll just log the action and return success
        return True

    def execute_perception_step(self, step: PlanStep) -> bool:
        """
        Execute a perception step.
        """
        target = step.parameters.get("target", "")
        attribute = step.parameters.get("attribute", "")

        self.get_logger().info(f'Executing perception: looking for {target} ({attribute if attribute else "all attributes"})')
        # In a real system, this would interface with perception systems
        # For now, we'll just log the action and return success
        return True

    def send_navigation_goal(self, x: float, y: float, theta: float = 0.0) -> bool:
        """
        Send a navigation goal to Nav2.
        """
        # Wait for the action server to be available
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return False

        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        cos_half = math.cos(theta / 2.0)
        sin_half = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = float(cos_half)
        goal_msg.pose.pose.orientation.z = float(sin_half)

        # Send the goal
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=None
        )

        # Wait for result
        import asyncio
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        async def wait_for_result():
            goal_handle = await self._send_goal_future
            if not goal_handle.accepted:
                self.get_logger().info('Navigation goal rejected')
                return False

            self.get_logger().info('Navigation goal accepted')
            result_future = goal_handle.get_result_async()
            result = await result_future
            status = result.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation succeeded')
                return True
            else:
                self.get_logger().info(f'Navigation failed with status: {status}')
                return False

        return loop.run_until_complete(wait_for_result())

    def extract_destination(self, natural_language: str, parameters: Dict[str, Any]) -> Optional[str]:
        """
        Extract destination from natural language command.
        """
        # Try to get from parameters first
        if "destination" in parameters:
            return parameters["destination"]
        if "location" in parameters:
            return parameters["location"]

        # Extract from natural language
        destinations = ["kitchen", "bedroom", "office", "living room", "bathroom", "front door", "back yard"]
        for dest in destinations:
            if dest in natural_language.lower():
                return dest

        # Look for location-specific keywords
        location_patterns = [
            r"go to the (\w+)",
            r"navigate to the (\w+)",
            r"move to the (\w+)",
            r"travel to the (\w+)"
        ]

        for pattern in location_patterns:
            match = re.search(pattern, natural_language.lower())
            if match:
                potential_dest = match.group(1)
                # Verify it's a known location
                if potential_dest in destinations:
                    return potential_dest

        return None

    def extract_object(self, natural_language: str) -> Optional[str]:
        """
        Extract object from natural language command.
        """
        # Common objects that robots might manipulate
        objects = [
            "cup", "bottle", "book", "phone", "keys", "apple", "banana",
            "ball", "toy", "box", "plate", "fork", "spoon", "knife"
        ]

        for obj in objects:
            if obj in natural_language.lower():
                return obj

        # Look for "the X" or "a X" patterns
        object_patterns = [
            r"(?:the|a|an)\s+(\w+)(?:\s+(?:on|in|at|to)\s+|\s+please|,|\.)",
            r"pick up(?:\s+the)?\s+(\w+)",
            r"grab(?:\s+the)?\s+(\w+)",
            r"take(?:\s+the)?\s+(\w+)"
        ]

        for pattern in object_patterns:
            match = re.search(pattern, natural_language.lower())
            if match:
                potential_obj = match.group(1)
                if potential_obj in objects:
                    return potential_obj

        return None

    def extract_action(self, natural_language: str) -> Optional[str]:
        """
        Extract action from natural language command.
        """
        actions = ["pick up", "grab", "take", "place", "put down", "drop", "lift", "hold", "move", "bring"]

        for action in actions:
            if action in natural_language.lower():
                return action

        return None

    def extract_location(self, natural_language: str) -> Optional[str]:
        """
        Extract location from natural language command.
        """
        # Similar to destination extraction
        return self.extract_destination(natural_language, {})

    def extract_attribute(self, natural_language: str) -> Optional[str]:
        """
        Extract attribute from natural language command.
        """
        attributes = ["color", "shape", "size", "weight", "temperature", "texture", "material"]

        for attr in attributes:
            if attr in natural_language.lower():
                return attr

        return None

    def planning_loop(self):
        """
        Main planning loop - currently just for status updates.
        """
        if self.current_plan:
            # Publish current plan status periodically
            status_msg = String()
            status_msg.data = json.dumps({
                "current_plan_status": self.current_plan.status.value,
                "steps_completed": len([s for s in self.current_plan.steps if s.status == TaskStatus.COMPLETED]),
                "total_steps": len(self.current_plan.steps),
                "confidence": self.current_plan.confidence
            })
            self.plan_status_pub.publish(status_msg)


def main(args=None):
    """
    Main function to run the cognitive planner node.
    """
    rclpy.init(args=args)

    cognitive_planner_node = CognitivePlannerNode()

    try:
        rclpy.spin(cognitive_planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        cognitive_planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import threading
    import math
    main()