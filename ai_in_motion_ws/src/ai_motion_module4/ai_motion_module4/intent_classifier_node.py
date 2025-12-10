"""
Intent Classifier Node

Classifies user intents from voice commands to determine appropriate
robot responses and actions for the Vision-Language-Action system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_motion_common_msgs.msg import VoiceCommand, IntentClassification
from typing import Dict, List, Tuple
import re
import json


class IntentClassifierNode(Node):
    """
    A ROS 2 node that classifies user intents from voice commands.
    """

    def __init__(self):
        super().__init__('intent_classifier_node')

        # Define intent patterns and keywords
        self.intent_patterns = {
            'navigation': [
                r'.*\b(go|move|navigate|go to|head to|walk to|travel to)\b.*',
                r'.*\b(room|location|area|kitchen|bedroom|office|living room|front door|charger)\b.*',
                r'.*\b(navigate|move|go)\b.*'
            ],
            'manipulation': [
                r'.*\b(pick up|grab|take|get|hold|lift|drop|put down|place|release)\b.*',
                r'.*\b(object|item|thing|cup|bottle|box|toy|book|phone)\b.*'
            ],
            'perception': [
                r'.*\b(see|find|look for|detect|identify|recognize|spot|locate)\b.*',
                r'.*\b(object|person|thing|item|cup|bottle|box|toy|book|phone|human|robot)\b.*'
            ],
            'report': [
                r'.*\b(tell me|report|status|where are you|what can you see|what is your status)\b.*',
                r'.*\b(position|location|status|report|information)\b.*'
            ],
            'wait': [
                r'.*\b(wait|stop|pause|hold|stay|remain)\b.*',
                r'.*\b(wait|stop|pause|for|until|while)\b.*'
            ]
        }

        # Initialize state
        self.confidence_threshold = 0.7

        # Create subscribers
        self.voice_cmd_sub = self.create_subscription(
            VoiceCommand,
            '/voice_command',
            self.voice_command_callback,
            10
        )

        # Create publishers
        self.intent_pub = self.create_publisher(
            IntentClassification,
            '/intent_classification',
            10
        )

        self.classified_cmd_pub = self.create_publisher(
            String,
            '/classified_command',
            10
        )

        self.get_logger().info('Intent Classifier Node initialized')

    def voice_command_callback(self, msg: VoiceCommand):
        """
        Callback for voice commands that performs intent classification.
        """
        self.get_logger().info(f'Classifying intent for: {msg.command}')

        intent, confidence = self.classify_intent(msg.command)

        # Create and publish intent classification
        classification_msg = IntentClassification()
        classification_msg.command = msg.command
        classification_msg.intent = intent
        classification_msg.confidence = confidence
        classification_msg.timestamp = self.get_clock().now().to_msg()

        self.intent_pub.publish(classification_msg)
        self.get_logger().info(f'Classified intent: {intent} (confidence: {confidence:.2f})')

        # If confidence is high enough, publish classified command
        if confidence >= self.confidence_threshold:
            classified_cmd = String()
            classified_cmd.data = json.dumps({
                'command': msg.command,
                'intent': intent,
                'confidence': confidence,
                'original_timestamp': msg.timestamp.sec + msg.timestamp.nanosec * 1e-9
            })
            self.classified_cmd_pub.publish(classified_cmd)

    def classify_intent(self, command: str) -> Tuple[str, float]:
        """
        Classify the intent of a voice command.
        Returns tuple of (intent_type, confidence_score).
        """
        command_lower = command.lower().strip()

        # Check for direct matches first
        direct_matches = self.check_direct_matches(command_lower)
        if direct_matches:
            return direct_matches[0], 1.0

        # Check pattern matches
        pattern_scores = {}
        for intent, patterns in self.intent_patterns.items():
            score = 0
            for pattern in patterns:
                if re.search(pattern, command_lower):
                    score += 1

            if score > 0:
                # Normalize score based on number of patterns for this intent
                pattern_scores[intent] = score / len(patterns)

        # Return the intent with highest confidence
        if pattern_scores:
            best_intent = max(pattern_scores, key=pattern_scores.get)
            best_confidence = pattern_scores[best_intent]

            # Apply confidence threshold
            if best_confidence >= self.confidence_threshold:
                return best_intent, best_confidence

        # Default to 'unknown' if no high-confidence match
        return 'unknown', 0.0

    def check_direct_matches(self, command: str) -> List[str]:
        """
        Check for direct keyword matches to quickly identify intents.
        """
        matches = []

        # Navigation keywords
        nav_keywords = ['kitchen', 'bedroom', 'office', 'living room', 'front door', 'charger',
                       'go to', 'navigate to', 'move to', 'head to', 'walk to']
        if any(keyword in command for keyword in nav_keywords):
            matches.append('navigation')

        # Manipulation keywords
        manip_keywords = ['pick up', 'grab', 'take', 'get', 'hold', 'lift', 'drop',
                         'put down', 'place', 'release']
        if any(keyword in command for keyword in manip_keywords):
            matches.append('manipulation')

        # Perception keywords
        percep_keywords = ['see', 'find', 'look for', 'detect', 'identify', 'recognize',
                          'spot', 'locate', 'what do you see', 'is there']
        if any(keyword in command for keyword in percep_keywords):
            matches.append('perception')

        # Report keywords
        report_keywords = ['where are you', 'what is your status', 'tell me', 'report',
                          'what can you see', 'status']
        if any(keyword in command for keyword in report_keywords):
            matches.append('report')

        # Wait keywords
        wait_keywords = ['wait', 'stop', 'pause', 'hold', 'stay', 'remain']
        if any(keyword in command for keyword in wait_keywords):
            matches.append('wait')

        return matches


def main(args=None):
    """
    Main function to run the intent classifier node.
    """
    rclpy.init(args=args)

    intent_classifier_node = IntentClassifierNode()

    try:
        rclpy.spin(intent_classifier_node)
    except KeyboardInterrupt:
        pass
    finally:
        intent_classifier_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()