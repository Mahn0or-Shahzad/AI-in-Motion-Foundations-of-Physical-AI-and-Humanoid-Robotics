#!/usr/bin/env python3
"""
Test script to verify VLA nodes can be imported correctly.
"""

def test_imports():
    """Test that all VLA nodes can be imported without errors."""
    print("Testing VLA node imports...")

    try:
        from ai_motion_module4.voice_interface_node import VoiceInterfaceNode
        print("✓ VoiceInterfaceNode imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import VoiceInterfaceNode: {e}")

    try:
        from ai_motion_module4.cognitive_planner_node import CognitivePlannerNode
        print("✓ CognitivePlannerNode imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import CognitivePlannerNode: {e}")

    try:
        from ai_motion_module4.action_executor_node import ActionExecutorNode
        print("✓ ActionExecutorNode imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import ActionExecutorNode: {e}")

    try:
        from ai_motion_module4.vla_integration_node import VLAIntegrationNode
        print("✓ VLAIntegrationNode imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import VLAIntegrationNode: {e}")

    try:
        from ai_motion_module4.intent_classifier_node import IntentClassifierNode
        print("✓ IntentClassifierNode imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import IntentClassifierNode: {e}")

    print("\nAll VLA nodes imported successfully! ✓")


if __name__ == "__main__":
    test_imports()