# Module 4: Vision-Language-Action (VLA) System

This module implements a complete Vision-Language-Action system for humanoid robots, enabling natural language interaction and autonomous task execution.

## Overview

The VLA system integrates three key components:
- **Voice Processing**: Speech-to-text conversion using OpenAI Whisper API
- **Cognitive Planning**: Natural language understanding and action planning using LLMs
- **Action Execution**: Execution of navigation, manipulation, and perception tasks

## Architecture

```
[User Speech] -> [Voice Interface] -> [Intent Classifier] -> [Cognitive Planner] -> [Action Executor] -> [Robot Actions]
```

### Components

#### 1. Voice Interface Node
- Captures audio from microphone or simulated source
- Processes speech-to-text using OpenAI Whisper API
- Publishes transcribed commands to the system

#### 2. Intent Classifier Node
- Analyzes voice commands to determine user intent
- Classifies commands into navigation, manipulation, perception, report, or wait actions
- Provides confidence scores for classifications

#### 3. Cognitive Planner Node
- Uses LLMs to interpret natural language commands
- Generates executable action plans with specific parameters
- Handles complex multi-step task decomposition

#### 4. Action Executor Node
- Executes navigation, manipulation, and perception actions
- Implements safety checks and emergency stops
- Manages action queuing and execution state

#### 5. VLA Integration Node
- Coordinates between all VLA components
- Manages system state and monitoring
- Provides unified interface to the rest of the system

## Messages and Services

### Custom Messages
- `VoiceCommand`: Contains transcribed speech commands
- `IntentClassification`: Represents classified user intent
- `CognitivePlan`: Contains planned action sequences
- `RobotState`: Current robot state information
- `SensorData`: Aggregated sensor information

## Configuration

The system is configured through `config/vla_config.yaml` which includes:

- Voice processing parameters (API keys, audio settings)
- Cognitive planning parameters (LLM settings, prompts)
- Action execution parameters (safety thresholds, timeouts)
- Named locations for navigation
- Intent classification settings

## Launch Files

Use the main launch file to start the complete VLA system:

```bash
ros2 launch ai_motion_module4 vla_system.launch.py
```

You can customize the launch with the following arguments:
- `use_sim_time`: Use simulation time (default: true)
- `enable_voice_processing`: Enable voice components (default: true)
- `enable_cognitive_planning`: Enable planning components (default: true)
- `enable_action_execution`: Enable execution components (default: true)
- `openai_api_key`: OpenAI API key for Whisper and LLM integration

## Usage Examples

### Basic Navigation
Voice command: "Go to the kitchen"
- Voice interface transcribes the command
- Intent classifier identifies navigation intent
- Cognitive planner generates navigation action
- Action executor moves robot to kitchen location

### Object Manipulation
Voice command: "Pick up the red cup"
- Voice interface transcribes the command
- Intent classifier identifies manipulation intent
- Cognitive planner generates perception and manipulation actions
- Action executor detects and picks up the object

### Status Query
Voice command: "Where are you?"
- Voice interface transcribes the command
- Intent classifier identifies report intent
- Cognitive planner generates status report action
- Action executor publishes robot position and state

## Safety Features

- Continuous LiDAR-based obstacle detection
- Emergency stop when safety thresholds are violated
- Timeout mechanisms for action execution
- Collision avoidance during navigation

## Dependencies

- OpenAI API (for Whisper and LLMs)
- ROS 2 (Humble or later)
- Python 3.8+
- Required Python packages: openai, transformers, torch, nltk, spacy

## Environment Variables

Set the following environment variables:
- `OPENAI_API_KEY`: Your OpenAI API key for Whisper and LLM access

## Topics

- `/voice_command` - Input for voice commands
- `/intent_classification` - Output of intent classification
- `/cognitive_plan` - Output of cognitive planning
- `/plan_execution_queue` - Input for action execution
- `/action_executor/status` - Action execution status
- `/vla_system/status` - Overall system status
- `/robot_state` - Current robot state
- `/sensor_data_aggregated` - Aggregated sensor data
- `/cmd_vel` - Velocity commands for robot movement

## Development

To run individual components for testing:

```bash
# Run voice interface only
ros2 run ai_motion_module4 voice_interface_node

# Run cognitive planner only
ros2 run ai_motion_module4 cognitive_planner_node

# Run action executor only
ros2 run ai_motion_module4 action_executor_node

# Run intent classifier only
ros2 run ai_motion_module4 intent_classifier_node

# Run VLA integration only
ros2 run ai_motion_module4 vla_integration_node
```