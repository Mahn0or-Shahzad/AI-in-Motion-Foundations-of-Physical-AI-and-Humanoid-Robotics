# AI in Motion: Foundations of Physical AI and Humanoid Robotics

Welcome to AI in Motion, an educational curriculum focused on building AI-powered humanoid robots. This repository contains the complete curriculum covering ROS 2 fundamentals, digital twin simulation, Isaac AI integration, and Vision-Language-Action systems.

## Curriculum Status: COMPLETE ✅

All four modules have been fully implemented with working code, documentation, and integration. The complete system enables humanoid robots to:
- Process natural language voice commands using OpenAI Whisper
- Generate cognitive plans using LLMs for complex tasks
- Execute navigation, manipulation, and perception actions
- Operate safely in dynamic environments with real-time obstacle detection
- Transfer seamlessly between simulation and real-world deployment

## Project Structure

```
ai_in_motion_ws/
├── src/                    # ROS 2 source packages
│   ├── ai_motion_common_msgs/  # Common message definitions
│   ├── ai_motion_module1/      # Module 1: ROS 2 Fundamentals
│   ├── ai_motion_module2/      # Module 2: Digital Twin & Simulation
│   ├── ai_motion_module3/      # Module 3: Isaac AI & Navigation
│   └── ai_motion_module4/      # Module 4: Vision-Language-Action (VLA)
```

## System Architecture

The complete AI in Motion system integrates all modules into a unified framework:

```
[Voice Commands] -> [VLA System] -> [Cognitive Planner] -> [Action Executor] -> [Robot Actions]
                        |                    |                    |              |
                        v                    v                    v              v
              [Voice Interface] -> [Robot State] -> [Navigation System] -> [Real/Sim Robot]
```

## Modules Overview

### Module 1: ROS 2 — Robotic Nervous System
- Complete ROS 2 node architecture with topics, services, and actions
- Humanoid robot URDF model with joint control
- Robot state management and sensor integration
- Joint state publisher and control interfaces

### Module 2: Digital Twin & Simulation
- Gazebo simulation environment with physics and sensors
- LiDAR, camera, and IMU simulation with realistic parameters
- Unity visualization for enhanced human-robot interaction
- Hardware abstraction layer for sim-to-real transfer

### Module 3: Isaac AI & Navigation
- NVIDIA Isaac Sim integration with VSLAM capabilities
- Isaac ROS bridge for advanced perception and mapping
- Nav2 navigation stack with bipedal-specific configurations
- Obstacle course environments and gait planning

### Module 4: Vision-Language-Action (VLA) System
- Voice processing with OpenAI Whisper API integration
- LLM-based cognitive planning for natural language understanding
- Action execution for navigation, manipulation, and perception
- Intent classification and safety-aware operation

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- NVIDIA Isaac Sim
- Gazebo Garden
- Unity 2022.3 LTS
- Python 3.10+
- OpenAI API key for Whisper

## Getting Started

1. Install ROS 2 Humble Hawksbill
2. Set up NVIDIA Isaac Sim (for Module 3)
3. Configure OpenAI API keys (for Module 4)
4. Build all packages: `colcon build`
5. Launch the complete system: `ros2 launch ai_motion_module4 vla_system.launch.py`

For detailed setup instructions, see the individual module READMEs in each package directory.

## Launching the Complete System

To run the complete AI in Motion system:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd ai_in_motion_ws

# Build the workspace
colcon build

# Source the built packages
source install/setup.bash

# Launch the complete VLA system
ros2 launch ai_motion_module4 vla_system.launch.py
```

## Capstone Project: Autonomous Humanoid

The curriculum culminates in a capstone project where students integrate all modules to create an autonomous humanoid robot capable of:
- Processing natural language voice commands
- Generating cognitive plans for complex tasks
- Executing navigation, manipulation, and perception actions
- Operating safely in dynamic environments
- Demonstrating sim-to-real transfer capabilities

## Contributing

Please read the [CONTRIBUTING.md](CONTRIBUTING.md) file for details on our code of conduct and the process for submitting pull requests.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- ROS 2 community for the robotics framework
- NVIDIA Isaac team for simulation tools
- OpenAI for Whisper API