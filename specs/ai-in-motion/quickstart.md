# Quickstart Guide for AI in Motion

## Prerequisites

### Hardware Requirements
- **Digital Twin Workstation** (recommended):
  - GPU: NVIDIA RTX 4070 Ti (12GB) minimum
  - CPU: Intel i7 (13th Gen) or AMD Ryzen 9
  - RAM: 64GB DDR5 (32GB minimum)
  - OS: Ubuntu 22.04 LTS

### Software Requirements
- ROS 2 Humble Hawksbill
- NVIDIA Isaac Sim
- Gazebo Garden
- Unity 2022.3 LTS
- Python 3.10+
- OpenAI API key (for Whisper)
- Node.js and npm (for Docusaurus documentation site)

## Installation

### 1. Install ROS 2 Humble
```bash
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2
source /opt/ros/humble/setup.bash
```

### 2. Install Python Dependencies
```bash
pip3 install openai rospy numpy transforms3d opencv-python
```

### 3. Install Node.js and Docusaurus for Documentation
```bash
# Install Node.js (if not already installed)
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt-get install -y nodejs

# Install Docusaurus
npm install @docusaurus/core@latest @docusaurus/preset-classic@latest
```

### 4. Set Up Workspace
```bash
mkdir -p ~/ai_in_motion_ws/src
cd ~/ai_in_motion_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Running the Simulation

### 1. Launch the Base System
```bash
cd ~/ai_in_motion_ws
source install/setup.bash
ros2 launch ai_motion_bringup simulation.launch.py
```

### 2. In a New Terminal - Launch Voice Interface
```bash
cd ~/ai_in_motion_ws
source install/setup.bash
ros2 run ai_motion_vla voice_interface_node
```

### 3. Send a Voice Command
```bash
# Using ROS 2 CLI tools
ros2 service call /process_voice_command ai_motion_interfaces/srv/VoiceCommand "{
  audio_data: 'base64_encoded_audio_data',
  language: 'en'
}"
```

## Docusaurus Homepage Development

### 1. Development Setup for Homepage
1. Navigate to project directory: `cd ~/ai-in-motion` (documentation site)
2. Install dependencies if needed: `npm install` or `yarn install`
3. Start development server: `npm run start` or `yarn start`
4. Homepage will be available at http://localhost:3000

### 2. Homepage File Structure
```
src/
├── pages/
│   └── index.js (Main homepage component)
├── components/
│   ├── HomepageFeatures/
│   │   ├── index.js (Feature cards component)
│   │   └── styles.module.css (Feature cards styling)
│   └── TranslateButton/
│       ├── index.js (Translation button component)
│       └── styles.module.css (Translation button styling)
├── utils/
│   └── translate.js (Translation functions)
└── css/
    └── custom.css (Global homepage styles)
```

### 3. Homepage Components
- **Hero Section**: Main title "AI in Motion", subtitle "Foundations of Physical AI and Humanoid Robotics"
- **Feature Cards**: Physical AI Basics, Humanoid Robotics, Motion Intelligence with emoji icons
- **Buttons**: "Start Learning" (primary) and "Open Book" (outline) linking to curriculum
- **Translation**: Navbar button to toggle between English and Urdu

### 4. Building and Deployment for Documentation
1. Build the site: `npm run build`
2. The built site will be in the `build` directory
3. Deploy to GitHub Pages or your preferred hosting platform
4. For GitHub Pages deployment, run: `npm run deploy`

## Module-Specific Quickstarts

### Module 1: ROS 2 Basics
1. Launch the basic ROS 2 demo:
```bash
ros2 launch ai_motion_module1 demo.launch.py
```
2. View the robot state:
```bash
ros2 topic echo /robot_state
```

### Module 2: Digital Twin
1. Launch Gazebo simulation:
```bash
ros2 launch ai_motion_module2 gazebo.launch.py
```
2. Launch Unity visualization (separate process):
```bash
# Unity application needs to be started separately
# It will connect to ROS 2 bridge
```

### Module 3: Isaac AI
1. Launch Isaac Sim environment:
```bash
ros2 launch ai_motion_module3 isaac_sim.launch.py
```
2. Start VSLAM processing:
```bash
ros2 run ai_motion_module3 vslam_node
```

### Module 4: Vision-Language-Action
1. Start the voice processing pipeline:
```bash
ros2 run ai_motion_vla voice_processor_node
```
2. Start the cognitive planner:
```bash
ros2 run ai_motion_vla cognitive_planner_node
```

## Troubleshooting

### Common Issues

#### ROS 2 Not Found
- Ensure you've sourced the ROS 2 environment: `source /opt/ros/humble/setup.bash`

#### Isaac Sim Not Connecting
- Verify Isaac Sim is running and ROS bridge is configured
- Check that Isaac ROS packages are installed: `sudo apt install ros-humble-isaac-ros-*`

#### Audio Processing Errors
- Verify OpenAI API key is set in environment: `export OPENAI_API_KEY=your_key_here`
- Check audio format is supported (WAV, MP3, etc.)

#### Simulation Performance Issues
- Ensure GPU drivers are up to date
- Check that Isaac Sim has access to GPU resources
- Reduce simulation complexity if running on lower-end hardware

#### Docusaurus Homepage Issues
- Translation not working: Check that `translate.js` is properly loaded and DOM elements have text content
- Responsive design issues: Verify CSS media queries are properly implemented
- Build errors: Run `npm install` to ensure all dependencies are installed

## Development Workflow

### Creating New ROS 2 Packages
```bash
cd ~/ai_in_motion_ws/src
ros2 pkg create --dependency rclpy std_msgs sensor_msgs ai_motion_new_package
```

### Building the Workspace
```bash
cd ~/ai_in_motion_ws
colcon build --packages-select ai_motion_new_package
source install/setup.bash
```

### Running Tests
```bash
cd ~/ai_in_motion_ws
colcon test --packages-select ai_motion_new_package
colcon test-result --all
```

## Next Steps

1. Complete Module 1: ROS 2 basics
2. Explore Module 2: Digital twin simulation
3. Implement Module 3: AI integration
4. Develop Module 4: Voice-language-action pipeline
5. Integrate all modules in the capstone project
6. Customize and deploy the Docusaurus homepage for curriculum access