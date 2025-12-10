# AI in Motion — Foundations of Physical AI and Humanoid Robotics Constitution

**Project Name:** AI in Motion — Foundations of Physical AI and Humanoid Robotics
**Purpose:** Define global rules for content creation, lab setup, AI/robotic simulations, hardware requirements, and workflow for Physical AI course.

## Core Principles

### I. Educational Focus
Content must be beginner-friendly and clear. All materials follow a structured approach: Introduction → Concept → Example → Practical Exercise → Summary. The tone must be educational and neutral to ensure accessibility for all learners.

### II. Modularity and Structure
Course content is organized into distinct modules with clear learning objectives. Each module builds upon previous knowledge while maintaining independence for flexible learning paths. Modules follow a consistent structure to enhance comprehension.

### III. Simulation-to-Real Pipeline
All development follows a sim-to-real workflow: train in simulation → deploy on Jetson/physical kit. This approach ensures theoretical understanding translates to practical implementation while reducing hardware costs during development.

### IV. Technology Stack Consistency
Use standardized tools across the course: ROS 2 Humble/Iron, Gazebo, Unity, NVIDIA Isaac Sim, and Jetson platforms. This consistency ensures reproducible results and reduces environment setup complexity.

### V. Hardware Accessibility
Design for multiple hardware tiers: Digital Twin Workstation (high-performance), Physical AI Edge Kit (intermediate), and Robot Lab (advanced). This ensures accessibility for various budgets and learning objectives.

### VI. AI Integration
Integrate AI throughout the curriculum with focus on Vision-Language-Action (VLA) capabilities. All AI models must be versioned and saved for reproducibility, with RAG Chatbot integration for enhanced learning experiences.

## Course Modules & Technical Requirements

### Module 1: ROS 2 — Robotic Nervous System
- Nodes, Topics, Services
- Python integration with ROS 2 using rclpy
- URDF for humanoid description

### Module 2: Digital Twin (Gazebo & Unity)
- Physics simulation: gravity, collisions
- Sensor simulation: LiDAR, Depth Cameras, IMUs
- High-fidelity visualization in Unity

### Module 3: NVIDIA Isaac AI-Robot Brain
- Isaac Sim for photorealistic simulation
- Isaac ROS for VSLAM and navigation
- Nav2 for humanoid path planning

### Module 4: Vision-Language-Action (VLA)
- Voice-to-Action using OpenAI Whisper
- Cognitive Planning: natural language → ROS 2 actions

### Capstone: Autonomous Humanoid
- Voice command → plan → navigation → perception → manipulation

## Hardware & Lab Requirements

### Digital Twin Workstation
- GPU: NVIDIA RTX 4070 Ti (12GB) minimum
- CPU: Intel i7 (13th Gen) or AMD Ryzen 9
- RAM: 64GB DDR5 (32GB minimum)
- OS: Ubuntu 22.04 LTS

### Physical AI Edge Kit
- Jetson Orin Nano (8GB) or Orin NX (16GB)
- Intel RealSense D435i/D455 camera for RGB-D perception
- USB IMU (BNO055) for balance
- USB microphone/speaker array for Whisper integration

### Robot Lab
- Proxy robot: Unitree Go2 Edu for budget
- Miniature humanoids: Hiwonder TonyPi Pro for basic kinematics
- Premium: Unitree G1 for full sim-to-real humanoid deployment

### Cloud Lab Option
- AWS/Azure GPU cloud for Isaac Sim if local RTX unavailable
- Edge kit (Jetson + sensors) mandatory for physical deployment

## Development Workflow

### Content Creation
- Chapters stored as .mdx or Markdown files
- User customization: theme, font size, language toggle (English/Urdu)
- AI automatically suggests content improvements
- Content history automatically saved in SP workflow

### Quality Assurance
- All AI models must be versioned and saved for reproducibility
- RAG Chatbot integration for voice-command interpretation
- Human review mandatory before final deployment
- Changes logged with timestamp and responsible user

### Assessment & Projects
- ROS 2 package development project
- Gazebo simulation implementation
- Isaac-based perception pipeline
- Capstone: Simulated humanoid robot with conversational AI

## Governance

This constitution supersedes all other practices and guides all development activities within the AI in Motion project. All implementations must align with the educational objectives, hardware requirements, and technology stack defined herein. Amendments require documentation, approval, and migration planning to maintain consistency across all course materials.

All PRs/reviews must verify compliance with educational objectives, hardware requirements, and technology stack consistency. Use this constitution for development guidance and decision-making processes.

**Version**: 1.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09
