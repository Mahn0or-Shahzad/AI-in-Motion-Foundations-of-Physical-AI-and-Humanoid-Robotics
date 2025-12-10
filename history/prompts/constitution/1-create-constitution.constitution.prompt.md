---
id: 1
title: Create Constitution
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: FRIENDX COMPUTERS
command: sp.constitution
labels: [constitution,spec,ai-motion,physical-ai,humanoid-robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
---

## Prompt

sp.constitution : # Spec-Kit Plus Constitution — AI in Motion: Foundations of Physical AI and Humanoid Robotics

**ProjectName:** AI in Motion — Foundations of Physical AI and Humanoid Robotics
**Purpose:** Define global rules for content creation, lab setup, AI/robotic simulations, hardware requirements, and workflow for Physical AI course.

---

# 1 — General Principles
- Content must be beginner-friendly and clear.
- Chapters/modules structured as: Introduction → Concept → Example → Practical Exercise → Summary.
- Headings and subheadings must be consistent.
- Tone must be educational and neutral.

---

# 2 — Course Modules & AI/Spec Rules
- **Module 1: ROS 2 — Robotic Nervous System**
  - Nodes, Topics, Services.
  - Python integration with ROS 2 using rclpy.
  - URDF for humanoid description.
- **Module 2: Digital Twin (Gazebo & Unity)**
  - Physics simulation: gravity, collisions.
  - Sensor simulation: LiDAR, Depth Cameras, IMUs.
  - High-fidelity visualization in Unity.
- **Module 3: NVIDIA Isaac AI-Robot Brain**
  - Isaac Sim for photorealistic simulation.
  - Isaac ROS for VSLAM and navigation.
  - Nav2 for humanoid path planning.
- **Module 4: Vision-Language-Action (VLA)**
  - Voice-to-Action using OpenAI Whisper.
  - Cognitive Planning: natural language → ROS 2 actions.
- **Capstone: Autonomous Humanoid**
  - Voice command → plan → navigation → perception → manipulation.

---

# 3 — Simulation & Deployment Rules
- Use ROS 2 Humble/Iron, Gazebo, Unity, NVIDIA Isaac Sim.
- Chapters stored as .mdx or Markdown files.
- Sim-to-Real workflow: train in simulation → deploy on Jetson/physical kit.
- RAG Chatbot integration for voice-command interpretation.
- All AI models must be versioned and saved for reproducibility.

---

# 4 — Lab & Hardware Rules
- **Digital Twin Workstation:**
  - GPU: NVIDIA RTX 4070 Ti (12GB) minimum.
  - CPU: Intel i7 (13th Gen) or AMD Ryzen 9.
  - RAM: 64GB DDR5 (32GB minimum).
  - OS: Ubuntu 22.04 LTS.
- **Physical AI Edge Kit:**
  - Jetson Orin Nano (8GB) or Orin NX (16GB).
  - Intel RealSense D435i/D455 camera for RGB-D perception.
  - USB IMU (BNO055) for balance.
  - USB microphone/speaker array for Whisper integration.
- **Robot Lab:**
  - Proxy robot: Unitree Go2 Edu for budget.
  - Miniature humanoids: Hiwonder TonyPi Pro for basic kinematics.
  - Premium: Unitree G1 for full sim-to-real humanoid deployment.
- **Cloud Lab Option:**
  - AWS/Azure GPU cloud for Isaac Sim if local RTX unavailable.
  - Edge kit (Jetson + sensors) mandatory for physical deployment.

---

# 5 — Personalization & Automation
- User customization: theme, font size, language toggle (English/Urdu).
- AI automatically suggests content improvements.
- Content history automatically saved in SP workflow.

---

# 6 — Assessments & Projects
- ROS 2 package development project.
- Gazebo simulation implementation.
- Isaac-based perception pipeline.
- Capstone: Simulated humanoid robot with conversational AI.

---

# 7 — Global SP Workflow
- All updates follow SP workflow.
- Human review mandatory before final deployment.
- Changes logged with timestamp and responsible user.
- Execute all prompts sequentially using SP workflow.
- All content, simulations, and hardware instructions are linked to modules for consistency.

## Response snapshot

I'll help you create a constitution for your AI in Motion project. Let me first check if there's an existing constitution file and then create or update it based on your specifications.

[File search and read operations were performed, followed by updating the constitution file with the provided content, organizing it into appropriate sections with proper formatting.]

## Outcome

- ✅ Impact: Created comprehensive constitution for AI in Motion project
- 🧪 Tests: No tests required for constitution file creation
- 📁 Files: Updated .specify/memory/constitution.md with structured content
- 🔁 Next prompts: Module specification and planning can follow
- 🧠 Reflection: Constitution now properly defines project principles, modules, hardware requirements, and workflow

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A