// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 - Robotic Nervous System',
      items: ['module1/intro', 'module1/ros-basics', 'module1/nodes-topics-services', 'module1/urdf', 'module1/exercises'],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin - Gazebo & Unity',
      items: ['module2/intro', 'module2/gazebo-simulation', 'module2/unity-visualization', 'module2/sensors', 'module2/exercises'],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac AI-Robot Brain',
      items: ['module3/intro', 'module3/isaac-sim', 'module3/vslam', 'module3/navigation', 'module3/exercises'],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: ['module4/intro', 'module4/voice-processing', 'module4/cognitive-planning', 'module4/integration', 'module4/exercises'],
    },
    {
      type: 'category',
      label: 'Capstone: Autonomous Humanoid',
      items: ['capstone/intro', 'capstone/integration', 'capstone/assessment', 'capstone/exercises'],
    },
    'conclusion',
  ],
};

module.exports = sidebars;