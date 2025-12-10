import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai-in-motion/__docusaurus/debug',
    component: ComponentCreator('/ai-in-motion/__docusaurus/debug', '14c'),
    exact: true
  },
  {
    path: '/ai-in-motion/__docusaurus/debug/config',
    component: ComponentCreator('/ai-in-motion/__docusaurus/debug/config', '463'),
    exact: true
  },
  {
    path: '/ai-in-motion/__docusaurus/debug/content',
    component: ComponentCreator('/ai-in-motion/__docusaurus/debug/content', 'bb7'),
    exact: true
  },
  {
    path: '/ai-in-motion/__docusaurus/debug/globalData',
    component: ComponentCreator('/ai-in-motion/__docusaurus/debug/globalData', 'ee3'),
    exact: true
  },
  {
    path: '/ai-in-motion/__docusaurus/debug/metadata',
    component: ComponentCreator('/ai-in-motion/__docusaurus/debug/metadata', 'c1f'),
    exact: true
  },
  {
    path: '/ai-in-motion/__docusaurus/debug/registry',
    component: ComponentCreator('/ai-in-motion/__docusaurus/debug/registry', '591'),
    exact: true
  },
  {
    path: '/ai-in-motion/__docusaurus/debug/routes',
    component: ComponentCreator('/ai-in-motion/__docusaurus/debug/routes', '22f'),
    exact: true
  },
  {
    path: '/ai-in-motion/blog',
    component: ComponentCreator('/ai-in-motion/blog', '0d4'),
    exact: true
  },
  {
    path: '/ai-in-motion/blog/archive',
    component: ComponentCreator('/ai-in-motion/blog/archive', '220'),
    exact: true
  },
  {
    path: '/ai-in-motion/blog/first-post',
    component: ComponentCreator('/ai-in-motion/blog/first-post', '09d'),
    exact: true
  },
  {
    path: '/ai-in-motion/docs',
    component: ComponentCreator('/ai-in-motion/docs', 'e92'),
    routes: [
      {
        path: '/ai-in-motion/docs/capstone/assessment',
        component: ComponentCreator('/ai-in-motion/docs/capstone/assessment', '069'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/capstone/exercises',
        component: ComponentCreator('/ai-in-motion/docs/capstone/exercises', 'c32'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/capstone/integration',
        component: ComponentCreator('/ai-in-motion/docs/capstone/integration', 'aa3'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/capstone/intro',
        component: ComponentCreator('/ai-in-motion/docs/capstone/intro', '2aa'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/conclusion',
        component: ComponentCreator('/ai-in-motion/docs/conclusion', '066'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/intro',
        component: ComponentCreator('/ai-in-motion/docs/intro', '8a6'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module1/exercises',
        component: ComponentCreator('/ai-in-motion/docs/module1/exercises', '773'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module1/intro',
        component: ComponentCreator('/ai-in-motion/docs/module1/intro', '197'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module1/nodes-topics-services',
        component: ComponentCreator('/ai-in-motion/docs/module1/nodes-topics-services', 'a0c'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module1/ros-basics',
        component: ComponentCreator('/ai-in-motion/docs/module1/ros-basics', '04a'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module1/urdf',
        component: ComponentCreator('/ai-in-motion/docs/module1/urdf', '65c'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module2/exercises',
        component: ComponentCreator('/ai-in-motion/docs/module2/exercises', '17a'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module2/gazebo-simulation',
        component: ComponentCreator('/ai-in-motion/docs/module2/gazebo-simulation', '501'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module2/integration',
        component: ComponentCreator('/ai-in-motion/docs/module2/integration', '2e8'),
        exact: true
      },
      {
        path: '/ai-in-motion/docs/module2/intro',
        component: ComponentCreator('/ai-in-motion/docs/module2/intro', 'b17'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module2/sensors',
        component: ComponentCreator('/ai-in-motion/docs/module2/sensors', '3e0'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module2/unity-visualization',
        component: ComponentCreator('/ai-in-motion/docs/module2/unity-visualization', 'be4'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module3/exercises',
        component: ComponentCreator('/ai-in-motion/docs/module3/exercises', '267'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module3/integration',
        component: ComponentCreator('/ai-in-motion/docs/module3/integration', '076'),
        exact: true
      },
      {
        path: '/ai-in-motion/docs/module3/intro',
        component: ComponentCreator('/ai-in-motion/docs/module3/intro', 'f3d'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module3/isaac-sim',
        component: ComponentCreator('/ai-in-motion/docs/module3/isaac-sim', '4fc'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module3/navigation',
        component: ComponentCreator('/ai-in-motion/docs/module3/navigation', 'db8'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module3/vslam',
        component: ComponentCreator('/ai-in-motion/docs/module3/vslam', '071'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module4/cognitive-planning',
        component: ComponentCreator('/ai-in-motion/docs/module4/cognitive-planning', '6e1'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module4/exercises',
        component: ComponentCreator('/ai-in-motion/docs/module4/exercises', '833'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module4/integration',
        component: ComponentCreator('/ai-in-motion/docs/module4/integration', '8b0'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module4/intro',
        component: ComponentCreator('/ai-in-motion/docs/module4/intro', '2aa'),
        exact: true,
        sidebar: "tutorialSidebar"
      },
      {
        path: '/ai-in-motion/docs/module4/voice-processing',
        component: ComponentCreator('/ai-in-motion/docs/module4/voice-processing', 'ce6'),
        exact: true,
        sidebar: "tutorialSidebar"
      }
    ]
  },
  {
    path: '/ai-in-motion/',
    component: ComponentCreator('/ai-in-motion/', '65f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
