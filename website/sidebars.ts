import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro', 'module-template', 'course-summary'],
    },
    {
      type: 'category',
      label: 'Week 1: Introduction to Physical AI',
      items: [
        'week-01-physical-ai/introduction',
        'week-01-physical-ai/embodied-intelligence',
        'week-01-physical-ai/physical-laws',
        'week-01-physical-ai/assessment',
      ],
    },
    {
      type: 'category',
      label: 'Week 2: Physical AI and Embodied Intelligence',
      items: [
        'week-02-physical-ai/advanced-concepts',
        'week-02-physical-ai/embodied-intelligence',
        'week-02-physical-ai/applications',
        'week-02-physical-ai/case-studies',
        'week-02-physical-ai/resources',
        'week-02-physical-ai/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Week 3: ROS 2 Architecture and Core Concepts',
      items: [
        'week-03-ros2/ros2-architecture',
      ],
    },
    {
      type: 'category',
      label: 'Week 4: ROS 2 Packages and Nodes',
      items: [
        'week-04-ros2/working-with-nodes',
      ],
    },
    {
      type: 'category',
      label: 'Week 5: ROS 2 Communication Patterns',
      items: [
        'week-05-ros2/communication-examples',
        'week-05-ros2/templates/project-templates',
      ],
    },
    {
      type: 'category',
      label: 'Week 6: Gazebo Simulation Environment Setup',
      items: [
        'week-06-simulation/gazebo-setup',
      ],
    },
    {
      type: 'category',
      label: 'Week 7: Unity for Robot Visualization',
      items: [
        'week-07-simulation/unity-visualization',
      ],
    },
    {
      type: 'category',
      label: 'Week 8: NVIDIA Isaac SDK Introduction',
      items: [
        'week-08-nvidia-isaac/isaac-overview',
      ],
    },
    {
      type: 'category',
      label: 'Week 9: Isaac AI Perception Systems',
      items: [
        'week-09-nvidia-isaac/vision-processing',
      ],
    },
    {
      type: 'category',
      label: 'Week 10: Isaac Sim and Reinforcement Learning',
      items: [
        'week-10-nvidia-isaac/isaac-sim',
      ],
    },
    {
      type: 'category',
      label: 'Week 11: Humanoid Robot Kinematics',
      items: [
        'week-11-humanoid-robotics/kinematics-basics',
      ],
    },
    {
      type: 'category',
      label: 'Week 12: Humanoid Robot Dynamics and Interaction Design',
      items: [
        'week-12-humanoid-robotics/dynamics',
      ],
    },
    {
      type: 'category',
      label: 'Week 13: Conversational AI for Robotics',
      items: [
        'week-13-conversational-ai/gpt-integration',
      ],
    },
    {
      type: 'category',
      label: 'Assessments and Projects',
      items: [
        'week-03-ros2/ros2-architecture',
        'week-06-simulation/gazebo-setup',
        'week-08-nvidia-isaac/isaac-overview',
        'week-10-nvidia-isaac/isaac-sim',
      ],
    },
  ],
};

export default sidebars;
