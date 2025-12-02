
export type Chapter = {
  slug: string;
  title: string;
  contentPath: string;
};

export type Module = {
  slug: string;
  title: string;
  chapters: Chapter[];
};

export const bookData: Module[] = [
  {
    slug: 'module-1',
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    chapters: [
      {
        slug: 'introduction-to-physical-ai',
        title: 'Introduction to Physical AI',
        contentPath: 'module-1/introduction-to-physical-ai.md',
      },
      {
        slug: 'ros-2-fundamentals',
        title: 'ROS 2 Fundamentals',
        contentPath: 'module-1/ros-2-fundamentals.md',
      },
    ],
  },
  {
    slug: 'module-2',
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    chapters: [
      {
        slug: 'robot-simulation-with-gazebo',
        title: 'Robot Simulation with Gazebo',
        contentPath: 'module-2/robot-simulation-with-gazebo.md',
      },
    ],
  },
];
