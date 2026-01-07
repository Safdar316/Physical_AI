import React, { useState, useEffect, JSX } from 'react';
import clsx from 'clsx';
import styles from './ProgressTracker.module.css';

type ModuleProgress = {
  id: string;
  title: string;
  completed: boolean;
  lastAccessed?: Date;
};

type ProgressTrackerProps = {
  courseId: string;
  moduleName: string;
  moduleId: string;
};

const ProgressTracker = ({ courseId, moduleName, moduleId }: ProgressTrackerProps): JSX.Element => {
  const [progress, setProgress] = useState<ModuleProgress[]>([]);
  const [loading, setLoading] = useState(true);

  // Load progress from localStorage
  useEffect(() => {
    const loadProgress = () => {
      try {
        const savedProgress = localStorage.getItem(`course-progress-${courseId}`);
        if (savedProgress) {
          const parsedProgress = JSON.parse(savedProgress);
          setProgress(parsedProgress);
        } else {
          // Initialize with default progress
          const defaultProgress: ModuleProgress[] = [
            { id: 'intro', title: 'Introduction', completed: false },
            { id: 'module-template', title: 'Module Template', completed: false },
            { id: 'week-01-physical-ai', title: 'Week 1: Introduction to Physical AI', completed: false },
            { id: 'week-02-physical-ai', title: 'Week 2: Physical AI and Embodied Intelligence', completed: false },
            { id: 'week-03-ros2', title: 'Week 3: ROS 2 Architecture and Core Concepts', completed: false },
            { id: 'week-04-ros2', title: 'Week 4: ROS 2 Packages and Nodes', completed: false },
            { id: 'week-05-ros2', title: 'Week 5: ROS 2 Communication Patterns', completed: false },
            { id: 'week-06-simulation', title: 'Week 6: Gazebo Simulation Environment Setup', completed: false },
            { id: 'week-07-simulation', title: 'Week 7: Unity for Robot Visualization', completed: false },
            { id: 'week-08-nvidia-isaac', title: 'Week 8: NVIDIA Isaac SDK Introduction', completed: false },
            { id: 'week-09-nvidia-isaac', title: 'Week 9: Isaac AI Perception Systems', completed: false },
            { id: 'week-10-nvidia-isaac', title: 'Week 10: Isaac Sim and Reinforcement Learning', completed: false },
            { id: 'week-11-humanoid-robotics', title: 'Week 11: Humanoid Robot Kinematics', completed: false },
            { id: 'week-12-humanoid-robotics', title: 'Week 12: Humanoid Robot Dynamics and Interaction Design', completed: false },
            { id: 'week-13-conversational-ai', title: 'Week 13: Conversational AI for Robotics', completed: false },
          ];
          setProgress(defaultProgress);
          localStorage.setItem(`course-progress-${courseId}`, JSON.stringify(defaultProgress));
        }
      } catch (error) {
        console.error('Error loading progress:', error);
        // Initialize with default progress if there's an error
        const defaultProgress: ModuleProgress[] = [
          { id: 'intro', title: 'Introduction', completed: false },
          { id: 'module-template', title: 'Module Template', completed: false },
          { id: 'week-01-physical-ai', title: 'Week 1: Introduction to Physical AI', completed: false },
          { id: 'week-02-physical-ai', title: 'Week 2: Physical AI and Embodied Intelligence', completed: false },
          { id: 'week-03-ros2', title: 'Week 3: ROS 2 Architecture and Core Concepts', completed: false },
          { id: 'week-04-ros2', title: 'Week 4: ROS 2 Packages and Nodes', completed: false },
          { id: 'week-05-ros2', title: 'Week 5: ROS 2 Communication Patterns', completed: false },
          { id: 'week-06-simulation', title: 'Week 6: Gazebo Simulation Environment Setup', completed: false },
          { id: 'week-07-simulation', title: 'Week 7: Unity for Robot Visualization', completed: false },
          { id: 'week-08-nvidia-isaac', title: 'Week 8: NVIDIA Isaac SDK Introduction', completed: false },
          { id: 'week-09-nvidia-isaac', title: 'Week 9: Isaac AI Perception Systems', completed: false },
          { id: 'week-10-nvidia-isaac', title: 'Week 10: Isaac Sim and Reinforcement Learning', completed: false },
          { id: 'week-11-humanoid-robotics', title: 'Week 11: Humanoid Robot Kinematics', completed: false },
          { id: 'week-12-humanoid-robotics', title: 'Week 12: Humanoid Robot Dynamics and Interaction Design', completed: false },
          { id: 'week-13-conversational-ai', title: 'Week 13: Conversational AI for Robotics', completed: false },
        ];
        setProgress(defaultProgress);
        localStorage.setItem(`course-progress-${courseId}`, JSON.stringify(defaultProgress));
      } finally {
        setLoading(false);
      }
    };

    loadProgress();
  }, [courseId]);

  // Mark current module as accessed
  useEffect(() => {
    if (!loading && moduleId) {
      const updateCurrentModule = () => {
        setProgress(prevProgress => {
          const updatedProgress = prevProgress.map(module => 
            module.id === moduleId 
              ? { ...module, lastAccessed: new Date() } 
              : module
          );
          
          localStorage.setItem(`course-progress-${courseId}`, JSON.stringify(updatedProgress));
          return updatedProgress;
        });
      };

      updateCurrentModule();
    }
  }, [loading, courseId, moduleId]);

  const markModuleComplete = (moduleId: string) => {
    setProgress(prevProgress => {
      const updatedProgress = prevProgress.map(module => 
        module.id === moduleId 
          ? { ...module, completed: true, lastAccessed: new Date() } 
          : module
      );
      
      localStorage.setItem(`course-progress-${courseId}`, JSON.stringify(updatedProgress));
      return updatedProgress;
    });
  };

  if (loading) {
    return <div className={styles.progressTracker}>Loading progress...</div>;
  }

  const completedCount = progress.filter(m => m.completed).length;
  const totalCount = progress.length;
  const completionPercentage = Math.round((completedCount / totalCount) * 100);

  return (
    <div className={clsx('margin-vert--md', styles.progressTracker)}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h3>Course Progress</h3>
            <div className={styles.progressBarContainer}>
              <div 
                className={styles.progressBar} 
                style={{ width: `${completionPercentage}%` }}
              >
                <span className={styles.progressText}>{completionPercentage}% Complete</span>
              </div>
            </div>
            
            <div className={styles.progressDetails}>
              <p>{completedCount} of {totalCount} modules completed</p>
              
              <div className={styles.moduleList}>
                {progress.map((module) => (
                  <div 
                    key={module.id} 
                    className={clsx(
                      styles.moduleItem, 
                      module.id === moduleId && styles.currentModule
                    )}
                  >
                    <span className={styles.moduleTitle}>{module.title}</span>
                    <span className={styles.moduleStatus}>
                      {module.completed ? (
                        <span className={styles.completed}>✓ Completed</span>
                      ) : module.id === moduleId ? (
                        <span className={styles.current}>Currently viewing</span>
                      ) : (
                        <button 
                          className={clsx('button button--sm', styles.markCompleteButton)}
                          onClick={() => markModuleComplete(module.id)}
                        >
                          Mark Complete
                        </button>
                      )}
                    </span>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ProgressTracker;