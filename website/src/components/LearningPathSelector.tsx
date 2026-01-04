import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './LearningPathSelector.module.css';
import Link from '@docusaurus/Link';

type PathType = 'beginner' | 'intermediate' | 'advanced';

type LearningPath = {
  id: PathType;
  title: string;
  description: string;
  duration: string;
  prerequisites: string[];
  learningOutcomes: string[];
  weeklyCommitment: string;
  difficulty: string;
  color: string;
};

const learningPaths: LearningPath[] = [
  {
    id: 'beginner',
    title: 'Beginner Path: Foundations of Humanoid Robotics',
    description: 'Perfect for newcomers to robotics with minimal technical background. Covers fundamental concepts and basic implementations.',
    duration: '16-20 weeks',
    weeklyCommitment: '3-4 hours per week',
    difficulty: 'Beginner',
    color: '#4285f4',
    prerequisites: [
      'Basic programming concepts (any language)',
      'High school level mathematics',
      'Curiosity about robotics and AI'
    ],
    learningOutcomes: [
      'Understand fundamental robotics concepts and terminology',
      'Implement simple robot behaviors in simulation',
      'Navigate basic simulation environments',
      'Build confidence to pursue more advanced robotics topics'
    ]
  },
  {
    id: 'intermediate',
    title: 'Intermediate Path: Applied Humanoid Robotics',
    description: 'Designed for developers and engineers with some robotics experience. Focuses on practical implementation of humanoid robotics concepts.',
    duration: '12-16 weeks',
    weeklyCommitment: '4-5 hours per week',
    difficulty: 'Intermediate',
    color: '#ea4335',
    prerequisites: [
      'Programming experience (Python/C++)',
      'Basic understanding of linear algebra and calculus',
      'Some exposure to robotics concepts'
    ],
    learningOutcomes: [
      'Develop advanced robot applications using ROS 2',
      'Implement perception and control systems for humanoid robots',
      'Design complex robot behaviors and navigation systems',
      'Integrate multiple AI systems for comprehensive robot capabilities'
    ]
  },
  {
    id: 'advanced',
    title: 'Advanced Path: Humanoid AI Systems Development',
    description: 'For experienced robotics engineers and researchers. Focuses on research-level concepts and state-of-the-art techniques.',
    duration: '10-12 weeks',
    weeklyCommitment: '5-6 hours per week',
    difficulty: 'Advanced',
    color: '#fbbc04',
    prerequisites: [
      'Strong programming skills (Python/C++/CUDA)',
      'Advanced mathematics background',
      'Significant robotics experience',
      'Understanding of machine learning and deep learning'
    ],
    learningOutcomes: [
      'Implement state-of-the-art AI algorithms for humanoid robots',
      'Design and conduct robotics research projects',
      'Deploy advanced systems on real hardware',
      'Contribute to the robotics research community'
    ]
  }
];

const LearningPathSelector = (): JSX.Element => {
  const [selectedPath, setSelectedPath] = useState<PathType | null>(null);

  const handlePathSelect = (pathId: PathType) => {
    setSelectedPath(pathId);
    // Store selection in localStorage
    localStorage.setItem('selectedLearningPath', pathId);
  };

  const getPathLink = (pathId: PathType) => {
    switch (pathId) {
      case 'beginner':
        return '/docs/learning-paths/beginner-path';
      case 'intermediate':
        return '/docs/learning-paths/intermediate-path';
      case 'advanced':
        return '/docs/learning-paths/advanced-path';
      default:
        return '/docs/learning-paths/beginner-path';
    }
  };

  return (
    <div className={styles.pathSelectorContainer}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h1 className={styles.mainTitle}>Choose Your Learning Path</h1>
            <p className={styles.introText}>
              Select the learning path that best matches your experience level and goals. 
              Each path is designed to provide the right level of challenge and support for your robotics journey.
            </p>
            
            <div className={styles.pathsGrid}>
              {learningPaths.map((path) => (
                <div 
                  key={path.id} 
                  className={clsx(
                    styles.pathCard, 
                    selectedPath === path.id && styles.selected
                  )}
                  style={{ borderLeftColor: path.color }}
                  onClick={() => handlePathSelect(path.id)}
                >
                  <div className={styles.pathHeader}>
                    <h2 className={styles.pathTitle}>{path.title}</h2>
                    <span 
                      className={styles.difficultyBadge} 
                      style={{ backgroundColor: path.color }}
                    >
                      {path.difficulty}
                    </span>
                  </div>
                  
                  <p className={styles.pathDescription}>{path.description}</p>
                  
                  <div className={styles.pathDetails}>
                    <div className={styles.detailItem}>
                      <strong>Duration:</strong> {path.duration}
                    </div>
                    <div className={styles.detailItem}>
                      <strong>Time Commitment:</strong> {path.weeklyCommitment}
                    </div>
                  </div>
                  
                  <div className={styles.prerequisitesSection}>
                    <h3>Prerequisites</h3>
                    <ul>
                      {path.prerequisites.map((prereq, idx) => (
                        <li key={idx}>{prereq}</li>
                      ))}
                    </ul>
                  </div>
                  
                  <div className={styles.outcomesSection}>
                    <h3>Learning Outcomes</h3>
                    <ul>
                      {path.learningOutcomes.map((outcome, idx) => (
                        <li key={idx}>{outcome}</li>
                      ))}
                    </ul>
                  </div>
                  
                  <div className={styles.pathActions}>
                    <Link
                      className={clsx(
                        'button button--primary',
                        styles.startButton
                      )}
                      to={getPathLink(path.id)}
                      onClick={(e) => {
                        if (selectedPath !== path.id) {
                          e.preventDefault();
                          handlePathSelect(path.id);
                          setTimeout(() => {
                            window.location.href = getPathLink(path.id);
                          }, 300);
                        }
                      }}
                    >
                      Start {path.difficulty} Path
                    </Link>
                    
                    {selectedPath === path.id && (
                      <div className={styles.confirmationMessage}>
                        <p>✅ Your learning path has been selected!</p>
                        <p>You can change this at any time in your profile settings.</p>
                      </div>
                    )}
                  </div>
                </div>
              ))}
            </div>
            
            <div className={styles.additionalInfo}>
              <h3>Need Help Choosing?</h3>
              <p>
                Take our <Link to="/docs/learning-paths/assessment">quick assessment</Link> to determine the best path for your current skill level.
              </p>
              <p>
                Still unsure? Join our <Link to="/community">community forums</Link> to discuss with other learners and instructors.
              </p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default LearningPathSelector;