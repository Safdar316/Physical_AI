import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './RobotPhysicsVisualizations.module.css';

type Visualization = {
  id: string;
  title: string;
  description: string;
  type: 'balance' | 'kinematics' | 'dynamics' | 'manipulation' | 'locomotion';
  imageUrl: string;
  caption: string;
  physicsConcept: string;
};

type RobotPhysicsVisualizationsProps = {
  visualizations: Visualization[];
};

const visualizationTypes = {
  balance: { title: 'Balance & Stability', icon: '⚖️', color: '#4285f4' },
  kinematics: { title: 'Kinematics', icon: '🔄', color: '#ea4335' },
  dynamics: { title: 'Dynamics', icon: '⚡', color: '#fbbc04' },
  manipulation: { title: 'Manipulation', icon: '✋', color: '#34a853' },
  locomotion: { title: 'Locomotion', icon: '🏃', color: '#aa00ff' },
};

const RobotPhysicsVisualizations = ({ visualizations }: RobotPhysicsVisualizationsProps): JSX.Element => {
  const [activeVisualization, setActiveVisualization] = useState<string | null>(null);

  const handleVisualizationClick = (id: string) => {
    setActiveVisualization(activeVisualization === id ? null : id);
  };

  return (
    <div className={clsx('margin-vert--md', styles.visualizationsContainer)}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h3 className={styles.header}>Robot Physics Visualizations</h3>
            <p className={styles.description}>
              Interactive visualizations showing robots interacting with fundamental physical laws
            </p>
            
            <div className={styles.visualizationsGrid}>
              {visualizations.map((viz) => (
                <div 
                  key={viz.id} 
                  className={clsx(
                    styles.visualizationCard, 
                    activeVisualization === viz.id && styles.active
                  )}
                  onClick={() => handleVisualizationClick(viz.id)}
                >
                  <div 
                    className={styles.visualizationHeader}
                    style={{ backgroundColor: visualizationTypes[viz.type].color }}
                  >
                    <span className={styles.visualizationIcon}>
                      {visualizationTypes[viz.type].icon}
                    </span>
                    <h4 className={styles.visualizationTitle}>
                      {viz.title}
                    </h4>
                  </div>
                  
                  <div className={styles.visualizationImage}>
                    <img 
                      src={viz.imageUrl} 
                      alt={viz.title}
                      className={styles.image}
                    />
                  </div>
                  
                  <div className={styles.visualizationContent}>
                    <div className={styles.physicsConcept}>
                      <strong>Physics Concept:</strong> {viz.physicsConcept}
                    </div>
                    <p className={styles.description}>{viz.description}</p>
                    <p className={styles.caption}>{viz.caption}</p>
                  </div>
                  
                  {activeVisualization === viz.id && (
                    <div className={styles.detailedExplanation}>
                      <h5>Detailed Physics Explanation</h5>
                      <p>
                        This visualization demonstrates how robots interact with fundamental physical laws. 
                        {viz.type === 'balance' && ' Balance is maintained by keeping the center of mass within the support polygon.'}
                        {viz.type === 'kinematics' && ' Kinematics describes the motion of robot links without considering forces.'}
                        {viz.type === 'dynamics' && ' Dynamics considers the forces that cause robot motion and interaction.'}
                        {viz.type === 'manipulation' && ' Manipulation involves applying forces to objects while maintaining stability.'}
                        {viz.type === 'locomotion' && ' Locomotion requires coordinated control of multiple joints to achieve movement.'}
                      </p>
                      <p>
                        The visualization shows the interplay between the robot's control system and physical laws, 
                        demonstrating how Physical AI principles enable robots to operate effectively in the real world.
                      </p>
                    </div>
                  )}
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default RobotPhysicsVisualizations;