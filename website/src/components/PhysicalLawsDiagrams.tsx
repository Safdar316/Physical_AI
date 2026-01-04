import React from 'react';
import clsx from 'clsx';
import styles from './PhysicalLawsDiagrams.module.css';

type Diagram = {
  id: string;
  title: string;
  description: string;
  type: 'newton' | 'kinematics' | 'dynamics' | 'balance' | 'friction';
  imageUrl: string;
  caption: string;
};

type PhysicalLawsDiagramsProps = {
  diagrams: Diagram[];
};

const diagramTypes = {
  newton: { title: 'Newton\'s Laws', icon: '⚖️' },
  kinematics: { title: 'Kinematics', icon: '🔄' },
  dynamics: { title: 'Dynamics', icon: '⚡' },
  balance: { title: 'Balance & Stability', icon: '⚖️' },
  friction: { title: 'Friction & Forces', icon: '🧱' },
};

const PhysicalLawsDiagrams = ({ diagrams }: PhysicalLawsDiagramsProps): JSX.Element => {
  return (
    <div className={clsx('margin-vert--md', styles.diagramsContainer)}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h3 className={styles.header}>Interactive Diagrams: Physical Laws in Robotics</h3>
            <div className={styles.diagramsGrid}>
              {diagrams.map((diagram) => (
                <div key={diagram.id} className={styles.diagramCard}>
                  <div className={styles.diagramHeader}>
                    <span className={styles.diagramIcon}>
                      {diagramTypes[diagram.type].icon}
                    </span>
                    <h4 className={styles.diagramTitle}>
                      {diagram.title}
                    </h4>
                  </div>
                  <div className={styles.diagramImage}>
                    <img 
                      src={diagram.imageUrl} 
                      alt={diagram.title}
                      className={styles.image}
                    />
                  </div>
                  <div className={styles.diagramContent}>
                    <p className={styles.description}>{diagram.description}</p>
                    <p className={styles.caption}>{diagram.caption}</p>
                  </div>
                </div>
              ))}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default PhysicalLawsDiagrams;