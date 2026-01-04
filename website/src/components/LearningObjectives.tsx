import React from 'react';
import clsx from 'clsx';
import styles from './LearningObjectives.module.css';

type Objective = {
  id: string;
  text: string;
  type: 'knowledge' | 'skill' | 'application';
};

type LearningObjectivesProps = {
  objectives: Objective[];
};

const objectiveTypeLabels = {
  knowledge: 'Knowledge Objective',
  skill: 'Skill Objective',
  application: 'Application Objective'
};

const objectiveTypeIcons = {
  knowledge: '🧠',
  skill: '🛠️',
  application: '🎯'
};

const LearningObjectives = ({ objectives }: LearningObjectivesProps): JSX.Element => {
  return (
    <div className={clsx('margin-vert--md', styles.learningObjectivesContainer)}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h3 className={styles.header}>Learning Objectives</h3>
            <div className={styles.objectivesList}>
              {objectives.map((objective) => (
                <div key={objective.id} className={styles.objectiveItem}>
                  <div className={styles.objectiveIcon}>
                    {objectiveTypeIcons[objective.type]}
                  </div>
                  <div className={styles.objectiveContent}>
                    <div className={styles.objectiveType}>
                      {objectiveTypeLabels[objective.type]}
                    </div>
                    <div className={styles.objectiveText}>
                      {objective.text}
                    </div>
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

export default LearningObjectives;