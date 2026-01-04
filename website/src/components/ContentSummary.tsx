import React from 'react';
import clsx from 'clsx';
import styles from './ContentSummary.module.css';

type KeyPoint = {
  id: string;
  title: string;
  description: string;
};

type ContentSummaryProps = {
  title: string;
  description: string;
  keyPoints: KeyPoint[];
  furtherReading?: string[];
};

const ContentSummary = ({ 
  title, 
  description, 
  keyPoints, 
  furtherReading 
}: ContentSummaryProps): JSX.Element => {
  return (
    <div className={clsx('margin-vert--md', styles.contentSummaryContainer)}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <div className={styles.summaryHeader}>
              <h3>{title}</h3>
              <p>{description}</p>
            </div>
            
            <div className={styles.keyPointsSection}>
              <h4>Key Points</h4>
              <div className={styles.keyPointsList}>
                {keyPoints.map((point) => (
                  <div key={point.id} className={styles.keyPointItem}>
                    <div className={styles.keyPointTitle}>{point.title}</div>
                    <div className={styles.keyPointDescription}>{point.description}</div>
                  </div>
                ))}
              </div>
            </div>
            
            {furtherReading && furtherReading.length > 0 && (
              <div className={styles.furtherReadingSection}>
                <h4>Further Reading</h4>
                <ul className={styles.furtherReadingList}>
                  {furtherReading.map((item, index) => (
                    <li key={index} className={styles.furtherReadingItem}>
                      {item}
                    </li>
                  ))}
                </ul>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default ContentSummary;