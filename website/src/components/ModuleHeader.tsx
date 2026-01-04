import React from 'react';
import clsx from 'clsx';
import styles from './ModuleHeader.module.css';

type ModuleHeaderProps = {
  title: string;
  week: string;
  description: string;
};

const ModuleHeader = ({title, week, description}: ModuleHeaderProps): JSX.Element => {
  return (
    <div className={clsx('padding-horiz--md', styles.moduleHeader)}>
      <div className="container">
        <div className="row">
          <div className="col col--12 moduleHeaderContent">
            <h1 className={styles.moduleTitle}>{title}</h1>
            <div className={styles.moduleWeek}>{week}</div>
            <p className={styles.moduleDescription}>{description}</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ModuleHeader;