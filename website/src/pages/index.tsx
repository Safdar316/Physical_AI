import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.robotAnimation}>
            <div className={styles.robotContainer}>
              <div className={styles.robotHead}>
                <div className={styles.robotEye}></div>
                <div className={styles.robotEye}></div>
                <div className={styles.robotMouth}></div>
              </div>
              <div className={styles.robotBody}>
                <div className={styles.robotArm}></div>
                <div className={styles.robotArm}></div>
              </div>
              <div className={styles.robotLegs}>
                <div className={styles.robotLeg}></div>
                <div className={styles.robotLeg}></div>
              </div>
            </div>
          </div>
          <div className={styles.textContent}>
            <Heading as="h1" className={clsx('hero__title', styles.animatedTitle)}>
              {siteConfig.title}
            </Heading>
            <p className={clsx('hero__subtitle', styles.subtitle)}>
              {siteConfig.tagline}
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Start Learning - 13 Weeks to Mastery 🚀
              </Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical Humanoid AI Robotics Course`}
      description="Comprehensive course on Physical Humanoid AI Robotics - Learn to build intelligent humanoid robots with AI, ROS 2, and NVIDIA Isaac">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
