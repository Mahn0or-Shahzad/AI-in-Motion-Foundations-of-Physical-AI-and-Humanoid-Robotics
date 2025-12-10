import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Physical AI Basics',
    icon: '🧠',
    description: (
      <>
        Introduction to embodied intelligence fundamentals.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    icon: '🤖',
    description: (
      <>
        Learn structure and control of humanoid robots.
      </>
    ),
  },
  {
    title: 'Motion Intelligence',
    icon: '⚡',
    description: (
      <>
        Explore movement, balance, and real-world physics.
      </>
    ),
  },
];

function Feature({icon, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <span className={styles.featureIcon}>{icon}</span>
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}