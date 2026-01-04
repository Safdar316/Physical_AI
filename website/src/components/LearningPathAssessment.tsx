import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './LearningPathAssessment.module.css';

type Question = {
  id: number;
  text: string;
  options: {
    id: string;
    text: string;
    scores: {
      beginner: number;
      intermediate: number;
      advanced: number;
    };
  }[];
};

const questions: Question[] = [
  {
    id: 1,
    text: "What is your programming experience?",
    options: [
      {
        id: "a",
        text: "Little to no programming experience",
        scores: { beginner: 3, intermediate: 1, advanced: 0 }
      },
      {
        id: "b", 
        text: "Some programming experience (any language)",
        scores: { beginner: 2, intermediate: 2, advanced: 1 }
      },
      {
        id: "c",
        text: "Strong programming experience (Python/C++/Java)",
        scores: { beginner: 0, intermediate: 2, advanced: 3 }
      },
      {
        id: "d",
        text: "Expert programming skills with multiple languages",
        scores: { beginner: 0, intermediate: 1, advanced: 3 }
      }
    ]
  },
  {
    id: 2,
    text: "What is your mathematics background?",
    options: [
      {
        id: "a",
        text: "Basic math (algebra, geometry)",
        scores: { beginner: 3, intermediate: 1, advanced: 0 }
      },
      {
        id: "b",
        text: "Calculus and linear algebra",
        scores: { beginner: 1, intermediate: 2, advanced: 2 }
      },
      {
        id: "c", 
        text: "Advanced mathematics (multivariable calculus, differential equations)",
        scores: { beginner: 0, intermediate: 1, advanced: 3 }
      },
      {
        id: "d",
        text: "Mathematics degree or equivalent",
        scores: { beginner: 0, intermediate: 0, advanced: 3 }
      }
    ]
  },
  {
    id: 3,
    text: "How much robotics experience do you have?",
    options: [
      {
        id: "a",
        text: "None - completely new to robotics",
        scores: { beginner: 3, intermediate: 0, advanced: 0 }
      },
      {
        id: "b",
        text: "Some exposure to robotics concepts",
        scores: { beginner: 2, intermediate: 1, advanced: 0 }
      },
      {
        id: "c",
        text: "Hands-on experience with robots (any platform)",
        scores: { beginner: 0, intermediate: 2, advanced: 1 }
      },
      {
        id: "d",
        text: "Significant experience with robotic systems",
        scores: { beginner: 0, intermediate: 1, advanced: 3 }
      }
    ]
  },
  {
    id: 4,
    text: "What is your primary goal?",
    options: [
      {
        id: "a",
        text: "Learn about robotics for personal interest",
        scores: { beginner: 3, intermediate: 1, advanced: 0 }
      },
      {
        id: "b",
        text: "Develop practical robotics skills for career development",
        scores: { beginner: 1, intermediate: 3, advanced: 1 }
      },
      {
        id: "c",
        text: "Conduct robotics research or development",
        scores: { beginner: 0, intermediate: 1, advanced: 3 }
      },
      {
        id: "d",
        text: "Lead robotics projects or teams",
        scores: { beginner: 0, intermediate: 1, advanced: 3 }
      }
    ]
  },
  {
    id: 5,
    text: "How much time can you dedicate to learning per week?",
    options: [
      {
        id: "a",
        text: "Less than 2 hours",
        scores: { beginner: 2, intermediate: 1, advanced: 0 }
      },
      {
        id: "b",
        text: "2-4 hours",
        scores: { beginner: 3, intermediate: 2, advanced: 1 }
      },
      {
        id: "c",
        text: "4-6 hours",
        scores: { beginner: 1, intermediate: 3, advanced: 2 }
      },
      {
        id: "d",
        text: "More than 6 hours",
        scores: { beginner: 0, intermediate: 1, advanced: 3 }
      }
    ]
  }
];

const LearningPathAssessment = (): JSX.Element => {
  const [answers, setAnswers] = useState<Record<number, string>>({});
  const [result, setResult] = useState<{path: string, confidence: number, explanation: string} | null>(null);
  const [showResult, setShowResult] = useState(false);

  const handleAnswerSelect = (questionId: number, optionId: string) => {
    setAnswers(prev => ({
      ...prev,
      [questionId]: optionId
    }));
    
    // Reset result when answering
    setResult(null);
    setShowResult(false);
  };

  const calculateResult = () => {
    // Initialize scores
    let scores = {
      beginner: 0,
      intermediate: 0,
      advanced: 0
    };

    // Calculate scores based on answers
    questions.forEach(question => {
      const selectedOptionId = answers[question.id];
      if (selectedOptionId) {
        const selectedOption = question.options.find(opt => opt.id === selectedOptionId);
        if (selectedOption) {
          scores.beginner += selectedOption.scores.beginner;
          scores.intermediate += selectedOption.scores.intermediate;
          scores.advanced += selectedOption.scores.advanced;
        }
      }
    });

    // Determine the path with highest score
    let recommendedPath = 'beginner';
    let maxScore = scores.beginner;
    
    if (scores.intermediate > maxScore) {
      recommendedPath = 'intermediate';
      maxScore = scores.intermediate;
    }
    
    if (scores.advanced > maxScore) {
      recommendedPath = 'advanced';
      maxScore = scores.advanced;
    }

    // Calculate confidence (difference between highest and second highest scores)
    const sortedScores = Object.entries(scores).sort((a, b) => b[1] - a[1]);
    const firstScore = sortedScores[0][1];
    const secondScore = sortedScores[1][1];
    const confidence = Math.round(((firstScore - secondScore) / questions.length) * 100);

    // Generate explanation
    let explanation = '';
    if (recommendedPath === 'beginner') {
      explanation = 'Based on your responses, the beginner path is recommended. This path starts with fundamental concepts and gradually builds up to more complex topics. It provides a solid foundation for your robotics journey.';
    } else if (recommendedPath === 'intermediate') {
      explanation = 'Based on your responses, the intermediate path is recommended. This path focuses on practical implementation of robotics concepts and is suitable for those with some programming and robotics experience.';
    } else {
      explanation = 'Based on your responses, the advanced path is recommended. This path is designed for experienced practitioners who want to work on research-level robotics problems and advanced AI implementations.';
    }

    setResult({
      path: recommendedPath,
      confidence,
      explanation
    });
    setShowResult(true);
  };

  const resetAssessment = () => {
    setAnswers({});
    setResult(null);
    setShowResult(false);
  };

  const getPathDetails = (path: string) => {
    switch (path) {
      case 'beginner':
        return {
          title: 'Beginner Path: Foundations of Humanoid Robotics',
          description: 'Perfect for newcomers to robotics with minimal technical background. Covers fundamental concepts and basic implementations.',
          link: '/docs/learning-paths/beginner-path'
        };
      case 'intermediate':
        return {
          title: 'Intermediate Path: Applied Humanoid Robotics',
          description: 'Designed for developers and engineers with some robotics experience. Focuses on practical implementation of humanoid robotics concepts.',
          link: '/docs/learning-paths/intermediate-path'
        };
      case 'advanced':
        return {
          title: 'Advanced Path: Humanoid AI Systems Development',
          description: 'For experienced robotics engineers and researchers. Focuses on research-level concepts and state-of-the-art techniques.',
          link: '/docs/learning-paths/advanced-path'
        };
      default:
        return {
          title: 'Beginner Path: Foundations of Humanoid Robotics',
          description: 'Perfect for newcomers to robotics with minimal technical background. Covers fundamental concepts and basic implementations.',
          link: '/docs/learning-paths/beginner-path'
        };
    }
  };

  return (
    <div className={styles.assessmentContainer}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            {!showResult ? (
              <div className={styles.assessmentForm}>
                <h2>Assessment Questions</h2>
                <p>Please answer the following questions to help determine the best learning path for you:</p>
                
                {questions.map((question) => (
                  <div key={question.id} className={styles.question}>
                    <h3>{question.id}. {question.text}</h3>
                    <div className={styles.options}>
                      {question.options.map((option) => (
                        <label 
                          key={option.id} 
                          className={clsx(
                            styles.optionLabel,
                            answers[question.id] === option.id && styles.selected
                          )}
                        >
                          <input
                            type="radio"
                            name={`question-${question.id}`}
                            value={option.id}
                            checked={answers[question.id] === option.id}
                            onChange={() => handleAnswerSelect(question.id, option.id)}
                            className={styles.optionInput}
                          />
                          <span className={styles.optionText}>{option.text}</span>
                        </label>
                      ))}
                    </div>
                  </div>
                ))}
                
                <div className={styles.assessmentActions}>
                  <button 
                    className={clsx('button button--primary button--lg', styles.submitButton)}
                    onClick={calculateResult}
                    disabled={Object.keys(answers).length < questions.length}
                  >
                    Get My Recommended Path
                  </button>
                  
                  {Object.keys(answers).length > 0 && (
                    <button 
                      className={clsx('button button--secondary button--lg', styles.resetButton)}
                      onClick={resetAssessment}
                    >
                      Reset Assessment
                    </button>
                  )}
                </div>
              </div>
            ) : (
              <div className={styles.resultsContainer}>
                <h2>Assessment Results</h2>
                
                {result && (
                  <div className={styles.resultCard}>
                    <div className={styles.resultHeader}>
                      <h3>Your Recommended Learning Path</h3>
                      <div className={styles.confidenceBadge}>
                        Confidence: {result.confidence}%
                      </div>
                    </div>
                    
                    <div className={styles.recommendedPath}>
                      <h4>{getPathDetails(result.path).title}</h4>
                      <p>{getPathDetails(result.path).description}</p>
                    </div>
                    
                    <div className={styles.explanation}>
                      <h5>Why this path was recommended:</h5>
                      <p>{result.explanation}</p>
                    </div>
                    
                    <div className={styles.nextSteps}>
                      <h5>Next Steps:</h5>
                      <ol>
                        <li>Review the curriculum for your recommended path</li>
                        <li>Set up your development environment</li>
                        <li>Begin with the first module in your learning path</li>
                        <li>Join our community to connect with other learners</li>
                      </ol>
                    </div>
                    
                    <div className={styles.resultActions}>
                      <a 
                        href={getPathDetails(result.path).link}
                        className={clsx('button button--primary button--lg', styles.startPathButton)}
                      >
                        Start {result.path.charAt(0).toUpperCase() + result.path.slice(1)} Path
                      </a>
                      
                      <button 
                        className={clsx('button button--secondary button--lg', styles.tryAgainButton)}
                        onClick={resetAssessment}
                      >
                        Retake Assessment
                      </button>
                    </div>
                  </div>
                )}
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default LearningPathAssessment;