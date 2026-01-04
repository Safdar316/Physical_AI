import React, {useState} from 'react';
import clsx from 'clsx';
import styles from './Assessment.module.css';

type Question = {
  id: string;
  text: string;
  type: 'multiple-choice' | 'true-false' | 'short-answer';
  options?: string[];
  correctAnswer: string | number;
};

type AssessmentProps = {
  title: string;
  description: string;
  questions: Question[];
  onSubmit?: (answers: Record<string, any>) => void;
};

const Assessment = ({title, description, questions, onSubmit}: AssessmentProps): JSX.Element => {
  const [answers, setAnswers] = useState<Record<string, any>>({});
  const [submitted, setSubmitted] = useState(false);
  const [score, setScore] = useState<number | null>(null);

  const handleAnswerChange = (questionId: string, value: any) => {
    setAnswers(prev => ({
      ...prev,
      [questionId]: value
    }));
  };

  const handleSubmit = () => {
    if (!onSubmit) return;

    // Calculate score
    let correctCount = 0;
    questions.forEach(question => {
      if (answers[question.id] == question.correctAnswer) {
        correctCount++;
      }
    });

    const calculatedScore = Math.round((correctCount / questions.length) * 100);
    setScore(calculatedScore);
    setSubmitted(true);
    
    onSubmit(answers);
  };

  const resetAssessment = () => {
    setAnswers({});
    setSubmitted(false);
    setScore(null);
  };

  return (
    <div className={clsx('margin-vert--md', styles.assessmentContainer)}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <div className={styles.assessmentHeader}>
              <h2>{title}</h2>
              <p>{description}</p>
            </div>

            {score !== null && (
              <div className={styles.scoreDisplay}>
                <h3>Your Score: {score}%</h3>
                <p>({score >= 70 ? 'Pass' : 'Needs Improvement'})</p>
              </div>
            )}

            <div className={styles.questionsContainer}>
              {questions.map((question, index) => (
                <div key={question.id} className={styles.question}>
                  <h4>{index + 1}. {question.text}</h4>
                  
                  {question.type === 'multiple-choice' && (
                    <div className={styles.options}>
                      {question.options?.map((option, optIndex) => (
                        <label key={optIndex} className={styles.optionLabel}>
                          <input
                            type="radio"
                            name={question.id}
                            value={optIndex}
                            checked={answers[question.id] == optIndex}
                            onChange={() => handleAnswerChange(question.id, optIndex)}
                            disabled={submitted}
                          />
                          <span className={styles.optionText}>{option}</span>
                        </label>
                      ))}
                    </div>
                  )}

                  {question.type === 'true-false' && (
                    <div className={styles.options}>
                      <label className={styles.optionLabel}>
                        <input
                          type="radio"
                          name={question.id}
                          value="true"
                          checked={answers[question.id] === 'true'}
                          onChange={() => handleAnswerChange(question.id, 'true')}
                          disabled={submitted}
                        />
                        <span className={styles.optionText}>True</span>
                      </label>
                      <label className={styles.optionLabel}>
                        <input
                          type="radio"
                          name={question.id}
                          value="false"
                          checked={answers[question.id] === 'false'}
                          onChange={() => handleAnswerChange(question.id, 'false')}
                          disabled={submitted}
                        />
                        <span className={styles.optionText}>False</span>
                      </label>
                    </div>
                  )}

                  {question.type === 'short-answer' && (
                    <textarea
                      className={styles.shortAnswer}
                      value={answers[question.id] || ''}
                      onChange={(e) => handleAnswerChange(question.id, e.target.value)}
                      disabled={submitted}
                      rows={4}
                    />
                  )}
                </div>
              ))}
            </div>

            {!submitted ? (
              <button 
                className={clsx('button button--primary', styles.submitButton)}
                onClick={handleSubmit}
              >
                Submit Assessment
              </button>
            ) : (
              <button 
                className={clsx('button button--secondary', styles.resetButton)}
                onClick={resetAssessment}
              >
                Retake Assessment
              </button>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};

export default Assessment;