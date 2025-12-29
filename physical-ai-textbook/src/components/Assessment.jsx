import React, { useState, useEffect } from 'react';
import './Assessment.css';

/**
 * Assessment Component
 * Provides an interface for students to take assessments and quizzes
 */
const Assessment = ({ assessment, onSubmit, onSave, userId, moduleId }) => {
  const [currentQuestionIndex, setCurrentQuestionIndex] = useState(0);
  const [userAnswers, setUserAnswers] = useState(
    Array(assessment.questions.length).fill(null).map(() => null)
  );
  const [timeRemaining, setTimeRemaining] = useState(assessment.timeLimit);
  const [isSubmitted, setIsSubmitted] = useState(false);
  const [showFeedback, setShowFeedback] = useState(false);
  const [assessmentScore, setAssessmentScore] = useState(null);
  const [isTimerActive, setIsTimerActive] = useState(assessment.timeLimit > 0);

  // Timer effect
  useEffect(() => {
    let timer = null;

    if (isTimerActive && timeRemaining > 0) {
      timer = setTimeout(() => {
        setTimeRemaining(prev => prev - 1);
      }, 1000);
    } else if (isTimerActive && timeRemaining === 0) {
      // Time's up - auto-submit
      handleSubmit();
    }

    return () => {
      if (timer) clearTimeout(timer);
    };
  }, [timeRemaining, isTimerActive]);

  // Format time for display
  const formatTime = (seconds) => {
    if (seconds === null) return '--:--';
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`;
  };

  // Handle answer change for different question types
  const handleAnswerChange = (questionIndex, answer) => {
    const newAnswers = [...userAnswers];
    newAnswers[questionIndex] = answer;
    setUserAnswers(newAnswers);

    // Auto-save progress
    if (onSave) {
      onSave({
        userId,
        moduleId,
        assessmentId: assessment.id,
        answers: newAnswers
      });
    }
  };

  // Handle multiple choice answer
  const handleMultipleChoice = (questionIndex, option) => {
    handleAnswerChange(questionIndex, option);
  };

  // Handle true/false answer
  const handleTrueFalse = (questionIndex, value) => {
    handleAnswerChange(questionIndex, value);
  };

  // Handle text answer
  const handleTextAnswer = (questionIndex, value) => {
    handleAnswerChange(questionIndex, value);
  };

  // Handle matching answer
  const handleMatching = (questionIndex, item, match) => {
    const currentAnswer = userAnswers[questionIndex] || [];
    const newAnswer = [...currentAnswer];

    const existingIndex = newAnswer.findIndex(a => a.item === item);
    if (existingIndex >= 0) {
      newAnswer[existingIndex].match = match;
    } else {
      newAnswer.push({ item, match });
    }

    handleAnswerChange(questionIndex, newAnswer);
  };

  // Handle ordering answer
  const handleOrdering = (questionIndex, orderedItems) => {
    handleAnswerChange(questionIndex, orderedItems);
  };

  // Handle code answer
  const handleCodeAnswer = (questionIndex, code) => {
    handleAnswerChange(questionIndex, code);
  };

  // Navigate to next question
  const goToNextQuestion = () => {
    if (currentQuestionIndex < assessment.questions.length - 1) {
      setCurrentQuestionIndex(currentQuestionIndex + 1);
    }
  };

  // Navigate to previous question
  const goToPreviousQuestion = () => {
    if (currentQuestionIndex > 0) {
      setCurrentQuestionIndex(currentQuestionIndex - 1);
    }
  };

  // Submit the assessment
  const handleSubmit = () => {
    setIsSubmitted(true);
    setIsTimerActive(false);

    // Calculate score using the assessment model
    let score = 0;
    try {
      // Create an Assessment instance to calculate the score properly
      // For now, we'll implement the calculation directly based on our Assessment model logic
      let correctAnswers = 0;
      let totalWeight = 0;

      for (let i = 0; i < assessment.questions.length; i++) {
        const question = assessment.questions[i];
        const userAnswer = userAnswers[i];

        // Implement the same logic as in the Assessment model
        if (isAnswerCorrect(question, userAnswer)) {
          correctAnswers += question.weight || 1;
        }
        totalWeight += question.weight || 1;
      }

      // Calculate percentage and scale to maxScore
      const percentage = totalWeight > 0 ? (correctAnswers / totalWeight) : 0;
      score = Math.round(percentage * (assessment.maxScore || 100) * 100) / 100; // Round to 2 decimal places
    } catch (e) {
      // Fallback scoring if calculation fails
      let answered = userAnswers.filter(answer => answer !== null).length;
      score = Math.round((answered / assessment.questions.length) * 100);
    }

    setAssessmentScore(score);
    setShowFeedback(true);

    // Call the submit callback with results
    if (onSubmit) {
      onSubmit({
        assessmentId: assessment.id,
        userId,
        moduleId,
        answers: userAnswers,
        score,
        timeTaken: assessment.timeLimit - timeRemaining
      });
    }
  };

  // Helper function to check if an answer is correct
  const isAnswerCorrect = (question, userAnswer) => {
    if (question.correctAnswer === undefined) return false;

    switch (question.type) {
      case 'multiple-choice':
        if (Array.isArray(question.correctAnswer)) {
          // Multiple correct answers
          if (!Array.isArray(userAnswer) || userAnswer.length !== question.correctAnswer.length) {
            return false;
          }
          return userAnswer.every(ans => question.correctAnswer.includes(ans));
        } else {
          // Single correct answer
          return userAnswer === question.correctAnswer;
        }

      case 'true-false':
        return userAnswer === question.correctAnswer;

      case 'short-answer':
        if (typeof userAnswer !== 'string') return false;
        const userAns = userAnswer.trim().toLowerCase();
        const correctAns = question.correctAnswer;

        if (Array.isArray(correctAns)) {
          return correctAns.some(ans =>
            ans.trim().toLowerCase() === userAns ||
            userAns.includes(ans.trim().toLowerCase())
          );
        }

        return userAns === correctAns.trim().toLowerCase() ||
               userAns.includes(correctAns.trim().toLowerCase());

      case 'matching':
        if (!Array.isArray(userAnswer) || !Array.isArray(question.correctAnswer)) return false;
        return question.correctAnswer.every((pair, index) => {
          return userAnswer[index] &&
                 userAnswer[index].item === pair.item &&
                 userAnswer[index].match === pair.match;
        });

      case 'ordering':
        if (!Array.isArray(userAnswer) || !Array.isArray(question.correctAnswer)) return false;
        return userAnswer.every((item, index) => item === question.correctAnswer[index]);

      case 'code':
        // For code questions, in a real implementation, this would involve running tests against the code
        // For now, we'll just check if the answer matches exactly
        if (typeof userAnswer !== 'string') return false;
        const userCode = userAnswer.trim();
        const correctCode = question.correctAnswer;
        if (typeof correctCode === 'string') {
          return userCode === correctCode;
        }
        return false;

      default:
        return false;
    }
  };

  // Reset the assessment
  const handleReset = () => {
    setUserAnswers(Array(assessment.questions.length).fill(null).map(() => null));
    setCurrentQuestionIndex(0);
    setTimeRemaining(assessment.timeLimit);
    setIsSubmitted(false);
    setShowFeedback(false);
    setAssessmentScore(null);
    setIsTimerActive(assessment.timeLimit > 0);
  };

  // Get current question
  const currentQuestion = assessment.questions[currentQuestionIndex];

  // Render question based on type
  const renderQuestion = () => {
    switch (currentQuestion.type) {
      case 'multiple-choice':
        return (
          <div className="question-options">
            {currentQuestion.options?.map((option, idx) => (
              <div key={idx} className="option-item">
                <input
                  type="radio"
                  id={`q${currentQuestionIndex}-opt${idx}`}
                  name={`question-${currentQuestionIndex}`}
                  checked={userAnswers[currentQuestionIndex] === option}
                  onChange={() => handleMultipleChoice(currentQuestionIndex, option)}
                  disabled={isSubmitted}
                />
                <label htmlFor={`q${currentQuestionIndex}-opt${idx}`}>
                  {option}
                </label>
              </div>
            ))}
          </div>
        );

      case 'true-false':
        return (
          <div className="question-options">
            <div className="option-item">
              <input
                type="radio"
                id={`q${currentQuestionIndex}-true`}
                name={`question-${currentQuestionIndex}`}
                checked={userAnswers[currentQuestionIndex] === true}
                onChange={() => handleTrueFalse(currentQuestionIndex, true)}
                disabled={isSubmitted}
              />
              <label htmlFor={`q${currentQuestionIndex}-true`}>True</label>
            </div>
            <div className="option-item">
              <input
                type="radio"
                id={`q${currentQuestionIndex}-false`}
                name={`question-${currentQuestionIndex}`}
                checked={userAnswers[currentQuestionIndex] === false}
                onChange={() => handleTrueFalse(currentQuestionIndex, false)}
                disabled={isSubmitted}
              />
              <label htmlFor={`q${currentQuestionIndex}-false`}>False</label>
            </div>
          </div>
        );

      case 'short-answer':
        return (
          <div className="question-input">
            <textarea
              value={userAnswers[currentQuestionIndex] || ''}
              onChange={(e) => handleTextAnswer(currentQuestionIndex, e.target.value)}
              placeholder="Type your answer here..."
              disabled={isSubmitted}
              rows={4}
            />
          </div>
        );

      case 'matching':
        return (
          <div className="question-matching">
            {currentQuestion.items?.map((item, idx) => (
              <div key={idx} className="matching-pair">
                <span className="matching-item">{item}</span>
                <select
                  value={(userAnswers[currentQuestionIndex]?.find(a => a.item === item)?.match) || ''}
                  onChange={(e) => handleMatching(currentQuestionIndex, item, e.target.value)}
                  disabled={isSubmitted}
                >
                  <option value="">Select match...</option>
                  {currentQuestion.matches?.map((match, matchIdx) => (
                    <option key={matchIdx} value={match}>{match}</option>
                  ))}
                </select>
              </div>
            ))}
          </div>
        );

      case 'ordering':
        return (
          <div className="question-ordering">
            <p>Drag and drop to order the items correctly:</p>
            <div className="ordering-items">
              {currentQuestion.items?.map((item, idx) => (
                <div key={idx} className="ordering-item">
                  {item}
                </div>
              ))}
            </div>
          </div>
        );

      case 'code':
        return (
          <div className="question-code">
            <p>{currentQuestion.description}</p>
            <textarea
              value={userAnswers[currentQuestionIndex] || ''}
              onChange={(e) => handleCodeAnswer(currentQuestionIndex, e.target.value)}
              placeholder="Write your code here..."
              disabled={isSubmitted}
              rows={8}
              className="code-input"
            />
          </div>
        );

      default:
        return <p>Unknown question type</p>;
    }
  };

  // Calculate progress
  const progress = ((currentQuestionIndex + 1) / assessment.questions.length) * 100;

  return (
    <div className="assessment-container">
      <div className="assessment-header">
        <h2>{assessment.title}</h2>
        {assessment.timeLimit && (
          <div className={`time-display ${timeRemaining < 60 ? 'time-warning' : ''}`}>
            Time Remaining: {formatTime(timeRemaining)}
          </div>
        )}
        <div className="progress-bar">
          <div
            className="progress-fill"
            style={{ width: `${progress}%` }}
          ></div>
          <span className="progress-text">
            Question {currentQuestionIndex + 1} of {assessment.questions.length}
          </span>
        </div>
      </div>

      <div className="assessment-body">
        {showFeedback && assessmentScore !== null ? (
          <div className="assessment-results">
            <h3>Assessment Results</h3>
            <div className="score-display">
              Your Score: <strong>{assessmentScore}%</strong> ({assessmentScore}/100)
            </div>
            {assessmentScore >= 70 ? (
              <div className="result-message success">
                Congratulations! You passed this assessment.
              </div>
            ) : (
              <div className="result-message">
                You need to score at least 70% to pass. Consider reviewing the material.
              </div>
            )}
            <div className="result-actions">
              <button onClick={handleReset} className="btn btn-secondary">
                Retake Assessment
              </button>
            </div>
          </div>
        ) : (
          <>
            <div className="question-container">
              <h3>Question {currentQuestionIndex + 1}</h3>
              <p className="question-text">{currentQuestion.questionText}</p>
              {renderQuestion()}
            </div>

            <div className="assessment-navigation">
              <button
                onClick={goToPreviousQuestion}
                disabled={currentQuestionIndex === 0 || isSubmitted}
                className="btn btn-secondary"
              >
                Previous
              </button>

              {currentQuestionIndex < assessment.questions.length - 1 ? (
                <button
                  onClick={goToNextQuestion}
                  disabled={isSubmitted}
                  className="btn btn-primary"
                >
                  Next
                </button>
              ) : (
                <button
                  onClick={handleSubmit}
                  disabled={isSubmitted || userAnswers.some(a => a === null)}
                  className="btn btn-submit"
                >
                  Submit Assessment
                </button>
              )}
            </div>
          </>
        )}
      </div>
    </div>
  );
};

export default Assessment;