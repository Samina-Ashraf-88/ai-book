/**
 * Assessment Model
 * Represents an assessment/exercise in the Physical AI & Humanoid Robotics textbook
 */

class Assessment {
  constructor({
    id,
    moduleId,
    title,
    description,
    questions = [],
    maxScore = 100,
    timeLimit = null, // in seconds, null means no time limit
    isActive = true,
    createdAt,
    updatedAt
  }) {
    this.id = id;
    this.moduleId = moduleId;
    this.title = title;
    this.description = description;
    this.questions = questions; // Array of question objects
    this.maxScore = maxScore;
    this.timeLimit = timeLimit;
    this.isActive = isActive;
    this.createdAt = createdAt || new Date().toISOString();
    this.updatedAt = updatedAt || new Date().toISOString();
  }

  /**
   * Calculate the score for a given set of answers
   * @param {Array} userAnswers - Array of user answers
   * @returns {number} - Score between 0 and maxScore
   */
  calculateScore(userAnswers) {
    if (!Array.isArray(userAnswers) || userAnswers.length !== this.questions.length) {
      throw new Error('Invalid number of answers provided');
    }

    let correctAnswers = 0;
    let totalWeight = 0;

    for (let i = 0; i < this.questions.length; i++) {
      const question = this.questions[i];
      const userAnswer = userAnswers[i];

      if (this.isAnswerCorrect(question, userAnswer)) {
        correctAnswers += question.weight || 1;
      }
      totalWeight += question.weight || 1;
    }

    // Calculate percentage and scale to maxScore
    const percentage = totalWeight > 0 ? (correctAnswers / totalWeight) : 0;
    return Math.round(percentage * this.maxScore * 100) / 100; // Round to 2 decimal places
  }

  /**
   * Check if a user's answer is correct for a given question
   * @param {Object} question - Question object
   * @param {any} userAnswer - User's answer
   * @returns {boolean} - True if answer is correct
   */
  isAnswerCorrect(question, userAnswer) {
    switch (question.type) {
      case 'multiple-choice':
        return this.compareMultipleChoice(question, userAnswer);
      case 'true-false':
        return this.compareTrueFalse(question, userAnswer);
      case 'short-answer':
        return this.compareShortAnswer(question, userAnswer);
      case 'code':
        return this.compareCode(question, userAnswer);
      case 'matching':
        return this.compareMatching(question, userAnswer);
      case 'ordering':
        return this.compareOrdering(question, userAnswer);
      default:
        console.warn(`Unknown question type: ${question.type}`);
        return false;
    }
  }

  /**
   * Compare multiple choice answers
   */
  compareMultipleChoice(question, userAnswer) {
    if (Array.isArray(question.correctAnswer)) {
      // Multiple correct answers
      return Array.isArray(userAnswer) &&
             userAnswer.length === question.correctAnswer.length &&
             userAnswer.every(ans => question.correctAnswer.includes(ans));
    } else {
      // Single correct answer
      return userAnswer === question.correctAnswer;
    }
  }

  /**
   * Compare true/false answers
   */
  compareTrueFalse(question, userAnswer) {
    return userAnswer === question.correctAnswer;
  }

  /**
   * Compare short answer answers (with some tolerance for variations)
   */
  compareShortAnswer(question, userAnswer) {
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
  }

  /**
   * Compare code answers (simplified - in real implementation would need more sophisticated comparison)
   */
  compareCode(question, userAnswer) {
    // In a real implementation, this would involve running tests against the code
    // For now, we'll just check if the answer matches exactly or contains key elements
    if (typeof userAnswer !== 'string') return false;

    const userCode = userAnswer.trim();
    const correctCode = question.correctAnswer;

    if (typeof correctCode === 'string') {
      return userCode === correctCode;
    } else if (typeof correctCode === 'object' && correctCode.testFunction) {
      // SECURITY: Only allow predefined, safe test functions
      // In a real implementation, code evaluation should happen in a secure sandbox
      // For now, we'll use a safer approach that doesn't execute arbitrary code
      console.warn('Code evaluation detected - this should be done in a secure sandbox');
      // Instead of executing the function directly, return false to avoid security risk
      return false;
    }

    return false;
  }

  /**
   * Compare matching answers
   */
  compareMatching(question, userAnswer) {
    if (!Array.isArray(userAnswer) || !Array.isArray(question.correctAnswer)) return false;

    return question.correctAnswer.every((pair, index) => {
      return userAnswer[index] &&
             userAnswer[index].item === pair.item &&
             userAnswer[index].match === pair.match;
    });
  }

  /**
   * Compare ordering answers
   */
  compareOrdering(question, userAnswer) {
    if (!Array.isArray(userAnswer) || !Array.isArray(question.correctAnswer)) return false;

    return userAnswer.every((item, index) => item === question.correctAnswer[index]);
  }

  /**
   * Get the feedback for a user's answer to a specific question
   */
  getFeedback(questionIndex, userAnswer) {
    if (questionIndex < 0 || questionIndex >= this.questions.length) {
      throw new Error('Invalid question index');
    }

    const question = this.questions[questionIndex];
    const isCorrect = this.isAnswerCorrect(question, userAnswer);

    return {
      isCorrect,
      correctAnswer: question.correctAnswer,
      explanation: question.explanation || 'No explanation provided',
      pointsAwarded: isCorrect ? (question.weight || 1) : 0
    };
  }

  /**
   * Validate the assessment structure
   */
  validate() {
    const errors = [];

    if (!this.id) errors.push('ID is required');
    if (!this.moduleId) errors.push('Module ID is required');
    if (!this.title) errors.push('Title is required');
    if (!Array.isArray(this.questions) || this.questions.length === 0) {
      errors.push('At least one question is required');
    }

    // Validate each question
    this.questions.forEach((question, index) => {
      if (!question.type) errors.push(`Question ${index} must have a type`);
      if (!question.questionText) errors.push(`Question ${index} must have text`);
      if (question.correctAnswer === undefined) errors.push(`Question ${index} must have a correct answer`);
    });

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  /**
   * Get a simplified version of the assessment for student view (without answers)
   */
  getForStudent() {
    return {
      id: this.id,
      moduleId: this.moduleId,
      title: this.title,
      description: this.description,
      questions: this.questions.map(q => ({
        id: q.id,
        type: q.type,
        questionText: q.questionText,
        options: q.options, // Only options for multiple choice, etc.
        required: q.required || false
      })),
      maxScore: this.maxScore,
      timeLimit: this.timeLimit,
      isActive: this.isActive
    };
  }
}

module.exports = Assessment;