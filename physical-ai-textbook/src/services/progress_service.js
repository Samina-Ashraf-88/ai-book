/**
 * Progress Tracking Service
 * Handles the tracking and management of user progress through the textbook
 */

const ProgressRecord = require('../models/progress');
const Assessment = require('../models/assessment');

class ProgressService {
  constructor() {
    // In a real implementation, this would connect to a database
    // For now, we'll use an in-memory store
    this.progressRecords = new Map();
    this.assessmentResults = new Map();
  }

  /**
   * Initialize progress for a user in a module
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID
   * @returns {ProgressRecord} - New progress record
   */
  async initializeProgress(userId, moduleId) {
    const progressId = `${userId}-${moduleId}-${Date.now()}`;
    const progressRecord = new ProgressRecord({
      id: progressId,
      userId,
      moduleId,
      completed: false,
      score: null,
      timeSpent: 0,
      completedSections: [],
      attempts: 0
    });

    this.progressRecords.set(progressId, progressRecord);
    return progressRecord;
  }

  /**
   * Get progress for a user in a module
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID
   * @returns {ProgressRecord|null} - Progress record or null if not found
   */
  async getProgress(userId, moduleId) {
    // Find progress record for this user and module
    for (const [id, record] of this.progressRecords) {
      if (record.userId === userId && record.moduleId === moduleId) {
        return record;
      }
    }
    return null;
  }

  /**
   * Update progress when a user completes a section
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID
   * @param {string} sectionId - Section ID that was completed
   * @returns {ProgressRecord} - Updated progress record
   */
  async updateSectionProgress(userId, moduleId, sectionId) {
    let progressRecord = await this.getProgress(userId, moduleId);

    if (!progressRecord) {
      progressRecord = await this.initializeProgress(userId, moduleId);
    }

    progressRecord.markSectionCompleted(sectionId);
    this.progressRecords.set(progressRecord.id, progressRecord);

    return progressRecord;
  }

  /**
   * Record time spent by a user
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID
   * @param {number} seconds - Time spent in seconds
   * @returns {ProgressRecord} - Updated progress record
   */
  async recordTimeSpent(userId, moduleId, seconds) {
    let progressRecord = await this.getProgress(userId, moduleId);

    if (!progressRecord) {
      progressRecord = await this.initializeProgress(userId, moduleId);
    }

    progressRecord.addTimeSpent(seconds);
    this.progressRecords.set(progressRecord.id, progressRecord);

    return progressRecord;
  }

  /**
   * Submit an assessment and record the results
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID
   * @param {string} assessmentId - Assessment ID
   * @param {Array} answers - Array of user answers
   * @returns {Object} - Assessment result
   */
  async submitAssessment(userId, moduleId, assessmentId, answers) {
    // Get the assessment to calculate score
    const assessment = await this.getAssessment(assessmentId);
    if (!assessment) {
      throw new Error(`Assessment ${assessmentId} not found`);
    }

    // Calculate score
    const score = this.calculateAssessmentScore(assessment, answers);

    // Update progress record
    let progressRecord = await this.getProgress(userId, moduleId);
    if (!progressRecord) {
      progressRecord = await this.initializeProgress(userId, moduleId);
    }

    progressRecord.recordAssessmentAttempt(score);

    // Store assessment result
    const resultId = `${userId}-${assessmentId}-${Date.now()}`;
    const result = {
      id: resultId,
      userId,
      moduleId,
      assessmentId,
      answers,
      score,
      timestamp: new Date().toISOString()
    };

    this.assessmentResults.set(resultId, result);
    this.progressRecords.set(progressRecord.id, progressRecord);

    return {
      result,
      progress: progressRecord
    };
  }

  /**
   * Calculate assessment score based on answers
   * @param {Assessment} assessment - The assessment object
   * @param {Array} answers - Array of user answers
   * @returns {number} - Score (0-100)
   */
  calculateAssessmentScore(assessment, answers) {
    // In a real implementation, we would use the Assessment model's calculateScore method
    // For now, we'll implement a basic scoring algorithm
    let correctAnswers = 0;
    const totalQuestions = assessment.questions.length;

    if (answers.length !== totalQuestions) {
      throw new Error('Number of answers does not match number of questions');
    }

    for (let i = 0; i < totalQuestions; i++) {
      const question = assessment.questions[i];
      const userAnswer = answers[i];

      if (this.isAnswerCorrect(question, userAnswer)) {
        correctAnswers++;
      }
    }

    return Math.round((correctAnswers / totalQuestions) * 100);
  }

  /**
   * Check if a user's answer is correct for a given question
   * @param {Object} question - Question object
   * @param {any} userAnswer - User's answer
   * @returns {boolean} - True if answer is correct
   */
  isAnswerCorrect(question, userAnswer) {
    // This is a simplified version - in reality, we'd use the Assessment model's method
    if (question.correctAnswer === undefined) return false;

    if (Array.isArray(question.correctAnswer)) {
      // Multiple correct answers
      if (!Array.isArray(userAnswer) || userAnswer.length !== question.correctAnswer.length) {
        return false;
      }
      return userAnswer.every(ans => question.correctAnswer.includes(ans));
    } else {
      // Single correct answer
      return userAnswer == question.correctAnswer;
    }
  }

  /**
   * Get assessment by ID (in a real implementation, this would fetch from a database)
   * @param {string} assessmentId - Assessment ID
   * @returns {Assessment|null} - Assessment object or null if not found
   */
  async getAssessment(assessmentId) {
    // This is a placeholder - in a real implementation, this would fetch from a database
    // For now, we'll return a dummy assessment for testing
    return new Assessment({
      id: assessmentId,
      moduleId: 'module-1',
      title: 'Sample Assessment',
      questions: [
        {
          id: 'q1',
          type: 'multiple-choice',
          questionText: 'What is ROS?',
          options: ['Robot Operating System', 'Robot Operating Service', 'Rapid Operating System', 'Robot Open Source'],
          correctAnswer: 'Robot Operating System'
        }
      ],
      maxScore: 100
    });
  }

  /**
   * Get progress summary for a user across all modules
   * @param {string} userId - User ID
   * @returns {Object} - Progress summary
   */
  async getUserProgressSummary(userId) {
    const userProgress = [];
    let totalModules = 0;
    let completedModules = 0;
    let totalTimeSpent = 0;
    let totalScore = 0;
    let assessmentsTaken = 0;

    for (const [id, record] of this.progressRecords) {
      if (record.userId === userId) {
        userProgress.push({
          moduleId: record.moduleId,
          completed: record.completed,
          score: record.score,
          timeSpent: record.timeSpent,
          progressPercentage: record.getProgressPercentage(10) // Assuming 10 sections per module for demo
        });

        totalModules++;
        if (record.completed) completedModules++;
        totalTimeSpent += record.timeSpent;
        if (record.score !== null) {
          totalScore += record.score;
          assessmentsTaken++;
        }
      }
    }

    const averageScore = assessmentsTaken > 0 ? totalScore / assessmentsTaken : 0;

    return {
      userId,
      totalModules,
      completedModules,
      completionPercentage: totalModules > 0 ? Math.round((completedModules / totalModules) * 100) : 0,
      totalTimeSpent,
      timeSpentFormatted: this.formatTime(totalTimeSpent),
      averageScore: Math.round(averageScore * 100) / 100,
      assessmentsTaken,
      modules: userProgress
    };
  }

  /**
   * Format time in seconds to human-readable string
   * @param {number} seconds - Time in seconds
   * @returns {string} - Formatted time string
   */
  formatTime(seconds) {
    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const remainingSeconds = seconds % 60;

    if (hours > 0) {
      return `${hours}h ${minutes}m ${remainingSeconds}s`;
    } else if (minutes > 0) {
      return `${minutes}m ${remainingSeconds}s`;
    } else {
      return `${remainingSeconds}s`;
    }
  }

  /**
   * Save user note for a specific location
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID
   * @param {string} location - Location in the module
   * @param {string} content - Note content
   * @returns {ProgressRecord} - Updated progress record
   */
  async saveNote(userId, moduleId, location, content) {
    let progressRecord = await this.getProgress(userId, moduleId);

    if (!progressRecord) {
      progressRecord = await this.initializeProgress(userId, moduleId);
    }

    progressRecord.addNote(location, content);
    this.progressRecords.set(progressRecord.id, progressRecord);

    return progressRecord;
  }

  /**
   * Save user bookmark for a specific location
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID
   * @param {string} location - Location to bookmark
   * @param {string} description - Optional description
   * @returns {ProgressRecord} - Updated progress record
   */
  async saveBookmark(userId, moduleId, location, description = '') {
    let progressRecord = await this.getProgress(userId, moduleId);

    if (!progressRecord) {
      progressRecord = await this.initializeProgress(userId, moduleId);
    }

    progressRecord.addBookmark(location, description);
    this.progressRecords.set(progressRecord.id, progressRecord);

    return progressRecord;
  }

  /**
   * Update current lesson for a user
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID
   * @param {string} lessonId - Current lesson ID
   * @returns {ProgressRecord} - Updated progress record
   */
  async updateCurrentLesson(userId, moduleId, lessonId) {
    let progressRecord = await this.getProgress(userId, moduleId);

    if (!progressRecord) {
      progressRecord = await this.initializeProgress(userId, moduleId);
    }

    progressRecord.updateCurrentLesson(lessonId);
    this.progressRecords.set(progressRecord.id, progressRecord);

    return progressRecord;
  }

  /**
   * Get all assessment results for a user
   * @param {string} userId - User ID
   * @returns {Array} - Array of assessment results
   */
  async getUserAssessmentResults(userId) {
    const results = [];

    for (const [id, result] of this.assessmentResults) {
      if (result.userId === userId) {
        results.push(result);
      }
    }

    return results;
  }

  /**
   * Reset progress for a user in a module
   * @param {string} userId - User ID
   * @param {string} moduleId - Module ID
   * @returns {ProgressRecord} - New progress record
   */
  async resetProgress(userId, moduleId) {
    // Find and remove existing progress
    for (const [id, record] of this.progressRecords) {
      if (record.userId === userId && record.moduleId === moduleId) {
        this.progressRecords.delete(id);
        break;
      }
    }

    // Create new progress record
    return await this.initializeProgress(userId, moduleId);
  }

  /**
   * Get all data for debugging/diagnostic purposes
   * @returns {Object} - All progress and assessment data
   */
  getAllData() {
    return {
      progressRecords: Array.from(this.progressRecords.values()),
      assessmentResults: Array.from(this.assessmentResults.values())
    };
  }
}

// Export a singleton instance
const progressService = new ProgressService();
module.exports = progressService;