/**
 * Progress Record Model
 * Represents a user's progress through the Physical AI & Humanoid Robotics textbook
 */

class ProgressRecord {
  constructor({
    id,
    userId,
    moduleId,
    assessmentId = null,
    completed = false,
    score = null,
    timeSpent = 0, // in seconds
    completedSections = [],
    lastAccessed,
    attempts = 0,
    currentLesson = null,
    bookmarks = [],
    notes = [],
    createdAt,
    updatedAt
  }) {
    this.id = id;
    this.userId = userId;
    this.moduleId = moduleId;
    this.assessmentId = assessmentId;
    this.completed = completed;
    this.score = score;
    this.timeSpent = timeSpent;
    this.completedSections = completedSections;
    this.lastAccessed = lastAccessed || new Date().toISOString();
    this.attempts = attempts;
    this.currentLesson = currentLesson;
    this.bookmarks = bookmarks;
    this.notes = notes;
    this.createdAt = createdAt || new Date().toISOString();
    this.updatedAt = updatedAt || new Date().toISOString();
  }

  /**
   * Mark a section as completed
   * @param {string} sectionId - ID of the section to mark as completed
   */
  markSectionCompleted(sectionId) {
    if (!this.completedSections.includes(sectionId)) {
      this.completedSections.push(sectionId);
      this.updatedAt = new Date().toISOString();
    }
  }

  /**
   * Add time spent to the total
   * @param {number} seconds - Time spent in seconds
   */
  addTimeSpent(seconds) {
    this.timeSpent += seconds;
    this.updatedAt = new Date().toISOString();
  }

  /**
   * Record an assessment attempt
   * @param {number} score - Score achieved on the assessment
   */
  recordAssessmentAttempt(score) {
    this.attempts += 1;
    this.score = score;
    this.updatedAt = new Date().toISOString();

    // If score is high enough, mark as completed
    if (score >= 70) { // 70% as passing threshold
      this.completed = true;
    }
  }

  /**
   * Add a bookmark at a specific location
   * @param {string} location - The location to bookmark
   * @param {string} description - Optional description
   */
  addBookmark(location, description = '') {
    const bookmark = {
      id: this.generateId(),
      location,
      description,
      timestamp: new Date().toISOString()
    };

    // Check if bookmark already exists
    const existingIndex = this.bookmarks.findIndex(b => b.location === location);
    if (existingIndex >= 0) {
      this.bookmarks[existingIndex] = bookmark;
    } else {
      this.bookmarks.push(bookmark);
    }

    this.updatedAt = new Date().toISOString();
  }

  /**
   * Remove a bookmark
   * @param {string} location - The location of the bookmark to remove
   */
  removeBookmark(location) {
    this.bookmarks = this.bookmarks.filter(b => b.location !== location);
    this.updatedAt = new Date().toISOString();
  }

  /**
   * Add a note
   * @param {string} location - The location where the note is added
   * @param {string} content - The note content
   */
  addNote(location, content) {
    const note = {
      id: this.generateId(),
      location,
      content,
      timestamp: new Date().toISOString()
    };

    this.notes.push(note);
    this.updatedAt = new Date().toISOString();
  }

  /**
   * Update a note
   * @param {string} noteId - The ID of the note to update
   * @param {string} content - The new content
   */
  updateNote(noteId, content) {
    const noteIndex = this.notes.findIndex(n => n.id === noteId);
    if (noteIndex >= 0) {
      this.notes[noteIndex].content = content;
      this.notes[noteIndex].timestamp = new Date().toISOString();
      this.updatedAt = new Date().toISOString();
    }
  }

  /**
   * Remove a note
   * @param {string} noteId - The ID of the note to remove
   */
  removeNote(noteId) {
    this.notes = this.notes.filter(n => n.id !== noteId);
    this.updatedAt = new Date().toISOString();
  }

  /**
   * Update the current lesson the user is on
   * @param {string} lessonId - The ID of the current lesson
   */
  updateCurrentLesson(lessonId) {
    this.currentLesson = lessonId;
    this.lastAccessed = new Date().toISOString();
    this.updatedAt = new Date().toISOString();
  }

  /**
   * Get progress percentage for the module
   * @param {number} totalSections - Total number of sections in the module
   * @returns {number} - Progress percentage
   */
  getProgressPercentage(totalSections) {
    if (totalSections <= 0) return 0;

    const sectionsCompleted = this.completedSections.length;
    const sectionProgress = (sectionsCompleted / totalSections) * 100;

    // If the assessment is completed, it counts as additional progress
    const assessmentProgress = this.completed ? 10 : 0; // 10% for completing assessment

    return Math.min(100, Math.round(sectionProgress + assessmentProgress));
  }

  /**
   * Check if the module is fully completed
   * @param {number} totalSections - Total number of sections in the module
   * @returns {boolean} - True if module is completed
   */
  isModuleComplete(totalSections) {
    return this.completed &&
           this.completedSections.length >= totalSections;
  }

  /**
   * Get the time spent in a human-readable format
   * @returns {string} - Formatted time string
   */
  getTimeSpentFormatted() {
    const hours = Math.floor(this.timeSpent / 3600);
    const minutes = Math.floor((this.timeSpent % 3600) / 60);
    const seconds = this.timeSpent % 60;

    if (hours > 0) {
      return `${hours}h ${minutes}m ${seconds}s`;
    } else if (minutes > 0) {
      return `${minutes}m ${seconds}s`;
    } else {
      return `${seconds}s`;
    }
  }

  /**
   * Get the average time spent per section
   * @param {number} totalSections - Total number of sections in the module
   * @returns {number} - Average time in seconds
   */
  getAverageTimePerSection(totalSections) {
    if (totalSections <= 0) return 0;
    return Math.round(this.timeSpent / totalSections);
  }

  /**
   * Get a summary of the progress
   * @param {number} totalSections - Total number of sections in the module
   * @returns {Object} - Progress summary
   */
  getProgressSummary(totalSections) {
    return {
      progressPercentage: this.getProgressPercentage(totalSections),
      sectionsCompleted: this.completedSections.length,
      totalSections,
      timeSpent: this.timeSpent,
      timeSpentFormatted: this.getTimeSpentFormatted(),
      averageTimePerSection: this.getAverageTimePerSection(totalSections),
      attempts: this.attempts,
      score: this.score,
      completed: this.completed,
      bookmarksCount: this.bookmarks.length,
      notesCount: this.notes.length,
      lastAccessed: this.lastAccessed
    };
  }

  /**
   * Validate the progress record structure
   */
  validate() {
    const errors = [];

    if (!this.id) errors.push('ID is required');
    if (!this.userId) errors.push('User ID is required');
    if (!this.moduleId) errors.push('Module ID is required');
    if (typeof this.completed !== 'boolean') errors.push('Completed status must be boolean');
    if (typeof this.timeSpent !== 'number' || this.timeSpent < 0) errors.push('Time spent must be a non-negative number');
    if (!Array.isArray(this.completedSections)) errors.push('Completed sections must be an array');

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  /**
   * Generate a unique ID for internal items
   */
  generateId() {
    return Date.now().toString(36) + Math.random().toString(36).substr(2);
  }

  /**
   * Merge another progress record into this one
   * @param {ProgressRecord} otherRecord - The other progress record to merge
   */
  merge(otherRecord) {
    // Update basic properties from the other record if it's more recent
    if (new Date(otherRecord.updatedAt) > new Date(this.updatedAt)) {
      this.completed = otherRecord.completed;
      this.score = otherRecord.score;
      this.timeSpent = Math.max(this.timeSpent, otherRecord.timeSpent);
      this.attempts = Math.max(this.attempts, otherRecord.attempts);
      this.currentLesson = otherRecord.currentLesson;
      this.lastAccessed = otherRecord.lastAccessed;
      this.updatedAt = otherRecord.updatedAt;
    }

    // Merge completed sections
    this.completedSections = [...new Set([...this.completedSections, ...otherRecord.completedSections])];

    // Merge bookmarks (keep the most recent for each location)
    const bookmarkMap = new Map();
    [...this.bookmarks, ...otherRecord.bookmarks].forEach(bookmark => {
      const existing = bookmarkMap.get(bookmark.location);
      if (!existing || new Date(bookmark.timestamp) > new Date(existing.timestamp)) {
        bookmarkMap.set(bookmark.location, bookmark);
      }
    });
    this.bookmarks = Array.from(bookmarkMap.values());

    // Merge notes (combine both sets)
    this.notes = [...this.notes, ...otherRecord.notes];
  }
}

module.exports = ProgressRecord;