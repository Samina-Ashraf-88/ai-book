const ProgressRecord = require('../../src/models/progress');

describe('ProgressRecord Model', () => {
  describe('constructor', () => {
    it('should create a progress record with provided properties', () => {
      const progressData = {
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        completed: false,
        score: null,
        timeSpent: 0,
        completedSections: [],
        lastAccessed: new Date().toISOString(),
        attempts: 0
      };

      const progress = new ProgressRecord(progressData);

      expect(progress.id).toBe('progress-id');
      expect(progress.userId).toBe('user-1');
      expect(progress.moduleId).toBe('module-1');
      expect(progress.completed).toBe(false);
      expect(progress.score).toBeNull();
      expect(progress.timeSpent).toBe(0);
      expect(progress.completedSections).toEqual([]);
      expect(progress.attempts).toBe(0);
      expect(progress.createdAt).toBeDefined();
      expect(progress.updatedAt).toBeDefined();
    });

    it('should set default values when not provided', () => {
      const progressData = {
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1'
      };

      const progress = new ProgressRecord(progressData);

      expect(progress.completed).toBe(false);
      expect(progress.score).toBeNull();
      expect(progress.timeSpent).toBe(0);
      expect(progress.completedSections).toEqual([]);
      expect(progress.attempts).toBe(0);
      expect(progress.bookmarks).toEqual([]);
      expect(progress.notes).toEqual([]);
      expect(progress.createdAt).toBeDefined();
      expect(progress.updatedAt).toBeDefined();
    });
  });

  describe('markSectionCompleted', () => {
    it('should add section to completed sections', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1'
      });

      progress.markSectionCompleted('section-1');

      expect(progress.completedSections).toContain('section-1');
    });

    it('should not duplicate completed sections', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        completedSections: ['section-1']
      });

      progress.markSectionCompleted('section-1');

      expect(progress.completedSections).toEqual(['section-1']);
    });
  });

  describe('addTimeSpent', () => {
    it('should add time to total time spent', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        timeSpent: 100
      });

      progress.addTimeSpent(50);

      expect(progress.timeSpent).toBe(150);
    });

    it('should update updatedAt when time is added', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        timeSpent: 100
      });

      const oldUpdatedAt = progress.updatedAt;

      setTimeout(() => {
        progress.addTimeSpent(50);
        expect(progress.updatedAt).not.toBe(oldUpdatedAt);
      }, 10);
    });
  });

  describe('recordAssessmentAttempt', () => {
    it('should record assessment score and update attempts', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        attempts: 1
      });

      progress.recordAssessmentAttempt(85);

      expect(progress.score).toBe(85);
      expect(progress.attempts).toBe(2);
    });

    it('should mark as completed if score is 70 or higher', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        completed: false
      });

      progress.recordAssessmentAttempt(75);

      expect(progress.completed).toBe(true);
    });

    it('should not mark as completed if score is below 70', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        completed: false
      });

      progress.recordAssessmentAttempt(65);

      expect(progress.completed).toBe(false);
    });
  });

  describe('addBookmark and removeBookmark', () => {
    it('should add a bookmark', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1'
      });

      progress.addBookmark('section-1', 'Important concept');

      expect(progress.bookmarks).toHaveLength(1);
      expect(progress.bookmarks[0].location).toBe('section-1');
      expect(progress.bookmarks[0].description).toBe('Important concept');
    });

    it('should update existing bookmark', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1'
      });

      progress.addBookmark('section-1', 'Old description');
      progress.addBookmark('section-1', 'New description');

      expect(progress.bookmarks).toHaveLength(1);
      expect(progress.bookmarks[0].description).toBe('New description');
    });

    it('should remove a bookmark', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1'
      });

      progress.addBookmark('section-1', 'Important concept');
      progress.removeBookmark('section-1');

      expect(progress.bookmarks).toHaveLength(0);
    });
  });

  describe('addNote and removeNote', () => {
    it('should add a note', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1'
      });

      progress.addNote('section-1', 'This is important');

      expect(progress.notes).toHaveLength(1);
      expect(progress.notes[0].location).toBe('section-1');
      expect(progress.notes[0].content).toBe('This is important');
    });

    it('should update a note', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1'
      });

      progress.addNote('section-1', 'Old note');
      const noteId = progress.notes[0].id;

      progress.updateNote(noteId, 'New note');

      expect(progress.notes[0].content).toBe('New note');
    });

    it('should remove a note', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1'
      });

      progress.addNote('section-1', 'Note to remove');
      const noteId = progress.notes[0].id;

      progress.removeNote(noteId);

      expect(progress.notes).toHaveLength(0);
    });
  });

  describe('updateCurrentLesson', () => {
    it('should update current lesson', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1'
      });

      progress.updateCurrentLesson('lesson-2');

      expect(progress.currentLesson).toBe('lesson-2');
    });

    it('should update lastAccessed and updatedAt', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1'
      });

      const oldLastAccessed = progress.lastAccessed;
      const oldUpdatedAt = progress.updatedAt;

      setTimeout(() => {
        progress.updateCurrentLesson('lesson-2');
        expect(progress.lastAccessed).not.toBe(oldLastAccessed);
        expect(progress.updatedAt).not.toBe(oldUpdatedAt);
      }, 10);
    });
  });

  describe('getProgressPercentage', () => {
    it('should calculate progress percentage correctly', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        completedSections: ['section-1', 'section-2']
      });

      const percentage = progress.getProgressPercentage(5); // 5 total sections

      expect(percentage).toBe(40); // 2 out of 5 sections = 40%
    });

    it('should include assessment completion in progress', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        completedSections: ['section-1', 'section-2', 'section-3', 'section-4'],
        completed: true // Assessment completed
      });

      const percentage = progress.getProgressPercentage(5); // 5 total sections

      // 4 sections (80%) + assessment (10%) = 90%
      expect(percentage).toBe(90);
    });

    it('should not exceed 100%', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        completedSections: ['section-1', 'section-2', 'section-3'],
        completed: true // Assessment completed
      });

      const percentage = progress.getProgressPercentage(3); // 3 total sections

      // 3 sections (100%) + assessment (10%) = 110% but capped at 100%
      expect(percentage).toBe(100);
    });
  });

  describe('isModuleComplete', () => {
    it('should return true if all sections completed and assessment passed', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        completedSections: ['section-1', 'section-2', 'section-3'],
        completed: true
      });

      const isComplete = progress.isModuleComplete(3);

      expect(isComplete).toBe(true);
    });

    it('should return false if not all sections completed', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        completedSections: ['section-1'],
        completed: true
      });

      const isComplete = progress.isModuleComplete(3);

      expect(isComplete).toBe(false);
    });

    it('should return false if assessment not passed', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        completedSections: ['section-1', 'section-2', 'section-3'],
        completed: false
      });

      const isComplete = progress.isModuleComplete(3);

      expect(isComplete).toBe(false);
    });
  });

  describe('getTimeSpentFormatted', () => {
    it('should format time in hours, minutes, and seconds', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        timeSpent: 3725 // 1 hour, 2 minutes, 5 seconds
      });

      const formatted = progress.getTimeSpentFormatted();

      expect(formatted).toBe('1h 2m 5s');
    });

    it('should format time in minutes and seconds when less than an hour', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        timeSpent: 125 // 2 minutes, 5 seconds
      });

      const formatted = progress.getTimeSpentFormatted();

      expect(formatted).toBe('2m 5s');
    });

    it('should format time in seconds when less than a minute', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        timeSpent: 45 // 45 seconds
      });

      const formatted = progress.getTimeSpentFormatted();

      expect(formatted).toBe('45s');
    });
  });

  describe('getAverageTimePerSection', () => {
    it('should calculate average time per section', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        timeSpent: 300, // 5 minutes total
        completedSections: ['section-1', 'section-2', 'section-3']
      });

      const avgTime = progress.getAverageTimePerSection(5); // 5 total sections

      expect(avgTime).toBe(60); // 300 seconds / 5 sections = 60 seconds per section
    });

    it('should return 0 if total sections is 0', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        timeSpent: 300
      });

      const avgTime = progress.getAverageTimePerSection(0);

      expect(avgTime).toBe(0);
    });
  });

  describe('getProgressSummary', () => {
    it('should return a complete progress summary', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1',
        completedSections: ['section-1', 'section-2'],
        completed: true,
        score: 85,
        timeSpent: 1800, // 30 minutes
        attempts: 2,
        bookmarks: [{ id: 'b1', location: 'section-1', description: 'Bookmark' }],
        notes: [{ id: 'n1', location: 'section-1', content: 'Note' }],
        lastAccessed: new Date().toISOString()
      });

      const summary = progress.getProgressSummary(5); // 5 total sections

      expect(summary.progressPercentage).toBe(50); // 2 sections + assessment = 50%
      expect(summary.sectionsCompleted).toBe(2);
      expect(summary.totalSections).toBe(5);
      expect(summary.timeSpent).toBe(1800);
      expect(summary.timeSpentFormatted).toBe('30m 0s');
      expect(summary.averageTimePerSection).toBe(360); // 1800 / 5
      expect(summary.attempts).toBe(2);
      expect(summary.score).toBe(85);
      expect(summary.completed).toBe(true);
      expect(summary.bookmarksCount).toBe(1);
      expect(summary.notesCount).toBe(1);
      expect(summary.lastAccessed).toBeDefined();
    });
  });

  describe('validate', () => {
    it('should return valid for proper progress record', () => {
      const progress = new ProgressRecord({
        id: 'progress-id',
        userId: 'user-1',
        moduleId: 'module-1'
      });

      const result = progress.validate();

      expect(result.isValid).toBe(true);
      expect(result.errors).toEqual([]);
    });

    it('should return invalid for progress record without required fields', () => {
      const progress = new ProgressRecord({
        id: '', // Missing ID
        userId: '', // Missing user ID
        moduleId: '', // Missing module ID
        timeSpent: -10, // Negative time
        completedSections: 'not-an-array' // Should be array
      });

      const result = progress.validate();

      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('ID is required');
      expect(result.errors).toContain('User ID is required');
      expect(result.errors).toContain('Module ID is required');
      expect(result.errors).toContain('Time spent must be a non-negative number');
      expect(result.errors).toContain('Completed sections must be an array');
    });
  });
});