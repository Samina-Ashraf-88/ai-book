const Assessment = require('../../src/models/assessment');

describe('Assessment Model', () => {
  describe('constructor', () => {
    it('should create an assessment with provided properties', () => {
      const assessmentData = {
        id: 'test-id',
        moduleId: 'module-1',
        title: 'Test Assessment',
        description: 'A test assessment',
        questions: [],
        maxScore: 100,
        timeLimit: 300
      };

      const assessment = new Assessment(assessmentData);

      expect(assessment.id).toBe('test-id');
      expect(assessment.moduleId).toBe('module-1');
      expect(assessment.title).toBe('Test Assessment');
      expect(assessment.description).toBe('A test assessment');
      expect(assessment.questions).toEqual([]);
      expect(assessment.maxScore).toBe(100);
      expect(assessment.timeLimit).toBe(300);
    });

    it('should set default values when not provided', () => {
      const assessmentData = {
        id: 'test-id',
        moduleId: 'module-1',
        title: 'Test Assessment',
        questions: []
      };

      const assessment = new Assessment(assessmentData);

      expect(assessment.description).toBeUndefined();
      expect(assessment.maxScore).toBe(100); // default
      expect(assessment.timeLimit).toBeNull(); // default
      expect(assessment.isActive).toBe(true); // default
      expect(assessment.createdAt).toBeDefined();
      expect(assessment.updatedAt).toBeDefined();
    });
  });

  describe('calculateScore', () => {
    it('should calculate score correctly for multiple choice questions', () => {
      const questions = [
        {
          type: 'multiple-choice',
          correctAnswer: 'Option A'
        },
        {
          type: 'multiple-choice',
          correctAnswer: 'Option B'
        }
      ];

      const assessment = new Assessment({
        id: 'test-id',
        moduleId: 'module-1',
        title: 'Test Assessment',
        questions: questions,
        maxScore: 100
      });

      const userAnswers = ['Option A', 'Wrong Option']; // 1 out of 2 correct
      const score = assessment.calculateScore(userAnswers);

      expect(score).toBe(50); // 50% of max score
    });

    it('should handle multiple correct answers for multiple choice', () => {
      const questions = [
        {
          type: 'multiple-choice',
          correctAnswer: ['Option A', 'Option C']
        }
      ];

      const assessment = new Assessment({
        id: 'test-id',
        moduleId: 'module-1',
        title: 'Test Assessment',
        questions: questions,
        maxScore: 100
      });

      const userAnswers = [['Option A', 'Option C']]; // Correct
      const score = assessment.calculateScore(userAnswers);

      expect(score).toBe(100); // 100% of max score
    });

    it('should calculate score with weighted questions', () => {
      const questions = [
        {
          type: 'multiple-choice',
          correctAnswer: 'Option A',
          weight: 2
        },
        {
          type: 'true-false',
          correctAnswer: true,
          weight: 1
        }
      ];

      const assessment = new Assessment({
        id: 'test-id',
        moduleId: 'module-1',
        title: 'Test Assessment',
        questions: questions,
        maxScore: 100
      });

      const userAnswers = ['Option A', false]; // First correct (weight 2), second wrong (weight 1)
      const score = assessment.calculateScore(userAnswers);

      // 2 points out of 3 total points = 66.67% of max score
      expect(score).toBeCloseTo(66.67);
    });

    it('should throw error for invalid number of answers', () => {
      const assessment = new Assessment({
        id: 'test-id',
        moduleId: 'module-1',
        title: 'Test Assessment',
        questions: [{ type: 'multiple-choice', correctAnswer: 'A' }],
        maxScore: 100
      });

      expect(() => {
        assessment.calculateScore([]);
      }).toThrow('Invalid number of answers provided');
    });
  });

  describe('isAnswerCorrect', () => {
    const assessment = new Assessment({
      id: 'test-id',
      moduleId: 'module-1',
      title: 'Test Assessment',
      questions: [],
      maxScore: 100
    });

    it('should correctly validate multiple choice answers', () => {
      const question = {
        type: 'multiple-choice',
        correctAnswer: 'Option A'
      };

      expect(assessment.isAnswerCorrect(question, 'Option A')).toBe(true);
      expect(assessment.isAnswerCorrect(question, 'Option B')).toBe(false);
    });

    it('should correctly validate true/false answers', () => {
      const question = {
        type: 'true-false',
        correctAnswer: true
      };

      expect(assessment.isAnswerCorrect(question, true)).toBe(true);
      expect(assessment.isAnswerCorrect(question, false)).toBe(false);
    });

    it('should correctly validate short answer answers', () => {
      const question = {
        type: 'short-answer',
        correctAnswer: 'correct answer'
      };

      expect(assessment.isAnswerCorrect(question, 'correct answer')).toBe(true);
      expect(assessment.isAnswerCorrect(question, 'Correct Answer')).toBe(true); // Case insensitive
      expect(assessment.isAnswerCorrect(question, 'wrong answer')).toBe(false);
    });

    it('should handle multiple possible correct answers for short answers', () => {
      const question = {
        type: 'short-answer',
        correctAnswer: ['answer 1', 'answer 2']
      };

      expect(assessment.isAnswerCorrect(question, 'answer 1')).toBe(true);
      expect(assessment.isAnswerCorrect(question, 'answer 2')).toBe(true);
      expect(assessment.isAnswerCorrect(question, 'wrong answer')).toBe(false);
    });
  });

  describe('validate', () => {
    it('should return valid for proper assessment', () => {
      const assessment = new Assessment({
        id: 'test-id',
        moduleId: 'module-1',
        title: 'Test Assessment',
        questions: [
          {
            type: 'multiple-choice',
            questionText: 'Sample question?',
            correctAnswer: 'A'
          }
        ]
      });

      const result = assessment.validate();

      expect(result.isValid).toBe(true);
      expect(result.errors).toEqual([]);
    });

    it('should return invalid for assessment without required fields', () => {
      const assessment = new Assessment({
        id: '', // Missing ID
        moduleId: '', // Missing module ID
        title: '', // Missing title
        questions: [] // No questions
      });

      const result = assessment.validate();

      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('ID is required');
      expect(result.errors).toContain('Module ID is required');
      expect(result.errors).toContain('Title is required');
      expect(result.errors).toContain('At least one question is required');
    });

    it('should return invalid for questions without required fields', () => {
      const assessment = new Assessment({
        id: 'test-id',
        moduleId: 'module-1',
        title: 'Test Assessment',
        questions: [
          {
            // Missing type
            questionText: 'Sample question?'
            // Missing correctAnswer
          }
        ]
      });

      const result = assessment.validate();

      expect(result.isValid).toBe(false);
      expect(result.errors).toContain('Question 0 must have a type');
      expect(result.errors).toContain('Question 0 must have a correct answer');
    });
  });

  describe('getForStudent', () => {
    it('should return student view without answers', () => {
      const questions = [
        {
          id: 'q1',
          type: 'multiple-choice',
          questionText: 'What is 2+2?',
          options: ['3', '4', '5'],
          correctAnswer: '4'
        }
      ];

      const assessment = new Assessment({
        id: 'test-id',
        moduleId: 'module-1',
        title: 'Test Assessment',
        questions: questions,
        maxScore: 100,
        timeLimit: 300
      });

      const studentView = assessment.getForStudent();

      expect(studentView.id).toBe('test-id');
      expect(studentView.moduleId).toBe('module-1');
      expect(studentView.title).toBe('Test Assessment');
      expect(studentView.questions).toHaveLength(1);
      expect(studentView.questions[0]).toEqual({
        id: 'q1',
        type: 'multiple-choice',
        questionText: 'What is 2+2?',
        options: ['3', '4', '5'],
        required: false
      });
      // Should not include correctAnswer
      expect(studentView.questions[0].correctAnswer).toBeUndefined();
    });
  });
});