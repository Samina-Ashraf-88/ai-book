/**
 * Course Module entity based on the data model specification
 */

export interface CourseModule {
  moduleId: string;
  title: string;
  description: string;
  weekRange: string; // e.g., "Weeks 3-5"
  content: string; // Markdown content
  learningObjectives: string[];
  prerequisites: string[]; // Array of module IDs or knowledge areas
  sections: ModuleSection[];
  assessments?: ModuleAssessment[];
  createdAt: Date;
  updatedAt: Date;
  order: number; // Order in which the module appears
}

export interface ModuleSection {
  id: string;
  title: string;
  content: string; // Markdown content
  duration?: number; // Estimated time to complete in minutes
  interactiveElements?: InteractiveElement[];
  objectives?: string[]; // Learning objectives specific to this section
  prerequisites?: string[]; // Prerequisites for this section
  order: number; // Order within the module
}

export interface InteractiveElement {
  id: string;
  type: 'code-example' | 'diagram' | 'exercise' | 'video' | 'simulation' | 'quiz';
  content: string; // The actual content (code, image path, etc.)
  language?: string; // For code examples
  caption?: string;
  isInteractive?: boolean; // Whether user can interact with this element
  copyable?: boolean; // For code examples
}

export interface ModuleAssessment {
  id: string;
  title: string;
  type: 'quiz' | 'exercise' | 'project' | 'simulation';
  questions: AssessmentQuestion[];
  maxScore: number;
  timeLimit?: number; // in minutes
  weight: number; // Weight of this assessment in overall module grade
}

export interface AssessmentQuestion {
  id: string;
  type: 'multiple-choice' | 'short-answer' | 'code' | 'simulation';
  question: string;
  options?: string[]; // For multiple choice
  correctAnswer: string | string[]; // For multiple choice, could be array for multiple correct answers
  explanation?: string;
  points: number;
}

export interface ModuleProgress {
  moduleId: string;
  userId: string;
  completed: boolean;
  score?: number;
  timeSpent: number; // in minutes
  lastAccessed: Date;
  completedSections: string[]; // Array of section IDs
  currentSection?: string; // ID of the section the user was last on
}

/**
 * Input data for creating a new course module
 */
export interface CreateCourseModuleData {
  title: string;
  description: string;
  weekRange: string;
  content: string;
  learningObjectives: string[];
  prerequisites: string[];
  sections: Omit<ModuleSection, 'id' | 'order'>[];
  assessments?: Omit<ModuleAssessment, 'id'>[];
  order: number;
}

/**
 * Input data for updating a course module
 */
export interface UpdateCourseModuleData {
  title?: string;
  description?: string;
  weekRange?: string;
  content?: string;
  learningObjectives?: string[];
  prerequisites?: string[];
  sections?: Omit<ModuleSection, 'id' | 'order'>[];
  assessments?: Omit<ModuleAssessment, 'id'>[];
  order?: number;
}