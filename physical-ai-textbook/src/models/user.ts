/**
 * User entity based on the data model specification
 */
export interface User {
  userId: string;
  email: string;
  softwareBackground: 'None' | 'Beginner' | 'Intermediate' | 'Advanced';
  aiExperience: 'None' | 'Basic ML' | 'Deep Learning' | 'Production AI';
  roboticsExperience: 'None' | 'Hobby Projects' | 'Academic' | 'Professional';
  programmingLanguages: string[]; // e.g., ['Python', 'C++', 'JavaScript', 'Other']
  hardwareExperience: 'None' | 'Raspberry Pi' | 'Arduino' | 'Jetson' | 'Custom Boards';
  learningStyle: 'Visual' | 'Hands-on' | 'Theoretical' | 'Mixed';
  createdAt: Date;
  lastLogin?: Date;
  progress?: {
    [moduleId: string]: {
      completed: boolean;
      score?: number;
      timeSpent: number; // in minutes
      lastAccessed: Date;
      completedSections: string[];
    };
  };
}

/**
 * User registration input data
 */
export interface UserRegistrationData {
  email: string;
  password: string;
  softwareBackground: 'None' | 'Beginner' | 'Intermediate' | 'Advanced';
  aiExperience: 'None' | 'Basic ML' | 'Deep Learning' | 'Production AI';
  roboticsExperience: 'None' | 'Hobby Projects' | 'Academic' | 'Professional';
  programmingLanguages: string[];
  hardwareExperience: 'None' | 'Raspberry Pi' | 'Arduino' | 'Jetson' | 'Custom Boards';
  learningStyle: 'Visual' | 'Hands-on' | 'Theoretical' | 'Mixed';
}

/**
 * User profile update data
 */
export interface UserProfileUpdate {
  softwareBackground?: 'None' | 'Beginner' | 'Intermediate' | 'Advanced';
  aiExperience?: 'None' | 'Basic ML' | 'Deep Learning' | 'Production AI';
  roboticsExperience?: 'None' | 'Hobby Projects' | 'Academic' | 'Professional';
  programmingLanguages?: string[];
  hardwareExperience?: 'None' | 'Raspberry Pi' | 'Arduino' | 'Jetson' | 'Custom Boards';
  learningStyle?: 'Visual' | 'Hands-on' | 'Theoretical' | 'Mixed';
}

/**
 * User progress record
 */
export interface UserProgress {
  progressId: string;
  userId: string;
  moduleId: string;
  completed: boolean;
  score?: number;
  timeSpent: number; // in minutes
  lastAccessed: Date;
  completedSections: string[];
}