/**
 * Validation utilities for the Physical AI & Humanoid Robotics textbook application
 */

export interface ValidationResult {
  isValid: boolean;
  errors: string[];
}

// Email validation
export const validateEmail = (email: string): ValidationResult => {
  const errors: string[] = [];
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;

  if (!email) {
    errors.push('Email is required');
  } else if (!emailRegex.test(email)) {
    errors.push('Please enter a valid email address');
  }

  return {
    isValid: errors.length === 0,
    errors,
  };
};

// Password validation
export const validatePassword = (password: string): ValidationResult => {
  const errors: string[] = [];

  if (!password) {
    errors.push('Password is required');
  } else if (password.length < 8) {
    errors.push('Password must be at least 8 characters long');
  } else {
    // Check for uppercase letter
    if (!/[A-Z]/.test(password)) {
      errors.push('Password must contain at least one uppercase letter');
    }

    // Check for lowercase letter
    if (!/[a-z]/.test(password)) {
      errors.push('Password must contain at least one lowercase letter');
    }

    // Check for number
    if (!/\d/.test(password)) {
      errors.push('Password must contain at least one number');
    }

    // Check for special character
    if (!/[@$!%*?&]/.test(password)) {
      errors.push('Password must contain at least one special character (@$!%*?&)');
    }
  }

  return {
    isValid: errors.length === 0,
    errors,
  };
};

// Confirm password validation
export const validateConfirmPassword = (password: string, confirmPassword: string): ValidationResult => {
  const errors: string[] = [];

  if (!confirmPassword) {
    errors.push('Please confirm your password');
  } else if (password !== confirmPassword) {
    errors.push('Passwords do not match');
  }

  return {
    isValid: errors.length === 0,
    errors,
  };
};

// Registration form validation
export const validateRegistrationForm = (
  email: string,
  password: string,
  confirmPassword: string,
  softwareBackground: string,
  aiExperience: string,
  roboticsExperience: string,
  programmingLanguages: string[],
  hardwareExperience: string,
  learningStyle: string
): ValidationResult => {
  const errors: string[] = [];

  // Validate email
  const emailValidation = validateEmail(email);
  if (!emailValidation.isValid) {
    errors.push(...emailValidation.errors);
  }

  // Validate password
  const passwordValidation = validatePassword(password);
  if (!passwordValidation.isValid) {
    errors.push(...passwordValidation.errors);
  }

  // Validate confirm password
  const confirmPasswordValidation = validateConfirmPassword(password, confirmPassword);
  if (!confirmPasswordValidation.isValid) {
    errors.push(...confirmPasswordValidation.errors);
  }

  // Validate required background fields
  if (!softwareBackground) {
    errors.push('Software background is required');
  }

  if (!aiExperience) {
    errors.push('AI/ML experience level is required');
  }

  if (!roboticsExperience) {
    errors.push('Robotics experience is required');
  }

  if (!hardwareExperience) {
    errors.push('Hardware experience is required');
  }

  if (!learningStyle) {
    errors.push('Preferred learning style is required');
  }

  // Validate programming languages
  if (!programmingLanguages || programmingLanguages.length === 0) {
    errors.push('Please specify at least one programming language');
  }

  return {
    isValid: errors.length === 0,
    errors,
  };
};

// Profile update validation
export const validateProfileUpdate = (
  softwareBackground?: string,
  aiExperience?: string,
  roboticsExperience?: string,
  programmingLanguages?: string[],
  hardwareExperience?: string,
  learningStyle?: string
): ValidationResult => {
  const errors: string[] = [];

  // Validate optional background fields if provided
  if (softwareBackground && !['None', 'Beginner', 'Intermediate', 'Advanced'].includes(softwareBackground)) {
    errors.push('Invalid software background value');
  }

  if (aiExperience && !['None', 'Basic ML', 'Deep Learning', 'Production AI'].includes(aiExperience)) {
    errors.push('Invalid AI/ML experience value');
  }

  if (roboticsExperience && !['None', 'Hobby Projects', 'Academic', 'Professional'].includes(roboticsExperience)) {
    errors.push('Invalid robotics experience value');
  }

  if (hardwareExperience && !['None', 'Raspberry Pi', 'Arduino', 'Jetson', 'Custom Boards'].includes(hardwareExperience)) {
    errors.push('Invalid hardware experience value');
  }

  if (learningStyle && !['Visual', 'Hands-on', 'Theoretical', 'Mixed'].includes(learningStyle)) {
    errors.push('Invalid learning style value');
  }

  // Validate programming languages if provided
  if (programmingLanguages && programmingLanguages.length > 0) {
    // Check if any language is empty or too short
    const invalidLanguages = programmingLanguages.filter(lang => !lang || lang.trim().length < 2);
    if (invalidLanguages.length > 0) {
      errors.push('Programming languages must be at least 2 characters long');
    }
  }

  return {
    isValid: errors.length === 0,
    errors,
  };
};

// General form validation helper
export const validateForm = (formData: any, requiredFields: string[]): ValidationResult => {
  const errors: string[] = [];

  for (const field of requiredFields) {
    if (!formData[field] || (Array.isArray(formData[field]) && formData[field].length === 0)) {
      errors.push(`${field} is required`);
    }
  }

  return {
    isValid: errors.length === 0,
    errors,
  };
};

export default {
  validateEmail,
  validatePassword,
  validateConfirmPassword,
  validateRegistrationForm,
  validateProfileUpdate,
  validateForm,
};