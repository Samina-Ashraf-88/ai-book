import { logger } from './logger';

export interface AppError extends Error {
  statusCode: number;
  isOperational: boolean;
  context?: any;
}

export class ApplicationError extends Error implements AppError {
  public statusCode: number;
  public isOperational: boolean;
  public context?: any;

  constructor(
    message: string,
    statusCode: number,
    isOperational = true,
    context?: any
  ) {
    super(message);
    this.statusCode = statusCode;
    this.isOperational = isOperational;
    this.context = context;

    // Set the prototype explicitly to maintain proper inheritance
    Object.setPrototypeOf(this, new.target.prototype);
  }
}

export class ValidationError extends ApplicationError {
  constructor(message: string, context?: any) {
    super(message, 400, true, context);
    this.name = 'ValidationError';
  }
}

export class AuthenticationError extends ApplicationError {
  constructor(message: string = 'Authentication required') {
    super(message, 401, true);
    this.name = 'AuthenticationError';
  }
}

export class AuthorizationError extends ApplicationError {
  constructor(message: string = 'Insufficient permissions') {
    super(message, 403, true);
    this.name = 'AuthorizationError';
  }
}

export class NotFoundError extends ApplicationError {
  constructor(resource: string) {
    super(`${resource} not found`, 404, true);
    this.name = 'NotFoundError';
  }
}

export class RateLimitError extends ApplicationError {
  constructor(message: string = 'Rate limit exceeded') {
    super(message, 429, true);
    this.name = 'RateLimitError';
  }
}

export const errorHandler = {
  handle: (error: Error, userId?: string): AppError => {
    // Log the error
    logger.error(error.message, {
      stack: error.stack,
      name: error.name,
    }, userId);

    // If it's already an AppError, return it
    if (isAppError(error)) {
      return error;
    }

    // For operational errors, return a generic error
    if (error.message.includes('timeout') || error.message.includes('network')) {
      return new ApplicationError('A network error occurred', 503);
    }

    // For all other errors, return a generic server error
    return new ApplicationError('An unexpected error occurred', 500);
  },

  isAppError: (error: Error): error is AppError => {
    return (error as AppError).statusCode !== undefined;
  },
};

// Type guard function
function isAppError(error: Error): error is AppError {
  return (error as AppError).statusCode !== undefined;
}

export default errorHandler;