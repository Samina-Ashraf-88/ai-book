/**
 * Logging utility for the Physical AI & Humanoid Robotics textbook application
 */

export enum LogLevel {
  DEBUG = 'DEBUG',
  INFO = 'INFO',
  WARN = 'WARN',
  ERROR = 'ERROR',
}

export interface LogEntry {
  timestamp: Date;
  level: LogLevel;
  message: string;
  context?: any;
  userId?: string;
}

class Logger {
  private static instance: Logger;
  private logLevel: LogLevel = LogLevel.INFO;

  private constructor() {}

  public static getInstance(): Logger {
    if (!Logger.instance) {
      Logger.instance = new Logger();
    }
    return Logger.instance;
  }

  public setLogLevel(level: LogLevel): void {
    this.logLevel = level;
  }

  private shouldLog(level: LogLevel): boolean {
    const levels = [LogLevel.DEBUG, LogLevel.INFO, LogLevel.WARN, LogLevel.ERROR];
    return levels.indexOf(level) >= levels.indexOf(this.logLevel);
  }

  private log(level: LogLevel, message: string, context?: any, userId?: string): void {
    if (!this.shouldLog(level)) {
      return;
    }

    const logEntry: LogEntry = {
      timestamp: new Date(),
      level,
      message,
      context,
      userId,
    };

    // In development, log to console
    if (process.env.NODE_ENV === 'development') {
      console.log(`[${logEntry.timestamp.toISOString()}] ${level}: ${message}`, {
        context: logEntry.context,
        userId: logEntry.userId,
      });
    }

    // In production, you might want to send logs to an external service
    // For now, we'll only log to console in development
    if (process.env.NODE_ENV === 'development') {
      console.log(JSON.stringify(logEntry));
    }
  }

  public debug(message: string, context?: any, userId?: string): void {
    this.log(LogLevel.DEBUG, message, context, userId);
  }

  public info(message: string, context?: any, userId?: string): void {
    this.log(LogLevel.INFO, message, context, userId);
  }

  public warn(message: string, context?: any, userId?: string): void {
    this.log(LogLevel.WARN, message, context, userId);
  }

  public error(message: string, context?: any, userId?: string): void {
    this.log(LogLevel.ERROR, message, context, userId);
  }
}

export const logger = Logger.getInstance();

export default logger;