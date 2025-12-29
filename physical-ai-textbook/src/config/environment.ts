/**
 * Environment configuration management for the Physical AI & Humanoid Robotics textbook
 */

// Define the structure of our configuration
interface AppConfig {
  // Application settings
  app: {
    name: string;
    version: string;
    environment: 'development' | 'staging' | 'production';
    port: number;
    url: string;
  };

  // Database settings
  database: {
    url: string;
    connectionTimeout: number;
  };

  // Authentication settings
  auth: {
    jwtSecret: string;
    sessionTimeout: number; // in minutes
    passwordRequirements: {
      minLength: number;
      requireNumbers: boolean;
      requireSpecialChars: boolean;
    };
  };

  // Internationalization settings
  i18n: {
    defaultLocale: string;
    supportedLocales: string[];
    fallbackLocale: string;
  };

  // Logging settings
  logging: {
    level: 'debug' | 'info' | 'warn' | 'error';
    format: 'json' | 'text';
  };

  // Performance settings
  performance: {
    maxConcurrentRequests: number;
    cacheTimeout: number; // in seconds
    responseTimeout: number; // in seconds
  };

  // External services
  services: {
    // Add configuration for any external services here
  };
}

// Default configuration values
const defaultConfig: AppConfig = {
  app: {
    name: 'Physical AI & Humanoid Robotics Textbook',
    version: '1.0.0',
    environment: (process.env.NODE_ENV as 'development' | 'staging' | 'production') || 'development',
    port: parseInt(process.env.PORT || '3000', 10),
    url: process.env.APP_URL || 'http://localhost:3000',
  },
  database: {
    url: process.env.DATABASE_URL || './sqlite.db',
    connectionTimeout: parseInt(process.env.DB_CONNECTION_TIMEOUT || '5000', 10),
  },
  auth: {
    jwtSecret: process.env.JWT_SECRET || 'default-secret-for-dev',
    sessionTimeout: parseInt(process.env.SESSION_TIMEOUT || '1440', 10), // 24 hours
    passwordRequirements: {
      minLength: parseInt(process.env.PASSWORD_MIN_LENGTH || '8', 10),
      requireNumbers: process.env.PASSWORD_REQUIRE_NUMBERS === 'true',
      requireSpecialChars: process.env.PASSWORD_REQUIRE_SPECIAL_CHARS === 'true',
    },
  },
  i18n: {
    defaultLocale: 'en',
    supportedLocales: ['en', 'ur'], // English and Urdu
    fallbackLocale: 'en',
  },
  logging: {
    level: (process.env.LOG_LEVEL as 'debug' | 'info' | 'warn' | 'error') || 'info',
    format: (process.env.LOG_FORMAT as 'json' | 'text') || 'json',
  },
  performance: {
    maxConcurrentRequests: parseInt(process.env.MAX_CONCURRENT_REQUESTS || '100', 10),
    cacheTimeout: parseInt(process.env.CACHE_TIMEOUT || '3600', 10), // 1 hour
    responseTimeout: parseInt(process.env.RESPONSE_TIMEOUT || '30', 10), // 30 seconds
  },
  services: {
    // Add any external service configurations here
  },
};

// Validate configuration
function validateConfig(config: AppConfig): void {
  if (!config.app.environment) {
    throw new Error('Environment must be set to development, staging, or production');
  }

  if (config.app.port <= 0 || config.app.port > 65535) {
    throw new Error('Port must be a valid port number between 1 and 65535');
  }

  if (!config.auth.jwtSecret || config.auth.jwtSecret.length < 32) {
    throw new Error('JWT secret must be at least 32 characters long');
  }
}

// Initialize configuration
const config: AppConfig = { ...defaultConfig };

// Validate the configuration
validateConfig(config);

export default config;