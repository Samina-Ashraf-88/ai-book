import { betterAuth } from "better-auth";

export const auth = betterAuth({
  database: {
    provider: "sqlite",
    url: process.env.DATABASE_URL || "./sqlite.db",
  },
  socialProviders: {
    // Add social providers here if needed
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
  },
  user: {
    // Define custom fields for user background information
    additionalFields: {
      softwareBackground: {
        type: "string",
        required: true,
      },
      aiExperience: {
        type: "string",
        required: true,
      },
      roboticsExperience: {
        type: "string",
        required: true,
      },
      programmingLanguages: {
        type: "string", // Store as JSON string
        required: false,
      },
      hardwareExperience: {
        type: "string",
        required: false,
      },
      learningStyle: {
        type: "string",
        required: true,
      },
      // Field to track progress
      progress: {
        type: "string", // Store as JSON string
        required: false,
      },
    },
  },
});

export default auth;