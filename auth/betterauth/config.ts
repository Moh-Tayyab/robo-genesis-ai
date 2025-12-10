import { betterAuth } from "better-auth";
import { postgresAdapter } from "@better-auth/postgres-adapter";
import { neon } from "@neondatabase/serverless";

// Initialize Neon database client
const db = neon(process.env.DATABASE_URL!);

export const auth = betterAuth({
  database: postgresAdapter(db, {
    provider: "pg",
  }),
  socialProviders: {
    github: {
      clientId: process.env.GITHUB_CLIENT_ID!,
      clientSecret: process.env.GITHUB_CLIENT_SECRET!,
    },
  },
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false,
    sendEmailVerificationOnSignUp: true,
  },
  user: {
    // Add custom fields for user background information
    fields: {
      programming_experience: {
        type: "string",
        required: false,
      },
      os_preference: {
        type: "string",
        required: false,
      },
      gpu_available: {
        type: "boolean",
        required: false,
      },
      preferred_language: {
        type: "string",
        required: false,
      },
      learning_goals: {
        type: "string", // text field
        required: false,
      },
      hardware_background: {
        type: "string",
        required: false,
      },
      software_background: {
        type: "string",
        required: false,
      },
      profile_completed: {
        type: "boolean",
        required: false,
        default: false,
      }
    }
  },
  // Hooks to handle custom logic
  hooks: {
    createUser: {
      // This hook will be called after a user is created
      // We can initialize default values or perform additional setup
      async created(user) {
        console.log("New user created:", user.email);
        // Additional logic can be added here if needed
      }
    }
  },
  secret: process.env.BETTER_AUTH_SECRET!,
  trustHost: true,
});