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
  },
  secret: process.env.BETTER_AUTH_SECRET!,
  trustHost: true,
});