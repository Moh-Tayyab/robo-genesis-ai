import { drizzle } from 'drizzle-orm/neon-http';
import { neon } from '@neondatabase/serverless';
import * as schema from './schema/auth-schema';

// Create a lazy initialization for the neon client to avoid build-time connection attempts
let cachedClient: ReturnType<typeof neon> | null = null;
let cachedDb: ReturnType<typeof drizzle> | null = null;

// Only initialize the database client when it's actually needed (not during build)
const getNeonClient = () => {
  if (!cachedClient) {
    // Check if we're in a build phase where we shouldn't initialize the database
    const isBuildPhase = process.env.NEXT_PHASE === 'phase-production-build';

    if (isBuildPhase) {
      console.warn('Skipping database client initialization during build phase');
      // Return a mock client that won't be used during build
      return null as any;
    }

    if (!process.env.DATABASE_URL) {
      throw new Error('DATABASE_URL is not set! Check your .env file.');
    }

    cachedClient = neon(process.env.DATABASE_URL.replace('postgresql+asyncpg://', 'postgresql://'));
  }
  return cachedClient;
};

export const db = (() => {
  // Check if we're in build phase
  const isBuildPhase = process.env.NEXT_PHASE === 'phase-production-build';

  if (isBuildPhase) {
    // Return a mock database object during build phase
    console.warn('Using mock database during build phase');
    return {
      query: {} as any,
      select: () => ({} as any),
      insert: () => ({} as any),
      update: () => ({} as any),
      delete: () => ({} as any),
    } as any;
  }

  // Initialize the real database for runtime
  if (!cachedDb) {
    const sql = getNeonClient();
    cachedDb = drizzle(sql, { schema });
  }
  return cachedDb;
})();
