import { defineConfig } from 'drizzle-kit';
import * as dotenv from 'dotenv';
import { resolve } from 'path';

// Load from apps/auth/.env first, then root .env as fallback
dotenv.config({ path: resolve(__dirname, '../../apps/frontend/auth-app/.env.local') });
dotenv.config({ path: resolve(__dirname, '../../apps/frontend/auth-app/.env') });
dotenv.config({ path: resolve(__dirname, '../../.env') });

export default defineConfig({
  schema: './schema/auth-schema.ts',
  out: './migrations',
  dialect: 'postgresql',
  dbCredentials: {
    url: process.env.DATABASE_URL!.replace('postgresql+asyncpg://', 'postgresql://'),
  },
  verbose: true,
  strict: true,
});
