const { neon } = require('@neondatabase/serverless');
const dotenv = require('dotenv');
const path = require('path');

// Load env
const envPath = path.resolve(__dirname, '../../apps/frontend/auth-app/.env.local');
dotenv.config({ path: envPath });

async function main() {
	if (!process.env.DATABASE_URL) {
		console.error('DATABASE_URL not found');
		return;
	}

	const url = process.env.DATABASE_URL.replace('postgresql+asyncpg://', 'postgresql://');
	console.log('Connecting with URL starting with:', url.substring(0, 15));

	const sql = neon(url);

	try {
		const result = await sql`SELECT 1`;
		console.log("Connection successful:", result);

		try {
			const users = await sql`SELECT count(*) FROM "user"`;
			console.log("User table exists. Count:", users);
		} catch (e) {
			console.error("User table check failed:", e.message);
		}
	} catch (err) {
		console.error("Connection failed:", err);
	}
}

main();
