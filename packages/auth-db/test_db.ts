import { db } from './db';
import { sql } from 'drizzle-orm';

async function main() {
	console.log("Testing DB connection...");
	try {
		const result = await db.execute(sql`SELECT 1`);
		console.log("Connection successful:", result);

		console.log("Checking for user table...");
		try {
			const users = await db.execute(sql`SELECT count(*) FROM "user"`);
			console.log("User table exists. Count:", users);
		} catch (e) {
			console.error("User table check failed (likely missing):", e.message);
		}
	} catch (err) {
		console.error("Connection failed:", err);
	}
}

main();
