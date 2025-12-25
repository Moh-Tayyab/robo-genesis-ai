const dotenv = require('dotenv');
const path = require('path');
const fs = require('fs');

const envPath = path.resolve(__dirname, '../../apps/frontend/auth-app/.env.local');
console.log('Loading from:', envPath);

if (fs.existsSync(envPath)) {
	console.log('File exists.');
	const result = dotenv.config({ path: envPath });
	if (result.error) {
		console.error('Error loading env:', result.error);
	} else {
		console.log('DATABASE_URL is set:', !!process.env.DATABASE_URL);
		if (process.env.DATABASE_URL) {
			console.log('DATABASE_URL length:', process.env.DATABASE_URL.length);
			console.log('Starts with postgres:', process.env.DATABASE_URL.startsWith('postgres'));
			console.log('Contains @:', process.env.DATABASE_URL.includes('@'));
			console.log('First 15 chars:', process.env.DATABASE_URL.substring(0, 15));
		}
	}
} else {
	console.error('File does not exist.');
}
