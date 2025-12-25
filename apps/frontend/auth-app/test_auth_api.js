
// Native fetch is used

async function testAuth() {
	const baseUrl = 'http://localhost:3001/api/auth';
	const testEmail = `test_${Date.now()}@example.com`;
	const testPassword = 'Password123!';
	const testName = 'Test User';

	console.log(`Testing auth against ${baseUrl}`);
	console.log(`Attempting sign-up for ${testEmail}...`);

	try {
		// 1. Check if server is up / get session (should be null)
		try {
			const sessionRes = await fetch(`${baseUrl}/get-session`);
			console.log('Session endpoint status:', sessionRes.status);
		} catch (e) {
			console.log('Server might not be ready yet, retrying in 2s...');
			await new Promise(r => setTimeout(r, 2000));
		}

		// 2. Sign Up
		const signUpRes = await fetch(`${baseUrl}/sign-up-email`, {
			method: 'POST',
			headers: { 'Content-Type': 'application/json' },
			body: JSON.stringify({
				email: testEmail,
				password: testPassword,
				name: testName
			})
		});

		const signUpText = await signUpRes.text();
		console.log('Sign Up Status:', signUpRes.status);
		console.log('Sign Up Response:', signUpText.substring(0, 200));

		if (signUpRes.ok) {
			console.log('✅ Sign Up Successful!');

			// 3. Sign In (Login)
			console.log('Attempting sign-in...');
			const signInRes = await fetch(`${baseUrl}/sign-in-email`, {
				method: 'POST',
				headers: { 'Content-Type': 'application/json' },
				body: JSON.stringify({
					email: testEmail,
					password: testPassword
				})
			});

			console.log('Sign In Status:', signInRes.status);
			if (signInRes.ok) {
				console.log('✅ Sign In Successful!');
			} else {
				console.error('❌ Sign In Failed');
			}

		} else {
			console.error('❌ Sign Up Failed');
		}

	} catch (error) {
		console.error('Test failed with error:', error);
	}
}

// Check if fetch is available (Node 18+)
if (!globalThis.fetch) {
	console.log("Native fetch not found, this script requires Node 18+");
} else {
	testAuth();
}
