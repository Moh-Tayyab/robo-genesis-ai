import React from 'react';
import Chatbot from '@site/src/components/Chatbot';

// Root component wrapper - renders on every page
export default function Root({ children }: { children: React.ReactNode }) {
	return (
		<>
			{children}
			<Chatbot />
		</>
	);
}
