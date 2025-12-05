import React from 'react';
import { useAuth } from '../../hooks/useAuth';
import GlassCard from '../ui/GlassCard';

interface GitHubOAuthButtonProps {
  onAuthComplete?: () => void;
}

const GitHubOAuthButton: React.FC<GitHubOAuthButtonProps> = ({ onAuthComplete }) => {
  const { signIn } = useAuth();

  const handleGitHubSignIn = async () => {
    try {
      // In a real implementation, this would redirect to GitHub OAuth
      // For now, we'll simulate the process
      console.log('Redirecting to GitHub OAuth...');
      // In a real app, this would involve redirecting to the auth provider
      if (onAuthComplete) {
        onAuthComplete();
      }
    } catch (error) {
      console.error('GitHub OAuth error:', error);
    }
  };

  return (
    <button
      onClick={handleGitHubSignIn}
      className="w-full flex items-center justify-center gap-3 px-4 py-2 border border-gray-300 dark:border-gray-600 rounded-md shadow-sm text-sm font-medium text-gray-700 dark:text-gray-300 bg-white dark:bg-gray-700 hover:bg-gray-50 dark:hover:bg-gray-600 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
    >
      <svg className="h-5 w-5 text-gray-800 dark:text-gray-200" fill="currentColor" viewBox="0 0 24 24">
        <path fillRule="evenodd" d="M12 2C6.477 2 2 6.477 2 12c0 4.42 2.87 8.17 6.84 9.5.5.08.66-.23.66-.5v-1.69c-2.77.6-3.36-1.34-3.36-1.34-.46-1.16-1.11-1.47-1.11-1.47-.91-.62.07-.6.07-.6 1 .07 1.53 1.03 1.53 1.03.87 1.52 2.34 1.07 2.91.83.09-.65.35-1.09.63-1.34-2.22-.25-4.55-1.11-4.55-4.92 0-1.11.38-2 1.03-2.71-.1-.25-.45-1.29.1-2.64 0 0 .84-.27 2.75 1.02.79-.22 1.65-.33 2.5-.33.85 0 1.71.11 2.5.33 1.91-1.29 2.75-1.02 2.75-1.02.55 1.35.2 2.39.1 2.64.65.71 1.03 1.6 1.03 2.71 0 3.82-2.34 4.66-4.57 4.91.36.31.69.92.69 1.85V21c0 .27.16.59.67.5C19.14 20.16 22 16.42 22 12A10 10 0 0012 2z" clipRule="evenodd" />
      </svg>
      <span>Continue with GitHub</span>
    </button>
  );
};

export default GitHubOAuthButton;