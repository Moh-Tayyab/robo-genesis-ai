import React from 'react';
import Head from '@docusaurus/Head';
import SignUpForm from '../components/auth/SignUpForm';
import GitHubOAuthButton from '../components/auth/GitHubOAuthButton';
import { AuthProvider } from '../contexts/AuthContext';
import AuthLayout from '../components/auth/AuthLayout';

const SignUpPage: React.FC = () => {
  const handleSignUpComplete = () => {
    // Redirect to welcome page after successful sign up
    window.location.href = '/welcome';
  };

  return (
    <AuthProvider>
      <AuthLayout>
        <Head>
          <title>Sign Up | Physical AI & Humanoid Robotics</title>
          <meta name="description" content="Create your account to access the Physical AI & Humanoid Robotics textbook" />
        </Head>

        <div className="min-h-screen flex items-center justify-center py-12 px-4 sm:px-6 lg:px-8">
          <div className="max-w-md w-full space-y-8">
            <div className="text-center">
              <h1 className="text-3xl font-extrabold text-gray-900 dark:text-white">
                Start Learning Physical AI
              </h1>
              <p className="mt-2 text-gray-600 dark:text-gray-400">
                Join our community of robotics practitioners
              </p>
            </div>

            <div className="mt-8 space-y-6">
              <GitHubOAuthButton onAuthComplete={handleSignUpComplete} />

              <div className="relative">
                <div className="absolute inset-0 flex items-center">
                  <div className="w-full border-t border-gray-300 dark:border-gray-700" />
                </div>
                <div className="relative flex justify-center text-sm">
                  <span className="px-2 bg-white dark:bg-gray-900 text-gray-500 dark:text-gray-400">
                    Or continue with email
                  </span>
                </div>
              </div>

              <SignUpForm onSignUpComplete={handleSignUpComplete} />
            </div>
          </div>
        </div>
      </AuthLayout>
    </AuthProvider>
  );
};

export default SignUpPage;