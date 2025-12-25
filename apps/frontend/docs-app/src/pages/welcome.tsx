import React from 'react';
import Head from '@docusaurus/Head';
import { useAuth } from '../hooks/useAuth';
import GlassCard from '../components/ui/GlassCard';
import AuthLayout from '../components/auth/AuthLayout';

const WelcomePage: React.FC = () => {
  const { user, userProfile } = useAuth();

  // Function to get a personalized greeting based on comfort level
  const getGreetingMessage = () => {
    if (!userProfile) return "Welcome to the Physical AI & Humanoid Robotics course!";

    switch (userProfile.comfortLevel) {
      case 1:
        return "Welcome! We're excited to start your journey in Physical AI. We'll take it one step at a time.";
      case 2:
        return "Welcome! It's great to have you here. We'll help you build your skills progressively.";
      case 3:
        return "Welcome! We're glad you're here to deepen your understanding of Physical AI.";
      case 4:
        return "Welcome! We're excited to share advanced concepts with an experienced practitioner like you.";
      case 5:
        return "Welcome! We're thrilled to have an expert like you joining our community.";
      default:
        return "Welcome to the Physical AI & Humanoid Robotics course!";
    }
  };

  return (
    <AuthLayout>
      <Head>
        <title>Welcome | Physical AI & Humanoid Robotics</title>
        <meta name="description" content="Welcome to the Physical AI & Humanoid Robotics textbook" />
      </Head>

      <div className="min-h-screen flex items-center justify-center py-12 px-4 sm:px-6 lg:px-8">
        <GlassCard className="w-full max-w-2xl text-center">
          <div className="space-y-6">
            <div className="mx-auto bg-gradient-to-r from-blue-500 to-purple-600 p-1 rounded-full w-24 h-24 flex items-center justify-center">
              <div className="bg-white dark:bg-gray-900 rounded-full w-22 h-22 flex items-center justify-center">
                <svg xmlns="http://www.w3.org/2000/svg" className="h-12 w-12 text-blue-600" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 13l4 4L19 7" />
                </svg>
              </div>
            </div>

            <h1 className="text-3xl font-extrabold text-gray-900 dark:text-white">
              Welcome, {user?.email || 'Learner'}!
            </h1>

            <p className="text-lg text-gray-600 dark:text-gray-300 max-w-lg mx-auto">
              {getGreetingMessage()}
            </p>

            <div className="mt-8 bg-blue-50 dark:bg-blue-900/30 rounded-lg p-6 text-left">
              <h2 className="text-xl font-semibold text-gray-900 dark:text-white mb-4">Your Profile Summary</h2>
              <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
                <div className="bg-white dark:bg-gray-800 p-4 rounded-lg shadow-sm">
                  <h3 className="font-medium text-gray-700 dark:text-gray-300">Software Skills</h3>
                  <p className="text-gray-900 dark:text-white mt-1">
                    {userProfile?.softwareBackground?.join(', ') || 'Not specified'}
                  </p>
                </div>
                <div className="bg-white dark:bg-gray-800 p-4 rounded-lg shadow-sm">
                  <h3 className="font-medium text-gray-700 dark:text-gray-300">Hardware Access</h3>
                  <p className="text-gray-900 dark:text-white mt-1">
                    {userProfile?.hardwareAccess?.join(', ') || 'Not specified'}
                  </p>
                </div>
                <div className="bg-white dark:bg-gray-800 p-4 rounded-lg shadow-sm">
                  <h3 className="font-medium text-gray-700 dark:text-gray-300">Comfort Level</h3>
                  <p className="text-gray-900 dark:text-white mt-1">
                    {userProfile?.comfortLevel || 'Not specified'}
                    <span className="text-sm text-gray-500 dark:text-gray-400">/5</span>
                  </p>
                </div>
              </div>
            </div>

            <div className="mt-8">
              <p className="text-gray-600 dark:text-gray-400 mb-6">
                Based on your profile, we'll personalize your learning experience.
              </p>

              <a
                href="/"
                className="inline-block px-6 py-3 border border-transparent text-base font-medium rounded-md text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
              >
                Start Learning
              </a>
            </div>
          </div>
        </GlassCard>
      </div>
    </AuthLayout>
  );
};

export default WelcomePage;