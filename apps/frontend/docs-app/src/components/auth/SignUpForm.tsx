import React, { useState } from 'react';
import { useAuth } from '../../hooks/useAuth';
import GlassCard from '../ui/GlassCard';
import MultiSelect from '../ui/MultiSelect';
import { VALID_SOFTWARE_SKILLS, VALID_HARDWARE_ACCESS, DEFAULT_PROFILE } from '../../models/user-profile';

interface SignUpFormProps {
  onSignUpComplete?: () => void;
}

const SignUpForm: React.FC<SignUpFormProps> = ({ onSignUpComplete }) => {
  const { signUp } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [softwareBackground, setSoftwareBackground] = useState<string[]>(DEFAULT_PROFILE.softwareBackground);
  const [hardwareAccess, setHardwareAccess] = useState<string[]>(DEFAULT_PROFILE.hardwareAccess);
  const [comfortLevel, setComfortLevel] = useState<number>(DEFAULT_PROFILE.comfortLevel);
  const [showProfileForm, setShowProfileForm] = useState(false);
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (password.length < 8) {
      setError('Password must be at least 8 characters');
      return;
    }

    setLoading(true);
    try {
      // In a real implementation, we would call the API to create the profile
      // For now, we'll just call the signUp function with profile data
      await signUp(email, password, {
        userId: '', // Will be set by the backend
        softwareBackground,
        hardwareAccess,
        comfortLevel,
        createdAt: new Date(),
        updatedAt: new Date(),
        isProfileComplete: true
      });

      if (onSignUpComplete) {
        onSignUpComplete();
      }
    } catch (err) {
      setError('Sign up failed. Please try again.');
      console.error('Sign up error:', err);
    } finally {
      setLoading(false);
    }
  };

  const handleSkipProfile = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    try {
      await signUp(email, password, {
        userId: '', // Will be set by the backend
        softwareBackground: DEFAULT_PROFILE.softwareBackground,
        hardwareAccess: DEFAULT_PROFILE.hardwareAccess,
        comfortLevel: DEFAULT_PROFILE.comfortLevel,
        createdAt: new Date(),
        updatedAt: new Date(),
        isProfileComplete: false
      });

      if (onSignUpComplete) {
        onSignUpComplete();
      }
    } catch (err) {
      setError('Sign up failed. Please try again.');
      console.error('Sign up error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <GlassCard className="w-full max-w-md">
      <div className="text-center mb-6">
        <h2 className="text-2xl font-bold text-gray-900 dark:text-white" id="signup-heading">
          Create Account
        </h2>
        <p className="text-gray-600 dark:text-gray-400 mt-2">
          Join the Physical AI & Humanoid Robotics community
        </p>
      </div>

      {error && (
        <div
          className="mb-4 p-3 bg-red-100 border border-red-400 text-red-700 rounded-md"
          role="alert"
          aria-live="assertive"
        >
          {error}
        </div>
      )}

      <form onSubmit={showProfileForm ? handleSubmit : handleSkipProfile} aria-labelledby="signup-heading">
        <div className="space-y-4">
          <div>
            <label htmlFor="email" className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
              Email
            </label>
            <input
              id="email"
              type="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 dark:border-gray-600 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500 dark:bg-gray-700 dark:text-white"
              placeholder="you@example.com"
              required
              aria-describedby={error ? "email-error" : undefined}
            />
          </div>

          <div>
            <label htmlFor="password" className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
              Password
            </label>
            <input
              id="password"
              type="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 dark:border-gray-600 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500 dark:bg-gray-700 dark:text-white"
              placeholder="••••••••"
              required
              aria-describedby="password-help"
            />
            <p id="password-help" className="mt-1 text-xs text-gray-500 dark:text-gray-400">
              Password must be at least 8 characters
            </p>
          </div>

          <div>
            <label htmlFor="confirmPassword" className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
              Confirm Password
            </label>
            <input
              id="confirmPassword"
              type="password"
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 dark:border-gray-600 rounded-md shadow-sm focus:outline-none focus:ring-blue-500 focus:border-blue-500 dark:bg-gray-700 dark:text-white"
              placeholder="••••••••"
              required
            />
          </div>

          {showProfileForm && (
            <div className="space-y-4 pt-4 border-t border-gray-200 dark:border-gray-700">
              <div>
                <MultiSelect
                  label="Software Background (Select all that apply)"
                  options={VALID_SOFTWARE_SKILLS}
                  selected={softwareBackground}
                  onChange={setSoftwareBackground}
                  placeholder="Select software skills..."
                />
              </div>

              <div>
                <MultiSelect
                  label="Hardware Access (Select all that apply)"
                  options={VALID_HARDWARE_ACCESS}
                  selected={hardwareAccess}
                  onChange={setHardwareAccess}
                  placeholder="Select hardware access..."
                />
              </div>

              <div>
                <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
                  Comfort Level (1 = Beginner, 5 = Expert)
                </label>
                <div className="flex items-center space-x-2" role="radiogroup" aria-label="Comfort level selection">
                  {[1, 2, 3, 4, 5].map((level) => (
                    <button
                      key={level}
                      type="button"
                      role="radio"
                      aria-checked={comfortLevel === level}
                      className={`w-10 h-10 rounded-full border ${
                        comfortLevel === level
                          ? 'bg-blue-600 text-white border-blue-600 focus:ring-2 focus:ring-blue-500 focus:ring-offset-2'
                          : 'border-gray-300 dark:border-gray-600 text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-gray-700 focus:ring-2 focus:ring-blue-500 focus:ring-offset-2'
                      }`}
                      onClick={() => setComfortLevel(level)}
                      tabIndex={0}
                      onKeyDown={(e) => {
                        if (e.key === 'Enter' || e.key === ' ') {
                          e.preventDefault();
                          setComfortLevel(level);
                        }
                      }}
                    >
                      {level}
                    </button>
                  ))}
                </div>
                <div className="flex justify-between text-xs text-gray-500 dark:text-gray-400 mt-1">
                  <span>Beginner</span>
                  <span>Expert</span>
                </div>
              </div>
            </div>
          )}
        </div>

        <div className="mt-6 space-y-3">
          {!showProfileForm && (
            <button
              type="button"
              onClick={() => setShowProfileForm(true)}
              className="w-full flex justify-center py-2 px-4 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500"
              aria-label="Show profile questions"
            >
              Continue with Profile
            </button>
          )}

          {showProfileForm && (
            <button
              type="submit"
              disabled={loading}
              className="w-full flex justify-center py-2 px-4 border border-transparent rounded-md shadow-sm text-sm font-medium text-white bg-blue-600 hover:bg-blue-700 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500 disabled:opacity-50"
            >
              {loading ? 'Creating Account...' : 'Create Account'}
            </button>
          )}

          {!showProfileForm && (
            <button
              type="submit"
              disabled={loading}
              className="w-full flex justify-center py-2 px-4 border border-gray-300 dark:border-gray-600 rounded-md shadow-sm text-sm font-medium text-gray-700 dark:text-gray-300 bg-white dark:bg-gray-700 hover:bg-gray-50 dark:hover:bg-gray-600 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500 disabled:opacity-50"
            >
              {loading ? 'Creating Account...' : 'Skip Profile & Continue'}
            </button>
          )}
        </div>
      </form>

      <div className="mt-4 text-center">
        <p className="text-sm text-gray-600 dark:text-gray-400">
          Already have an account?{' '}
          <a href="/signin" className="font-medium text-blue-600 hover:text-blue-500 dark:text-blue-400 dark:hover:text-blue-300 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-blue-500">
            Sign in
          </a>
        </p>
      </div>
    </GlassCard>
  );
};

export default SignUpForm;