import React, { useState } from 'react';
import { useAuth } from './AuthProvider';

interface FormData {
  email: string;
  password: string;
  confirmPassword: string;
  name: string;
}

interface BackgroundFormData {
  programming_experience: string;
  os_preference: string;
  gpu_available: boolean | null;
  preferred_language: string;
  learning_goals: string;
  hardware_background: string;
  software_background: string;
}

const SignupForm: React.FC = () => {
  const [step, setStep] = useState<'account' | 'background'>('account');
  const [formData, setFormData] = useState<FormData>({
    email: '',
    password: '',
    confirmPassword: '',
    name: ''
  });
  const [backgroundData, setBackgroundData] = useState<BackgroundFormData>({
    programming_experience: '',
    os_preference: '',
    gpu_available: null,
    preferred_language: '',
    learning_goals: '',
    hardware_background: '',
    software_background: ''
  });
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  const { signup } = useAuth();

  const handleAccountSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    setLoading(true);
    try {
      await signup(formData.email, formData.password, formData.name);
      setStep('background');
    } catch (err) {
      setError('Signup failed. Please try again.');
      console.error('Signup error:', err);
    } finally {
      setLoading(false);
    }
  };

  const handleBackgroundSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError(null);

    setLoading(true);
    try {
      // In a real implementation, we would update the user profile here
      // For now, we'll just redirect to the home page
      window.location.href = '/';
    } catch (err) {
      setError('Failed to save background information. Please try again.');
      console.error('Background update error:', err);
    } finally {
      setLoading(false);
    }
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement | HTMLSelectElement>) => {
    const { name, value, type } = e.target;
    if (type === 'checkbox') {
      const target = e.target as HTMLInputElement;
      setBackgroundData(prev => ({
        ...prev,
        [name]: target.checked
      }));
    } else {
      setFormData(prev => ({
        ...prev,
        [name]: value
      }));
    }
  };

  const handleBackgroundChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement | HTMLSelectElement>) => {
    const { name, value, type } = e.target;
    if (type === 'checkbox') {
      const target = e.target as HTMLInputElement;
      setBackgroundData(prev => ({
        ...prev,
        [name]: target.checked
      }));
    } else if (type === 'radio') {
      const target = e.target as HTMLInputElement;
      setBackgroundData(prev => ({
        ...prev,
        [name]: target.value
      }));
    } else {
      setBackgroundData(prev => ({
        ...prev,
        [name]: value
      }));
    }
  };

  if (step === 'account') {
    return (
      <div className="signup-form">
        <h2>Create Account</h2>
        {error && <div className="error">{error}</div>}

        <form onSubmit={handleAccountSubmit}>
          <div className="form-group">
            <label htmlFor="name">Full Name</label>
            <input
              type="text"
              id="name"
              name="name"
              value={formData.name}
              onChange={handleInputChange}
              required
            />
          </div>

          <div className="form-group">
            <label htmlFor="email">Email</label>
            <input
              type="email"
              id="email"
              name="email"
              value={formData.email}
              onChange={handleInputChange}
              required
            />
          </div>

          <div className="form-group">
            <label htmlFor="password">Password</label>
            <input
              type="password"
              id="password"
              name="password"
              value={formData.password}
              onChange={handleInputChange}
              required
            />
          </div>

          <div className="form-group">
            <label htmlFor="confirmPassword">Confirm Password</label>
            <input
              type="password"
              id="confirmPassword"
              name="confirmPassword"
              value={formData.confirmPassword}
              onChange={handleInputChange}
              required
            />
          </div>

          <button type="submit" disabled={loading}>
            {loading ? 'Creating Account...' : 'Create Account'}
          </button>
        </form>

        <p>
          Already have an account? <a href="/login">Sign in</a>
        </p>
      </div>
    );
  }

  return (
    <div className="signup-form">
      <h2>Tell Us About Your Background</h2>
      <p className="subtitle">This information will help us personalize your learning experience.</p>
      {error && <div className="error">{error}</div>}

      <form onSubmit={handleBackgroundSubmit}>
        <div className="form-section">
          <h3>Technical Background</h3>

          <div className="form-group">
            <label>Programming Experience</label>
            <div className="radio-group">
              {['beginner', 'intermediate', 'advanced', 'expert'].map(level => (
                <label key={level} className="radio-option">
                  <input
                    type="radio"
                    name="programming_experience"
                    value={level}
                    checked={backgroundData.programming_experience === level}
                    onChange={handleBackgroundChange}
                  />
                  {level.charAt(0).toUpperCase() + level.slice(1)}
                </label>
              ))}
            </div>
          </div>

          <div className="form-group">
            <label>Operating System Preference</label>
            <div className="radio-group">
              {['windows', 'macos', 'linux'].map(os => (
                <label key={os} className="radio-option">
                  <input
                    type="radio"
                    name="os_preference"
                    value={os}
                    checked={backgroundData.os_preference === os}
                    onChange={handleBackgroundChange}
                  />
                  {os.charAt(0).toUpperCase() + os.slice(1)}
                </label>
              ))}
            </div>
          </div>

          <div className="form-group">
            <label>Do you have access to a GPU?</label>
            <div className="radio-group">
              <label className="radio-option">
                <input
                  type="radio"
                  name="gpu_available"
                  value="true"
                  checked={backgroundData.gpu_available === true}
                  onChange={handleBackgroundChange}
                />
                Yes
              </label>
              <label className="radio-option">
                <input
                  type="radio"
                  name="gpu_available"
                  value="false"
                  checked={backgroundData.gpu_available === false}
                  onChange={handleBackgroundChange}
                />
                No
              </label>
            </div>
          </div>

          <div className="form-group">
            <label htmlFor="preferred_language">Preferred Programming Language</label>
            <select
              id="preferred_language"
              name="preferred_language"
              value={backgroundData.preferred_language}
              onChange={handleBackgroundChange}
            >
              <option value="">Select a language</option>
              <option value="python">Python</option>
              <option value="typescript">TypeScript</option>
              <option value="javascript">JavaScript</option>
              <option value="cpp">C++</option>
              <option value="rust">Rust</option>
              <option value="other">Other</option>
            </select>
          </div>
        </div>

        <div className="form-section">
          <h3>Professional Background</h3>

          <div className="form-group">
            <label>Hardware Background</label>
            <div className="radio-group">
              {['none', 'maker', 'engineer', 'researcher'].map(level => (
                <label key={level} className="radio-option">
                  <input
                    type="radio"
                    name="hardware_background"
                    value={level}
                    checked={backgroundData.hardware_background === level}
                    onChange={handleBackgroundChange}
                  />
                  {level.charAt(0).toUpperCase() + level.slice(1)}
                </label>
              ))}
            </div>
          </div>

          <div className="form-group">
            <label>Software Background</label>
            <div className="radio-group">
              {['none', 'developer', 'engineer', 'researcher'].map(level => (
                <label key={level} className="radio-option">
                  <input
                    type="radio"
                    name="software_background"
                    value={level}
                    checked={backgroundData.software_background === level}
                    onChange={handleBackgroundChange}
                  />
                  {level.charAt(0).toUpperCase() + level.slice(1)}
                </label>
              ))}
            </div>
          </div>
        </div>

        <div className="form-section">
          <h3>Learning Goals</h3>

          <div className="form-group">
            <label htmlFor="learning_goals">What are your learning goals for this course?</label>
            <textarea
              id="learning_goals"
              name="learning_goals"
              value={backgroundData.learning_goals}
              onChange={handleBackgroundChange}
              rows={4}
              placeholder="Tell us what you hope to achieve by taking this course..."
            />
          </div>
        </div>

        <button type="submit" disabled={loading}>
          {loading ? 'Saving Information...' : 'Complete Setup'}
        </button>
      </form>
    </div>
  );
};

export default SignupForm;