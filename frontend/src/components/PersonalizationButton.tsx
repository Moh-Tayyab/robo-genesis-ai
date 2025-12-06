import React, { useState, useEffect } from 'react';
import { useAuth } from './AuthProvider';
import { Gear, MagicWand } from 'phosphor-react';

interface PersonalizationButtonProps {
  chapterId: string;
  chapterContent: string;
}

const PersonalizationButton: React.FC<PersonalizationButtonProps> = ({ chapterId, chapterContent }) => {
  const { user, loading } = useAuth();
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState('');
  const [error, setError] = useState<string | null>(null);

  const canPersonalize = user && user.profile_completed;

  const handlePersonalize = async () => {
    if (!user || !user.profile_completed) {
      setError('Please complete your profile to enable personalization');
      return;
    }

    setIsPersonalizing(true);
    setError(null);

    try {
      // In a real implementation, this would call an API to personalize the content
      // For now, we'll simulate the personalization
      const response = await fetch('/api/personalize', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          userId: user.id,
          chapterId,
          content: chapterContent,
          userBackground: {
            programming_experience: user.programming_experience,
            os_preference: user.os_preference,
            gpu_available: user.gpu_available,
            preferred_language: user.preferred_language,
            hardware_background: user.hardware_background,
            software_background: user.software_background,
          }
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to personalize content');
      }

      const data = await response.json();
      setPersonalizedContent(data.personalizedContent);
      setIsPersonalized(true);
    } catch (err) {
      console.error('Personalization error:', err);
      setError('Failed to personalize content. Please try again.');
    } finally {
      setIsPersonalizing(false);
    }
  };

  if (loading) {
    return (
      <div className="personalization-button">
        <button className="button button--secondary button--sm" disabled>
          Loading...
        </button>
      </div>
    );
  }

  if (!user) {
    return (
      <div className="personalization-button">
        <button
          className="button button--secondary button--sm"
          onClick={() => window.location.href = '/login'}
        >
          <Gear size={16} weight="bold" />
          <span>Login to Personalize</span>
        </button>
      </div>
    );
  }

  if (!canPersonalize) {
    return (
      <div className="personalization-button">
        <button
          className="button button--secondary button--sm"
          onClick={() => window.location.href = '/profile-setup'}
        >
          <Gear size={16} weight="bold" />
          <span>Complete Profile to Personalize</span>
        </button>
      </div>
    );
  }

  if (isPersonalized) {
    return (
      <div className="personalization-button">
        <button
          className="button button--primary button--sm"
          onClick={() => {
            // Reset to original content
            setPersonalizedContent('');
            setIsPersonalized(false);
          }}
        >
          <MagicWand size={16} weight="bold" />
          <span>Reset to Original</span>
        </button>
        {error && <div className="error">{error}</div>}
      </div>
    );
  }

  return (
    <div className="personalization-button">
      <button
        className={`button button--primary button--sm ${isPersonalizing ? 'button--loading' : ''}`}
        onClick={handlePersonalize}
        disabled={isPersonalizing}
      >
        <MagicWand size={16} weight="bold" />
        <span>{isPersonalizing ? 'Personalizing...' : 'Personalize Chapter'}</span>
      </button>
      {error && <div className="error">{error}</div>}
    </div>
  );
};

export default PersonalizationButton;