import React, { useState } from 'react';
import { Translate, Language, RotateCounterClockwise } from 'phosphor-react';

interface UrduTranslationButtonProps {
  content: string;
  onTranslationComplete?: (translatedContent: string) => void;
}

const UrduTranslationButton: React.FC<UrduTranslationButtonProps> = ({ content, onTranslationComplete }) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [isTranslated, setIsTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState('');
  const [error, setError] = useState<string | null>(null);

  const handleTranslate = async () => {
    setIsTranslating(true);
    setError(null);

    try {
      // In a real implementation, this would call an API to translate to Urdu
      // For now, we'll simulate the translation
      const response = await fetch('/api/translate/urdu', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content,
          targetLanguage: 'ur',
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to translate content');
      }

      const data = await response.json();
      setTranslatedContent(data.translatedContent);
      setIsTranslated(true);

      if (onTranslationComplete) {
        onTranslationComplete(data.translatedContent);
      }
    } catch (err) {
      console.error('Translation error:', err);
      setError('Failed to translate content. Please try again.');
    } finally {
      setIsTranslating(false);
    }
  };

  const handleReset = () => {
    setTranslatedContent('');
    setIsTranslated(false);
    if (onTranslationComplete) {
      onTranslationComplete(content);
    }
  };

  return (
    <div className="translation-button">
      {!isTranslated ? (
        <button
          className={`button button--secondary button--sm ${isTranslating ? 'button--loading' : ''}`}
          onClick={handleTranslate}
          disabled={isTranslating}
        >
          <Language size={16} weight="bold" />
          <span>{isTranslating ? 'Translating to Urdu...' : 'Translate to Urdu'}</span>
        </button>
      ) : (
        <button
          className="button button--secondary button--sm"
          onClick={handleReset}
        >
          <RotateCounterClockwise size={16} weight="bold" />
          <span>Reset to English</span>
        </button>
      )}
      {error && <div className="error">{error}</div>}
    </div>
  );
};

export default UrduTranslationButton;