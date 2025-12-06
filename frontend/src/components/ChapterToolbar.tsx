import React, { useState, useEffect } from 'react';
import PersonalizationButton from './PersonalizationButton';
import UrduTranslationButton from './UrduTranslationButton';
import { useAuth } from './AuthProvider';

interface ChapterToolbarProps {
  chapterId: string;
  chapterContent: string;
}

const ChapterToolbar: React.FC<ChapterToolbarProps> = ({ chapterId, chapterContent }) => {
  const [currentContent, setCurrentContent] = useState(chapterContent);
  const { user } = useAuth();

  // Update content when chapterContent changes (in case of personalization/translation)
  useEffect(() => {
    setCurrentContent(chapterContent);
  }, [chapterContent]);

  const handleTranslationComplete = (translatedContent: string) => {
    setCurrentContent(translatedContent);
  };

  return (
    <div className="chapter-toolbar">
      <div className="toolbar-buttons">
        <PersonalizationButton
          chapterId={chapterId}
          chapterContent={currentContent}
        />
        <UrduTranslationButton
          content={currentContent}
          onTranslationComplete={handleTranslationComplete}
        />
      </div>

      {/* Apply the personalized/translated content to the chapter display */}
      <div className="chapter-content-display" style={{ display: 'none' }}>
        {currentContent}
      </div>
    </div>
  );
};

export default ChapterToolbar;