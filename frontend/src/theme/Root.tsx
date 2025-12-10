import React, { useEffect } from 'react';
import Chatbot from '@site/src/components/Chatbot';
import { AuthProvider } from '@site/src/components/AuthProvider';
import NavbarUpdater from '@site/src/components/NavbarUpdater';

// Root component wrapper - renders on every page
export default function Root({ children }: { children: React.ReactNode }) {
  // Add text selection functionality
  useEffect(() => {
    const handleMouseUp = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText && selectedText.length > 0) {
        // Store the selected text in sessionStorage for the chatbot to access
        sessionStorage.setItem('selectedText', selectedText);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  return (
    <AuthProvider>
      <NavbarUpdater />
      {children}
      <Chatbot />
    </AuthProvider>
  );
}
