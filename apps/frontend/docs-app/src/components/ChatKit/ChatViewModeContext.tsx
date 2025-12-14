/**
 * ChatViewModeContext
 *
 * Provides view mode state management for ChatKit with three modes:
 * - compact: Narrow bar at bottom (~400px width)
 * - fullpage: Centered modal with backdrop blur
 * - sidedock-left: Fixed panel on left side (~350px)
 * - sidedock-right: Fixed panel on right side (~350px)
 *
 * Includes localStorage persistence for user preferences.
 */

import React, { createContext, useContext, useState, useEffect, useCallback, useMemo } from 'react';

// ============================================================================
// Types
// ============================================================================

/**
 * Available chat view modes
 */
export type ChatViewMode = 'compact' | 'fullpage';


/**
 * Context value shape
 */
export interface ChatViewModeContextValue {
  /** Current view mode */
  mode: ChatViewMode;
  /** Whether the chat panel is open (compact mode can be open/closed) */
  isOpen: boolean;
  /** Whether the component is mounted (client-side) */
  isMounted: boolean;
  /** Set the view mode */
  setMode: (mode: ChatViewMode) => void;
  /** Toggle chat open/closed */
  toggleOpen: () => void;
  /** Open chat in a specific mode */
  openInMode: (mode: ChatViewMode) => void;
  /** Minimize chat to compact mode (closed) */
  minimize: () => void;
  /** Expand to fullpage mode */
  expandFullpage: () => void;
  /** Check if current mode is fullpage */
  isFullpage: boolean;
  /** Check if current mode is compact */
  isCompact: boolean;
}

// ============================================================================
// Constants
// ============================================================================

const STORAGE_KEY = 'chatkit-view-mode';
const DEFAULT_MODE: ChatViewMode = 'compact';

// ============================================================================
// Context
// ============================================================================

const ChatViewModeContext = createContext<ChatViewModeContextValue | null>(null);

// ============================================================================
// Provider
// ============================================================================

export interface ChatViewModeProviderProps {
  children: React.ReactNode;
  /** Initial mode (overrides localStorage on first render) */
  initialMode?: ChatViewMode;
  /** Disable localStorage persistence */
  disablePersistence?: boolean;
}

/**
 * Provider component for ChatViewMode context
 */
export function ChatViewModeProvider({
  children,
  initialMode,
  disablePersistence = false,
}: ChatViewModeProviderProps): React.ReactElement {
  // Track if component is mounted (client-side)
  const [isMounted, setIsMounted] = useState(false);

  // Initialize state - always start with default to avoid hydration mismatch
  const [mode, setModeState] = useState<ChatViewMode>(initialMode || DEFAULT_MODE);
  const [isOpen, setIsOpen] = useState(false);

  // Load from localStorage after mount (client-side only)
  useEffect(() => {
    setIsMounted(true);

    if (initialMode || disablePersistence) return;

    try {
      const stored = localStorage.getItem(STORAGE_KEY);
      if (stored && isValidMode(stored)) {
        setModeState(stored as ChatViewMode);
      }
    } catch {
      // localStorage not available
    }
  }, [initialMode, disablePersistence]);

  // Persist mode to localStorage
  useEffect(() => {
    if (disablePersistence || typeof window === 'undefined') return;

    try {
      localStorage.setItem(STORAGE_KEY, mode);
    } catch {
      // localStorage not available
    }
  }, [mode, disablePersistence]);

  // Handle mobile viewport - no sidedock modes to consider now
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const handleResize = () => {
      // No sidedock modes to handle anymore
    };

    // Check on mount
    handleResize();

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, [mode]);

  // No body class needed since we don't have sidedock modes anymore
  useEffect(() => {
    if (typeof document === 'undefined') return;

    // Remove all chatkit body classes first
    document.body.classList.remove(
      'chatkit-sidedock-left-active',
      'chatkit-sidedock-right-active'
    );

    // No sidedock classes to add anymore

    return () => {
      document.body.classList.remove(
        'chatkit-sidedock-left-active',
        'chatkit-sidedock-right-active'
      );
    };
  }, []);

  // -------------------------------------------------------------------------
  // Actions
  // -------------------------------------------------------------------------

  const setMode = useCallback((newMode: ChatViewMode) => {
    setModeState(newMode);
    // Opening in a non-compact mode should also open the panel
    if (newMode !== 'compact') {
      setIsOpen(true);
    }
  }, []);

  const toggleOpen = useCallback(() => {
    setIsOpen(prev => !prev);
  }, []);

  const openInMode = useCallback((newMode: ChatViewMode) => {
    setModeState(newMode);
    setIsOpen(true);
  }, []);

  const minimize = useCallback(() => {
    setModeState('compact');
    setIsOpen(false);
  }, []);

  const expandFullpage = useCallback(() => {
    if (mode === 'fullpage') {
      // If already in fullpage mode, minimize to compact
      setModeState('compact');
      setIsOpen(false);
    } else {
      // Otherwise, expand to fullpage
      setModeState('fullpage');
      setIsOpen(true);
    }
  }, [mode]);


  // -------------------------------------------------------------------------
  // Computed values
  // -------------------------------------------------------------------------

  const isFullpage = mode === 'fullpage';
  const isCompact = mode === 'compact';

  // -------------------------------------------------------------------------
  // Context value
  // -------------------------------------------------------------------------

  const value = useMemo<ChatViewModeContextValue>(
    () => ({
      mode,
      isOpen,
      isMounted,
      setMode,
      toggleOpen,
      openInMode,
      minimize,
      expandFullpage,
      isFullpage,
      isCompact,
    }),
    [mode, isOpen, isMounted, setMode, toggleOpen, openInMode, minimize, expandFullpage, isFullpage, isCompact]
  );

  return (
    <ChatViewModeContext.Provider value={value}>
      {children}
    </ChatViewModeContext.Provider>
  );
}

// ============================================================================
// Hook
// ============================================================================

/**
 * Hook to access ChatViewMode context
 *
 * @throws Error if used outside of ChatViewModeProvider
 */
export function useChatViewMode(): ChatViewModeContextValue {
  const context = useContext(ChatViewModeContext);

  if (!context) {
    throw new Error('useChatViewMode must be used within a ChatViewModeProvider');
  }

  return context;
}

// ============================================================================
// Utilities
// ============================================================================

/**
 * Type guard to validate a mode string
 */
function isValidMode(value: string): value is ChatViewMode {
  return ['compact', 'fullpage'].includes(value);
}

/**
 * Get CSS class name for a view mode
 */
export function getViewModeClassName(mode: ChatViewMode): string {
  return `chatkit-${mode}`;
}

/**
 * Check if view mode allows showing dock buttons (not on mobile)
 */
export function canShowDockButtons(): boolean {
  if (typeof window === 'undefined') return true;
  return window.innerWidth >= 768;
}

export default ChatViewModeContext;
