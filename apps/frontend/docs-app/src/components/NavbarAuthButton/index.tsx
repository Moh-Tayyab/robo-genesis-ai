import React, { useState, useEffect } from 'react';
import { createAuthClient } from 'better-auth/react';

// Determine auth service URL based on environment
const getAuthUrl = () => {
  if (typeof window === 'undefined') return 'http://localhost:3001';
  // Production: GitHub Pages points to Vercel auth service
  if (window.location.hostname.includes('github.io')) {
    return 'https://robo-genesis-ai.vercel.app';
  }
  // Local development
  return 'http://localhost:3001';
};

const AUTH_URL = getAuthUrl();
const SESSION_TOKEN_KEY = 'better_auth_session_token';

// Helper to get token from localStorage
const getStoredToken = (): string | null => {
  if (typeof window === 'undefined') return null;
  return localStorage.getItem(SESSION_TOKEN_KEY);
};

// Helper to store token in localStorage
const storeToken = (token: string): void => {
  if (typeof window === 'undefined') return;
  localStorage.setItem(SESSION_TOKEN_KEY, token);
};

// Helper to remove token from localStorage
const removeToken = (): void => {
  if (typeof window === 'undefined') return;
  localStorage.removeItem(SESSION_TOKEN_KEY);
};

// Create auth client with bearer token authentication for cross-domain
const authClient = createAuthClient({
  baseURL: AUTH_URL,
  fetchOptions: {
    credentials: 'include',
    auth: {
      type: 'Bearer',
      token: () => getStoredToken() || '',
    },
  },
});

interface AuthUser {
  id: string;
  email: string;
  name: string;
}

/**
 * NavbarAuthButton component for Docusaurus navbar
 *
 * Displays:
 * - "Login" button when unauthenticated
 * - User name + "Logout" button when authenticated
 *
 * Authentication flow:
 * 1. Check BetterAuth session on mount
 * 2. Redirect to auth service (:3001) for login with callback
 * 3. Handle logout and session cleanup
 */
export function NavbarAuthButton(): React.ReactElement {
  const [user, setUser] = useState<AuthUser | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    async function initAuth() {
      try {
        // Check for session token in URL (from redirect after login)
        if (typeof window !== 'undefined') {
          const urlParams = new URLSearchParams(window.location.search);
          const tokenFromUrl = urlParams.get('session_token');

          if (tokenFromUrl) {
            console.log('[NavbarAuth] Found session token in URL, storing...');
            storeToken(tokenFromUrl);

            // Clean URL by removing token parameter
            urlParams.delete('session_token');
            const cleanUrl = `${window.location.pathname}${urlParams.toString() ? '?' + urlParams.toString() : ''}`;
            window.history.replaceState({}, '', cleanUrl);
          }
        }

        // Fetch session using stored token
        const token = getStoredToken();
        if (!token) {
          console.log('[NavbarAuth] No token found, user not authenticated');
          setIsLoading(false);
          return;
        }

        console.log('[NavbarAuth] Fetching session from:', AUTH_URL);
        const session = await authClient.getSession();
        console.log('[NavbarAuth] Session response:', session);

        if (session.data?.user) {
          console.log('[NavbarAuth] User authenticated:', session.data.user.email);
          setUser({
            id: session.data.user.id,
            email: session.data.user.email,
            name: session.data.user.name || '',
          });
        } else {
          console.log('[NavbarAuth] Invalid session, clearing token');
          removeToken();
        }
      } catch (error) {
        console.error('[NavbarAuth] Failed to fetch session:', error);
        removeToken();
      } finally {
        setIsLoading(false);
      }
    }
    initAuth();
  }, []);

  const handleLogin = () => {
    const callbackUrl = encodeURIComponent(window.location.href);
    window.location.href = `${AUTH_URL}/signin?callbackUrl=${callbackUrl}`;
  };

  const handleLogout = async () => {
    try {
      // Clear local token
      removeToken();

      // Call signOut on server (optional, but recommended)
      await authClient.signOut();

      // Reload page to reset UI
      window.location.reload();
    } catch (error) {
      console.error('Logout error:', error);
      // Still clear token even if server call fails
      removeToken();
      window.location.reload();
    }
  };

  if (isLoading) {
    return <span style={{ opacity: 0.5 }}>...</span>;
  }

  if (user) {
    return (
      <div style={{ display: 'flex', alignItems: 'center', gap: '0.75rem' }}>
        <span style={{
          fontSize: '0.875rem',
          color: 'var(--ifm-navbar-link-color)',
          fontFamily: 'var(--chatkit-font-family, inherit)',
          fontWeight: 500
        }}>
          {user.name || user.email}
        </span>
        <button
          onClick={handleLogout}
          style={{
            background: 'rgba(255, 255, 255, 0.1)',
            border: '1px solid var(--ifm-color-emphasis-300)',
            borderRadius: '8px',
            padding: '0.375rem 0.75rem',
            cursor: 'pointer',
            color: 'var(--ifm-color-emphasis-900)',
            fontSize: '0.875rem',
            fontWeight: 500,
            fontFamily: 'var(--chatkit-font-family, inherit)',
            backdropFilter: 'blur(4px)',
            transition: 'all 0.2s ease',
          }}
          onMouseEnter={(e) => {
            const target = e.target as HTMLButtonElement;
            target.style.background = 'rgba(255, 255, 255, 0.2)';
            target.style.borderColor = '#00F2FF';
            target.style.color = '#00F2FF';
          }}
          onMouseLeave={(e) => {
            const target = e.target as HTMLButtonElement;
            target.style.background = 'rgba(255, 255, 255, 0.1)';
            target.style.borderColor = 'var(--ifm-color-emphasis-300)';
            target.style.color = 'var(--ifm-color-emphasis-900)';
          }}
        >
          Logout
        </button>
      </div>
    );
  }

  return (
    <button
      onClick={handleLogin}
      style={{
        background: 'linear-gradient(135deg, #00F2FF, #FF00C1)',
        border: 'none',
        borderRadius: '8px',
        padding: '0.5rem 1rem',
        cursor: 'pointer',
        color: '#0B0C10',
        fontSize: '0.875rem',
        fontWeight: 600,
        fontFamily: 'var(--chatkit-font-family, inherit)',
        boxShadow: '0 2px 8px rgba(0, 242, 255, 0.2)',
        transition: 'all 0.2s ease',
        backdropFilter: 'blur(4px)',
      }}
      onMouseEnter={(e) => {
        const target = e.target as HTMLButtonElement;
        target.style.transform = 'translateY(-1px)';
        target.style.boxShadow = '0 4px 12px rgba(0, 242, 255, 0.3)';
      }}
      onMouseLeave={(e) => {
        const target = e.target as HTMLButtonElement;
        target.style.transform = 'translateY(0)';
        target.style.boxShadow = '0 2px 8px rgba(0, 242, 255, 0.2)';
      }}
    >
      Login
    </button>
  );
}

export default NavbarAuthButton;
