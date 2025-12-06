import React, { useEffect } from 'react';
import { useAuth } from './AuthProvider';

const NavbarUpdater: React.FC = () => {
  const { user, loading, logout } = useAuth();

  useEffect(() => {
    // This effect will run when the auth state changes
    const updateNavbar = () => {
      // Find and update navbar items based on auth status
      const loginLink = document.querySelector('a[href="/login"]') as HTMLAnchorElement;
      const signupLink = document.querySelector('a[href="/signup"]') as HTMLAnchorElement;

      if (loginLink && signupLink) {
        if (user) {
          // User is logged in - hide login/signup, show profile/logout
          loginLink.textContent = user.name || user.email?.split('@')[0] || 'Profile';
          loginLink.href = '/profile';

          signupLink.textContent = 'Logout';
          signupLink.href = '#';
          signupLink.onclick = (e) => {
            e.preventDefault();
            logout();
          };
        } else {
          // User is not logged in - show login/signup
          loginLink.textContent = 'Login';
          loginLink.href = '/login';

          signupLink.textContent = 'Sign Up';
          signupLink.href = '/signup';
        }
      }
    };

    // Run immediately and whenever user/loading changes
    updateNavbar();
  }, [user, loading, logout]);

  return null; // This component doesn't render anything
};

export default NavbarUpdater;