import React from 'react';
import { useAuth } from '@site/src/components/AuthProvider';

const UserNavbarItem: React.FC = () => {
  const { user, loading, logout } = useAuth();

  if (loading) {
    return (
      <div className="navbar__item navbar__link">
        Loading...
      </div>
    );
  }

  if (user) {
    return (
      <div className="navbar__item dropdown dropdown--right dropdown--username">
        <span className="navbar__link">
          {user.name || user.email?.split('@')[0] || 'User'}
          <span className="dropdown__menu">
            <a className="dropdown__link" href="/profile">Profile</a>
            <a
              className="dropdown__link"
              href="#"
              onClick={(e) => {
                e.preventDefault();
                logout();
              }}
            >
              Logout
            </a>
          </span>
        </span>
      </div>
    );
  }

  return (
    <div className="navbar__item navbar__link">
      <a href="/login">Login</a>
    </div>
  );
};

export default UserNavbarItem;