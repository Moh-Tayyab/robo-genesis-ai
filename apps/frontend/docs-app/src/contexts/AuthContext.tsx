import React, { createContext, useState, useEffect, ReactNode } from 'react';
import { UserProfile } from '../models/user-profile';

interface AuthContextType {
  user: any | null; // Better-Auth user object
  userProfile: UserProfile | null;
  loading: boolean;
  signIn: (email: string, password: string) => Promise<void>;
  signOut: () => Promise<void>;
  signUp: (email: string, password: string, profile?: UserProfile) => Promise<void>;
  updateProfile: (profile: UserProfile) => Promise<void>;
  isAuthenticated: boolean;
}

export const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<any | null>(null);
  const [userProfile, setUserProfile] = useState<UserProfile | null>(null);
  const [loading, setLoading] = useState(true);

  // Check authentication status on component mount
  useEffect(() => {
    const checkAuthStatus = async () => {
      try {
        // In a real implementation, we would check the auth status with an API call
        // For now, we'll just simulate the loading process
        setLoading(false);
      } catch (error) {
        console.error('Error checking auth status:', error);
        setLoading(false);
      }
    };

    checkAuthStatus();
  }, []);

  const signIn = async (email: string, password: string) => {
    // In a real implementation, this would call the Better-Auth API
    console.log(`Signing in user: ${email}`);
    // Mock implementation - in reality, this would make an API call to authenticate
    setUser({ id: 'mock-user-id', email });
    setLoading(false);
  };

  const signOut = async () => {
    // In a real implementation, this would call the Better-Auth API to sign out
    console.log('Signing out user');
    setUser(null);
    setUserProfile(null);
  };

  const signUp = async (email: string, password: string, profile?: UserProfile) => {
    // In a real implementation, this would call the Better-Auth API to sign up
    console.log(`Signing up user: ${email}`);
    // Mock implementation - in reality, this would make an API call to register
    setUser({ id: 'mock-user-id', email });
    if (profile) {
      setUserProfile(profile);
    }
    setLoading(false);
  };

  const updateProfile = async (profile: UserProfile) => {
    // In a real implementation, this would call the API to update the user profile
    console.log('Updating user profile:', profile);
    setUserProfile(profile);
  };

  const isAuthenticated = !!user;

  const value = {
    user,
    userProfile,
    loading,
    signIn,
    signOut,
    signUp,
    updateProfile,
    isAuthenticated
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};