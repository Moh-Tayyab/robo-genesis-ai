import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { auth, useSession } from '../auth-client';

// Define user types
interface UserBackground {
  programming_experience?: string;
  os_preference?: string;
  gpu_available?: boolean;
  preferred_language?: string;
  learning_goals?: string;
  hardware_background?: string;
  software_background?: string;
  profile_completed?: boolean;
}

interface User extends UserBackground {
  id: string;
  email: string;
  name?: string;
}

interface AuthContextType {
  user: User | null;
  loading: boolean;
  login: (email: string, password: string) => Promise<void>;
  signup: (email: string, password: string, name?: string) => Promise<void>;
  logout: () => Promise<void>;
  updateUserBackground: (background: UserBackground) => Promise<void>;
  fetchUserBackground: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [loading, setLoading] = useState(true);
  const { session, refetch } = useSession();

  useEffect(() => {
    if (session?.user) {
      setUser({
        id: session.user.id,
        email: session.user.email || '',
        name: session.user.name,
        programming_experience: session.user.programming_experience,
        os_preference: session.user.os_preference,
        gpu_available: session.user.gpu_available,
        preferred_language: session.user.preferred_language,
        learning_goals: session.user.learning_goals,
        hardware_background: session.user.hardware_background,
        software_background: session.user.software_background,
        profile_completed: session.user.profile_completed,
      });
    } else {
      setUser(null);
    }
    setLoading(false);
  }, [session]);

  const login = async (email: string, password: string) => {
    setLoading(true);
    try {
      const response = await auth.signIn.email({
        email,
        password,
        callbackURL: "/",
      });
      if (!response) {
        throw new Error("Login failed");
      }
    } catch (error) {
      console.error("Login error:", error);
      throw error;
    } finally {
      setLoading(false);
    }
  };

  const signup = async (email: string, password: string, name?: string) => {
    setLoading(true);
    try {
      const response = await auth.signUp.email({
        email,
        password,
        name: name || email.split('@')[0],
        callbackURL: "/profile-setup", // Redirect to profile setup after signup
      });
      if (!response) {
        throw new Error("Signup failed");
      }
    } catch (error) {
      console.error("Signup error:", error);
      throw error;
    } finally {
      setLoading(false);
    }
  };

  const logout = async () => {
    setLoading(true);
    try {
      await auth.signOut();
    } catch (error) {
      console.error("Logout error:", error);
      throw error;
    } finally {
      setLoading(false);
    }
  };

  const updateUserBackground = async (background: UserBackground) => {
    if (!user?.id) return;

    try {
      const response = await fetch('/api/user/profile', {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          programming_experience: background.programming_experience,
          os_preference: background.os_preference,
          gpu_available: background.gpu_available,
          preferred_language: background.preferred_language,
          learning_goals: background.learning_goals,
          hardware_background: background.hardware_background,
          software_background: background.software_background
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to update user background');
      }

      const data = await response.json();
      // Update local user state with the complete updated user data
      setUser(prev => prev ? { ...prev, ...data } : null);
    } catch (error) {
      console.error("Update user background error:", error);
      throw error;
    }
  };

  const fetchUserBackground = async () => {
    if (!user?.id) return;

    try {
      const response = await fetch(`/api/user/profile?user_id=${user.id}`, {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        throw new Error('Failed to fetch user background');
      }

      const data = await response.json();
      setUser(prev => prev ? { ...prev, ...data } : null);
    } catch (error) {
      console.error("Fetch user background error:", error);
    }
  };

  const value = {
    user,
    loading,
    login,
    signup,
    logout,
    updateUserBackground,
    fetchUserBackground,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};