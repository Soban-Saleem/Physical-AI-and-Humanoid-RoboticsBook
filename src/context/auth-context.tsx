'use client';

import type { ReactNode } from 'react';
import React, { createContext, useState, useEffect } from 'react';

export type UserExpertise = 'beginner' | 'intermediate' | 'expert' | 'unknown';

export interface User {
  isLoggedIn: boolean;
  name?: string;
  email?: string;
  expertise?: {
    software: UserExpertise;
    hardware: UserExpertise;
  };
}

export interface AuthContextType {
  user: User;
  login: (email: string, name: string) => void;
  logout: () => void;
  signup: (user: Omit<User, 'isLoggedIn'>) => void;
  loading: boolean;
}

export const AuthContext = createContext<AuthContextType | undefined>(undefined);

const defaultUser: User = {
  isLoggedIn: false,
  expertise: { software: 'unknown', hardware: 'unknown' },
};

export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<User>(defaultUser);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    try {
      const storedUser = localStorage.getItem('physai-user');
      if (storedUser) {
        setUser(JSON.parse(storedUser));
      }
    } catch (error) {
      console.error("Failed to parse user from localStorage", error);
      setUser(defaultUser);
    } finally {
      setLoading(false);
    }
  }, []);

  const updateLocalStorage = (user: User) => {
    try {
      localStorage.setItem('physai-user', JSON.stringify(user));
    } catch (error) {
      console.error("Failed to save user to localStorage", error);
    }
  };

  const login = (email: string, name: string) => {
    // In a real app, you'd fetch the full user profile.
    // Here we'll just use the stored expertise if available, or default.
    const loggedInUser = { ...user, isLoggedIn: true, email, name };
    setUser(loggedInUser);
    updateLocalStorage(loggedInUser);
  };

  const logout = () => {
    setUser(defaultUser);
    try {
      localStorage.removeItem('physai-user');
    } catch (error) {
        console.error("Failed to remove user from localStorage", error);
    }
  };
  
  const signup = (newUser: Omit<User, 'isLoggedIn'>) => {
    const userWithLogin = { ...newUser, isLoggedIn: true };
    setUser(userWithLogin);
    updateLocalStorage(userWithLogin);
  };

  const value = { user, login, logout, signup, loading };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}
