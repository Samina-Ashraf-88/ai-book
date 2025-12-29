import React, { createContext, useContext, ReactNode } from 'react';
import { useSession } from 'better-auth/react';

interface AuthContextType {
  user: any;
  isLoggedIn: boolean;
  signIn: () => void;
  signOut: () => void;
  signUp: () => void;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const { data: session, signIn, signOut, signUp } = useSession();

  const value: AuthContextType = {
    user: session?.user,
    isLoggedIn: !!session?.user,
    signIn,
    signOut,
    signUp,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};