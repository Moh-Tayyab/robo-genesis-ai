'use client';

import { SessionProvider } from '@/lib/providers/session-provider';
import { ReactNode } from 'react';

export function SessionProviderWrapper({ children }: { children: ReactNode }) {
  return <SessionProvider>{children}</SessionProvider>;
}