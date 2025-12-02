'use client';

import React from 'react';
import {
  SidebarProvider,
  Sidebar,
  SidebarInset,
} from '@/components/ui/sidebar';
import { AppSidebar } from './sidebar';
import { Header } from './header';
import { ChatbotWidget } from '../chatbot/chatbot-widget';

export function AppShell({ children }: { children: React.ReactNode }) {
  // Get sidebar state from cookie
  const getSidebarState = () => {
    if (typeof window === 'undefined') return true;
    const cookie = document.cookie
      .split('; ')
      .find((row) => row.startsWith('sidebar_state='));
    return cookie ? cookie.split('=')[1] === 'true' : true;
  };
  
  return (
    <SidebarProvider defaultOpen={getSidebarState()}>
      <Sidebar collapsible="icon" className="border-r">
        <AppSidebar />
      </Sidebar>
      <SidebarInset className="flex flex-col">
        <Header />
        <main className="flex-1 overflow-y-auto">{children}</main>
        <ChatbotWidget />
      </SidebarInset>
    </SidebarProvider>
  );
}
