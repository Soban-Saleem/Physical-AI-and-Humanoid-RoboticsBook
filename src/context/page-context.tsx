'use client';

import React, { createContext, useContext, useState, ReactNode, useEffect, useRef } from 'react';

interface PageContextType {
  pageContent: string;
  setPageContent: (content: string) => void;
  selectedText: string;
  setSelectedText: (text: string) => void;
  isSelectionMode: boolean;
  setIsSelectionMode: (isSelectionMode: boolean) => void;
  popoverRef: React.MutableRefObject<HTMLElement | null>;
  isPopoverOpen: boolean;
  openPopover: () => void;
  closePopover: () => void;
}

const PageContext = createContext<PageContextType | undefined>(undefined);

export function PageContextProvider({ children }: { children: ReactNode }) {
  const [pageContent, setPageContent] = useState<string>('');
  const [selectedText, setSelectedText] = useState<string>('');
  const [isSelectionMode, setIsSelectionMode] = useState<boolean>(false);
  
  const popoverRef = useRef<HTMLElement | null>(null);
  const [isPopoverOpen, setIsPopoverOpen] = useState(false);
  const [isClient, setIsClient] = useState(false);

  useEffect(() => {
    setIsClient(true);
  }, []);

  const openPopover = () => setIsPopoverOpen(true);
  const closePopover = () => {
    setIsPopoverOpen(false);
    setSelectedText('');
    if (typeof window !== 'undefined' && window.getSelection) {
        window.getSelection()?.removeAllRanges();
    }
  };

  useEffect(() => {
    if (!isClient) return;

    // This listener handles closing the popover when clicking *outside* of it.
    const handleClickOutside = (event: MouseEvent) => {
      const targetElement = event.target as HTMLElement;
      // Radix Popover content has the 'data-radix-popper-content-wrapper' attribute on its outermost div.
      // We check if the click was inside this wrapper.
      if (targetElement.closest('[data-radix-popper-content-wrapper]')) {
        return;
      }
      
      // If the popover is open and the click is outside, close it.
      if (isPopoverOpen) {
        closePopover();
      }
    };
    
    // This listener handles opening the popover when text is selected.
    const handleSelection = () => {
      if (!isSelectionMode) return;
      
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && selection && selection.rangeCount > 0) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        
        const virtualEl = {
          getBoundingClientRect: () => rect,
        };
        // The type assertion is a bit of a hack for Popper, but it works.
        popoverRef.current = virtualEl as HTMLElement;

        setSelectedText(text);
        openPopover();
      }
    };
    
    document.addEventListener('click', handleClickOutside);
    document.addEventListener('mouseup', handleSelection);

    return () => {
      document.removeEventListener('click', handleClickOutside);
      document.removeEventListener('mouseup', handleSelection);
    };
    // Re-run this effect if the popover's open state or selection mode changes.
  }, [isClient, isPopoverOpen, isSelectionMode]);

  useEffect(() => {
    // When switching out of selection mode, ensure the popover is closed.
    if (!isSelectionMode) {
      closePopover();
    }
  }, [isSelectionMode]);

  return (
    <PageContext.Provider value={{ 
        pageContent, setPageContent, 
        selectedText, setSelectedText, 
        isSelectionMode, setIsSelectionMode,
        popoverRef,
        isPopoverOpen,
        openPopover,
        closePopover
    }}>
      {children}
    </PageContext.Provider>
  );
}

export function usePageContext() {
  const context = useContext(PageContext);
  if (context === undefined) {
    throw new Error('usePageContext must be used within a PageContextProvider');
  }
  return context;
}
