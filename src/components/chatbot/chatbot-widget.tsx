'use client';

import { useState } from 'react';
import { Button } from '@/components/ui/button';
import {
  Popover,
  PopoverContent,
  PopoverTrigger,
} from '@/components/ui/popover';
import { MessageSquare, ScanText } from 'lucide-react';
import { Chatbot } from './chatbot';
import { cn } from '@/lib/utils';
import { usePageContext } from '@/context/page-context';
import {
    Tooltip,
    TooltipContent,
    TooltipProvider,
    TooltipTrigger,
} from '@/components/ui/tooltip';

export function ChatbotWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const { isSelectionMode } = usePageContext();

  return (
    <div className="fixed bottom-6 right-6 z-50">
        <Popover open={isOpen} onOpenChange={setIsOpen}>
          <TooltipProvider>
              <Tooltip>
                <TooltipTrigger asChild>
                    <PopoverTrigger asChild>
                    <Button 
                        size="icon" 
                        className={cn(
                            "h-16 w-16 rounded-full shadow-lg transition-transform duration-300 ease-in-out hover:scale-110",
                            isOpen && "rotate-90 scale-0"
                            )}
                        >
                        {isSelectionMode ? <ScanText className="h-7 w-7" /> : <MessageSquare className="h-7 w-7" />}
                        <span className="sr-only">Toggle Chatbot</span>
                    </Button>
                    </PopoverTrigger>
                </TooltipTrigger>
                <TooltipContent side="left">
                    <p>{isSelectionMode ? "Selection mode is active" : "Open chatbot"}</p>
                </TooltipContent>
              </Tooltip>
          </TooltipProvider>
            <PopoverContent
            side="top"
            align="end"
            onOpenAutoFocus={(e) => e.preventDefault()}
            className="w-[90vw] max-w-md rounded-xl p-0 shadow-none border-none bg-transparent"
            >
            <Chatbot />
            </PopoverContent>
        </Popover>
    </div>
  );
}
