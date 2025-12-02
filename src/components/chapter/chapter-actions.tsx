'use client';

import { useAuth } from '@/hooks/use-auth';
import { Button } from '../ui/button';
import { Languages, Loader2, Sparkles } from 'lucide-react';
import {
  Tooltip,
  TooltipContent,
  TooltipProvider,
  TooltipTrigger,
} from '@/components/ui/tooltip';

interface ChapterActionsProps {
  onPersonalize: () => void;
  onTranslate: () => void;
  isLoading: boolean;
}

export function ChapterActions({ onPersonalize, onTranslate, isLoading }: ChapterActionsProps) {
  const { user } = useAuth();

  if (!user.isLoggedIn) {
    return null;
  }

  return (
    <div className="flex items-center gap-2 not-prose">
      <TooltipProvider>
        <Tooltip>
          <TooltipTrigger asChild>
            <Button variant="outline" size="sm" onClick={onPersonalize} disabled={isLoading}>
              {isLoading ? <Loader2 className="mr-2 h-4 w-4 animate-spin" /> : <Sparkles className="mr-2 h-4 w-4" />}
              Personalize
            </Button>
          </TooltipTrigger>
          <TooltipContent>
            <p>Tailor content to your expertise ({user.expertise?.software || 'N/A'})</p>
          </TooltipContent>
        </Tooltip>
        <Tooltip>
          <TooltipTrigger asChild>
            <Button variant="outline" size="sm" onClick={onTranslate} disabled={isLoading}>
              {isLoading ? <Loader2 className="mr-2 h-4 w-4 animate-spin" /> : <Languages className="mr-2 h-4 w-4" />}
              Translate to Urdu
            </Button>
          </TooltipTrigger>
          <TooltipContent>
            <p>Translate this chapter into Urdu</p>
          </TooltipContent>
        </Tooltip>
      </TooltipProvider>
    </div>
  );
}
