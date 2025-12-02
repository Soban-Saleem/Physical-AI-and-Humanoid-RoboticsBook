'use client';

import { useState, FormEvent, useEffect } from 'react';
import { usePageContext } from '@/context/page-context';
import { Card, CardContent, CardFooter, CardHeader, CardTitle } from '../ui/card';
import { Textarea } from '../ui/textarea';
import { Button } from '../ui/button';
import { Loader2, Send, X, Bot, Sparkles } from 'lucide-react';
import { answerBookQuestionsWithRAG } from '@/ai/flows/answer-book-questions-with-rag';
import { ScrollArea } from '../ui/scroll-area';
import { cn } from '@/lib/utils';
import { Avatar } from '../ui/avatar';

export function TextSelectionPopover() {
  const {
    isSelectionMode,
    popoverRef,
    isPopoverOpen,
    closePopover,
    selectedText,
  } = usePageContext();

  const [popperElement, setPopperElement] = useState<HTMLDivElement | null>(null);

  const [question, setQuestion] = useState('');
  const [answer, setAnswer] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  
  useEffect(() => {
    if (isPopoverOpen) {
      // When the popover opens, clear any previous state.
      setQuestion('');
      setAnswer('');
      setIsLoading(false);
    }
  }, [isPopoverOpen]);


  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    if (!question.trim()) return;

    setIsLoading(true);
    setAnswer('');
    try {
      const result = await answerBookQuestionsWithRAG({
        question,
        selectedText,
      });
      setAnswer(result.answer);
    } catch (error) {
      console.error('Error getting answer:', error);
      setAnswer("Sorry, I couldn't get an answer. Please try again.");
    } finally {
      setIsLoading(false);
    }
  };

  const handleClose = () => {
    closePopover();
  };

  if (!isSelectionMode || !isPopoverOpen || !popoverRef.current) {
    return null;
  }

  return (
    <div
      ref={setPopperElement}
      className="fixed z-50 w-[90vw] max-w-md top-1/2 left-1/2 transform -translate-x-1/2 -translate-y-1/2"
      data-radix-popper-content-wrapper // Add this attribute for the click-outside logic
    >
      <Card className="shadow-2xl">
        <CardHeader className="flex flex-row items-center justify-between pb-2">
            <div className="flex items-center gap-3">
                 <div className="relative">
                    <Bot className="h-7 w-7 text-primary" />
                    <div className="absolute -top-1 -right-1 flex h-4 w-4 items-center justify-center rounded-full border-2 border-background bg-primary text-primary-foreground">
                        <Sparkles className="h-2 w-2" />
                    </div>
                </div>
                <CardTitle className="font-headline text-lg">
                    Ask about selection
                </CardTitle>
            </div>
          <Button variant="ghost" size="icon" className="h-7 w-7" onClick={handleClose}>
            <X className="h-4 w-4" />
          </Button>
        </CardHeader>
        <CardContent className="space-y-4">
          <ScrollArea className="max-h-32">
            <p className="rounded-md border bg-muted p-3 text-sm text-muted-foreground">
              &quot;{selectedText}&quot;
            </p>
          </ScrollArea>

          {isLoading && (
            <div className="flex items-start gap-3 justify-start">
                 <Avatar className="h-8 w-8 border">
                    <div className="flex h-full w-full items-center justify-center bg-primary text-primary-foreground">
                        <Bot className="h-5 w-5" />
                    </div>
                 </Avatar>
                <div className="bg-muted rounded-2xl rounded-bl-none px-4 py-3 text-sm flex items-center shadow-sm">
                    <Loader2 className="h-4 w-4 animate-spin text-muted-foreground" />
                </div>
            </div>
          )}

          {answer && !isLoading && (
            <div className={cn(
                'flex items-start gap-3 justify-start',
              )}>
                <Avatar className="h-8 w-8 border">
                    <div className="flex h-full w-full items-center justify-center bg-primary text-primary-foreground">
                        <Bot className="h-5 w-5" />
                    </div>
                </Avatar>
                <div className={'max-w-[85%] rounded-2xl rounded-bl-none bg-muted px-4 py-3 text-sm shadow-sm'}>
                    <p>{answer}</p>
                </div>
            </div>
          )}

        </CardContent>
        <CardFooter>
          <form onSubmit={handleSubmit} className="flex w-full items-center gap-2">
            <Textarea
              value={question}
              onChange={(e) => setQuestion(e.target.value)}
              placeholder="What would you like to know?"
              className="min-h-0"
              rows={1}
              disabled={isLoading}
              onKeyDown={(e) => {
                if (e.key === 'Enter' && !e.shiftKey) {
                    e.preventDefault();
                    handleSubmit(e);
                }
              }}
              autoFocus
            />
            <Button type="submit" size="icon" disabled={isLoading || !question.trim()}>
              {isLoading ? <Loader2 className="h-4 w-4 animate-spin" /> : <Send className="h-4 w-4" />}
              <span className="sr-only">Send</span>
            </Button>
          </form>
        </CardFooter>
      </Card>
    </div>
  );
}
