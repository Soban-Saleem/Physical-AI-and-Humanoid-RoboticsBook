'use client';

import { useState, useRef, useEffect, FormEvent } from 'react';
import {
  Bot,
  Loader2,
  Send,
  Sparkles,
  User,
  ScanText,
  MessageSquare,
} from 'lucide-react';
import {
  Card,
  CardContent,
  CardFooter,
  CardHeader,
  CardTitle,
} from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Input } from '@/components/ui/input';
import { answerBookQuestionsWithRAG } from '@/ai/flows/answer-book-questions-with-rag';
import { cn } from '@/lib/utils';
import { ScrollArea } from '../ui/scroll-area';
import { Avatar, AvatarFallback, AvatarImage } from '../ui/avatar';
import { useAuth } from '@/hooks/use-auth';
import { usePageContext } from '@/context/page-context';
import { Switch } from '../ui/switch';
import { Label } from '../ui/label';

interface Message {
  role: 'user' | 'assistant';
  content: string;
}

const suggestedQuestions = [
    "What is Physical AI?",
    "Summarize ROS 2 Fundamentals.",
    "Explain the difference between URDF and SDF.",
];

export function Chatbot() {
  const [messages, setMessages] = useState<Message[]>([
    {
        role: 'assistant',
        content: "Hi! I'm your PhysAI assistant. Ask me anything about the textbook content. You can also enable 'Selection Mode' to ask questions about specific text on the page."
    }
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const scrollAreaRef = useRef<HTMLDivElement>(null);
  const { user } = useAuth();
  const { selectedText, isSelectionMode, setIsSelectionMode } = usePageContext();

  const getInitials = (name?: string) => {
    if (!name) return 'U';
    return name.split(' ').map(n => n[0]).join('').substring(0, 2).toUpperCase();
  }


  useEffect(() => {
    if (scrollAreaRef.current) {
        const viewport = scrollAreaRef.current.querySelector('div');
        if (viewport) {
            viewport.scrollTop = viewport.scrollHeight;
        }
    }
  }, [messages]);


  const handleSendMessage = async (messageContent: string) => {
    if (!messageContent.trim()) return;

    const userMessage: Message = { role: 'user', content: messageContent };
    const newMessages = [...messages, userMessage];
    setMessages(newMessages);
    setIsLoading(true);

    try {
      const history = newMessages.slice(0, -1);
      const result = await answerBookQuestionsWithRAG({ 
        question: messageContent, 
        history,
        selectedText: (isSelectionMode && selectedText) ? selectedText : undefined
      });
      const assistantMessage: Message = {
        role: 'assistant',
        content: result.answer,
      };
      setMessages((prev) => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error getting answer:', error);
      const errorMessage: Message = {
        role: 'assistant',
        content: "Sorry, I couldn't get an answer. Please try again.",
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSubmit = async (e: FormEvent) => {
    e.preventDefault();
    handleSendMessage(input);
    setInput('');
  };

  return (
    <Card className="flex h-[70vh] max-h-[700px] w-full flex-col shadow-2xl">
      <CardHeader className="flex flex-row items-center justify-between">
        <div className="flex items-center gap-3">
          <div className="relative">
            <Bot className="h-8 w-8 text-primary" />
            <div className="absolute -top-1 -right-1 flex h-4 w-4 items-center justify-center rounded-full border-2 border-background bg-primary text-primary-foreground">
                <Sparkles className="h-2 w-2" />
            </div>
          </div>
          <CardTitle className="font-headline text-xl">
            PhysAI Assistant
          </CardTitle>
        </div>
        <div className="flex items-center space-x-2">
            <Label htmlFor="selection-mode" className="flex items-center gap-1 text-sm text-muted-foreground">
                <ScanText className="h-4 w-4" />
                <span>Select</span>
            </Label>
            <Switch id="selection-mode" checked={isSelectionMode} onCheckedChange={setIsSelectionMode} />
        </div>
      </CardHeader>
      <CardContent className="flex-1 overflow-hidden">
        <ScrollArea className="h-full pr-4" ref={scrollAreaRef}>
          <div className="space-y-6">
            {messages.map((message, index) => (
              <div
                key={index}
                className={cn(
                  'flex items-start gap-3',
                  message.role === 'user' ? 'justify-end' : 'justify-start'
                )}
              >
                {message.role === 'assistant' && (
                   <Avatar className="h-8 w-8 border">
                    <div className="flex h-full w-full items-center justify-center bg-primary text-primary-foreground">
                      <Bot className="h-5 w-5" />
                    </div>
                  </Avatar>
                )}
                <div
                  className={cn(
                    'max-w-[85%] rounded-2xl px-4 py-3 text-sm shadow-sm',
                    message.role === 'user'
                      ? 'rounded-br-none bg-primary text-primary-foreground'
                      : 'rounded-bl-none bg-muted'
                  )}
                >
                  <p>{message.content}</p>
                </div>
                 {message.role === 'user' && (
                   <Avatar className="h-8 w-8 border">
                    <AvatarImage src={user.email ? `https://avatar.vercel.sh/${user.email}.png` : ''} alt={user.name} />
                    <AvatarFallback>{getInitials(user.name)}</AvatarFallback>
                  </Avatar>
                )}
              </div>
            ))}
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
            {!isLoading && messages.length <= 1 && (
                <div className="space-y-2">
                    {suggestedQuestions.map((q) => (
                        <Button 
                            key={q}
                            variant="outline" 
                            size="sm" 
                            className="w-full justify-start text-left h-auto"
                            onClick={() => handleSendMessage(q)}
                        >
                            {q}
                        </Button>
                    ))}
                </div>
            )}
          </div>
        </ScrollArea>
      </CardContent>
      <CardFooter>
        <form onSubmit={handleSubmit} className="flex w-full items-center gap-2">
          <Input
            value={input}
            onChange={(e) => setInput(e.target.value)}
            placeholder={"Ask a follow-up..."}
            disabled={isLoading}
            autoFocus
          />
          <Button type="submit" size="icon" disabled={isLoading || !input.trim()}>
            <Send className="h-4 w-4" />
            <span className="sr-only">Send</span>
          </Button>
        </form>
      </CardFooter>
    </Card>
  );
}
