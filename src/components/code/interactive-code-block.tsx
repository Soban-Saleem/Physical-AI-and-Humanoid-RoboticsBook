'use client';
import { useState } from 'react';
import { Button } from '@/components/ui/button';
import { Terminal, Check, Copy, Loader2 } from 'lucide-react';
import { suggestCodeExamples } from '@/ai/flows/suggest-code-examples';
import { Alert, AlertDescription, AlertTitle } from '../ui/alert';
import { useToast } from '@/hooks/use-toast';

interface InteractiveCodeBlockProps {
  code: string;
  language: string;
}

export function InteractiveCodeBlock({ code, language }: InteractiveCodeBlockProps) {
  const [output, setOutput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [hasCopied, setHasCopied] = useState(false);
  const { toast } = useToast();


  const handleRun = async () => {
    setIsLoading(true);
    setOutput('');
    try {
      const result = await suggestCodeExamples({
        roboticsConcept: 'Explain the following code snippet.',
        programmingLanguage: language,
        robotOperatingSystem: 'ROS 2',
        codeType: 'node',
      });
      setOutput(result.explanation);
    } catch (error) {
      console.error('Failed to get code explanation', error);
      toast({
        variant: 'destructive',
        title: 'Error',
        description: 'Could not get code explanation.',
      });
    } finally {
      setIsLoading(false);
    }
  };

  const handleCopy = () => {
    navigator.clipboard.writeText(code);
    setHasCopied(true);
    setTimeout(() => setHasCopied(false), 2000);
  };

  return (
    <div className="my-6 not-prose">
      <div className="relative rounded-t-lg border bg-secondary/50 p-4 font-code">
        <div className="absolute top-2 right-2 flex gap-2">
           <Button variant="ghost" size="icon" className="h-7 w-7" onClick={handleCopy}>
            {hasCopied ? <Check className="h-4 w-4" /> : <Copy className="h-4 w-4" />}
            <span className="sr-only">Copy code</span>
          </Button>
          <Button variant="ghost" size="icon" className="h-7 w-7" onClick={handleRun} disabled={isLoading}>
            {isLoading ? <Loader2 className="h-4 w-4 animate-spin" /> : <Terminal className="h-4 w-4" />}
            <span className="sr-only">Run code</span>
          </Button>
        </div>
        <pre className="text-sm overflow-x-auto p-0 bg-transparent border-none"><code className={`language-${language}`}>{code}</code></pre>
      </div>
      {output && (
        <Alert className="rounded-t-none">
          <Terminal className="h-4 w-4" />
          <AlertTitle>Explanation</AlertTitle>
          <AlertDescription>
            <div className="prose prose-sm dark:prose-invert max-w-none" dangerouslySetInnerHTML={{ __html: output.replace(/\n/g, '<br />') }} />
          </AlertDescription>
        </Alert>
      )}
    </div>
  );
}
