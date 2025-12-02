'use client';

import { useState } from 'react';
import { ChapterActions } from './chapter-actions';
import { generateChapterContent } from '@/ai/flows/generate-chapter-content';
import { translateText } from '@/ai/flows/translate-text';
import { useAuth } from '@/hooks/use-auth';
import { InteractiveCodeBlock } from '../code/interactive-code-block';
import { Loader2 } from 'lucide-react';

export function ChapterContent({ initialContent }: { initialContent: string }) {
  const [content, setContent] = useState(initialContent);
  const [isLoading, setIsLoading] = useState(false);
  const { user } = useAuth();

  const handlePersonalize = async () => {
    setIsLoading(true);
    const expertise = user.expertise?.software || 'beginner';
    try {
      const result = await generateChapterContent({
        topic: "Personalize Content",
        existingContent: content,
        outline: `Rewrite the following content to be tailored for a user with ${expertise} level software expertise. Make it more or less technical as appropriate.`,
      });
      setContent(result.content);
    } catch (error) {
      console.error('Failed to personalize content:', error);
      // Optionally, show a toast notification
    } finally {
      setIsLoading(false);
    }
  };

  const handleTranslate = async () => {
    setIsLoading(true);
    try {
      const result = await translateText({
        textToTranslate: content,
        targetLanguage: 'Urdu',
      });
      setContent(result.translatedText);
    } catch (error) {
      console.error('Failed to translate content:', error);
    } finally {
      setIsLoading(false);
    }
  };

  const codeExample = `import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloWorldPublisher(Node):
    def __init__(self):
        super().__init__('hello_world_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_world', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = HelloWorldPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
`;

  const renderContent = () => {
    // This is a simple way to replace a placeholder with a component.
    // A more robust solution would use a Markdown/MDX parser.
    const parts = content.split('</h3>');
    if (parts.length > 1 && content.includes("Python Code Example: A Simple Publisher Node")) {
        return (
            <>
                <div dangerouslySetInnerHTML={{ __html: parts[0] + '</h3>' }} />
                <InteractiveCodeBlock code={codeExample} language="python" />
                <div dangerouslySetInnerHTML={{ __html: parts.slice(1).join('</h3>') }} />
            </>
        )
    }
    return <div dangerouslySetInnerHTML={{ __html: content }} />;
  }

  return (
    <article className="prose prose-neutral dark:prose-invert max-w-none">
      <ChapterActions
        onPersonalize={handlePersonalize}
        onTranslate={handleTranslate}
        isLoading={isLoading}
      />
      <div className="relative mt-8">
        {isLoading && (
          <div className="absolute inset-0 z-10 flex items-center justify-center rounded-lg bg-background/60 backdrop-blur-sm">
            <div className="h-10 w-10 animate-spin text-primary" />
          </div>
        )}
        <div className={isLoading ? 'opacity-50' : ''}>
            {renderContent()}
        </div>
      </div>
    </article>
  );
}
