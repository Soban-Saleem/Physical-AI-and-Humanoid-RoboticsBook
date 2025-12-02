'use client';

import { usePathname } from 'next/navigation';
import { type Chapter } from '@/lib/book-data';
import { getChapterContent } from '@/lib/chapter-content';
import { ChapterContent } from '@/components/chapter/chapter-content';
import { usePageContext } from '@/context/page-context';
import { useEffect, useState } from 'react';
import { Skeleton } from '@/components/ui/skeleton';

function ChapterLoadingSkeleton() {
  return (
    <div className="space-y-8">
      <Skeleton className="h-16 w-3/4" />
      <div className="space-y-4">
        <Skeleton className="h-6 w-full" />
        <Skeleton className="h-6 w-5/6" />
        <Skeleton className="h-6 w-full" />
      </div>
      <Skeleton className="h-10 w-1/2" />
      <div className="space-y-4">
        <Skeleton className="h-6 w-full" />
        <Skeleton className="h-6 w-full" />
        <Skeleton className="h-6 w-4/6" />
      </div>
    </div>
  );
}

export function ChapterPageClient({ chapter }: { chapter: Chapter }) {
  const [content, setContent] = useState<string>('');
  const [isLoading, setIsLoading] = useState(true);
  const { setPageContent } = usePageContext();
  const pathname = usePathname();

  useEffect(() => {
    async function loadContent() {
      if (chapter) {
        setIsLoading(true);
        try {
          const chapterContent = await getChapterContent(chapter.contentPath);
          setContent(chapterContent);
          setPageContent(chapterContent);
        } catch (error) {
          console.error('Failed to load chapter content', error);
          setContent('<p>Error loading content.</p>');
        } finally {
          setIsLoading(false);
        }
      }
    }
    loadContent();

    // Reset content when navigating away
    return () => {
      setPageContent('');
    };
  }, [chapter, setPageContent, pathname]);

  return (
    <div className="container max-w-4xl py-8 md:py-12">
      {isLoading ? <ChapterLoadingSkeleton /> : <ChapterContent initialContent={content} />}
    </div>
  );
}