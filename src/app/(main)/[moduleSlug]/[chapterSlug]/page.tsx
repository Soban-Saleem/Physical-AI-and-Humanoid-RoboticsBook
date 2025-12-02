import { notFound } from 'next/navigation';
import { bookData, type Chapter } from '@/lib/book-data';
import { ChapterPageClient } from './chapter-page-client';

// Generate static params for all book chapters
export function generateStaticParams() {
  const params: { moduleSlug: string; chapterSlug: string }[] = [];
  
  bookData.forEach((module) => {
    module.chapters.forEach((chapter) => {
      params.push({
        moduleSlug: module.slug,
        chapterSlug: chapter.slug,
      });
    });
  });
  
  return params;
}

function getChapter(params: { moduleSlug: string; chapterSlug: string }): Chapter | undefined {
  const module = bookData.find(m => m.slug === params.moduleSlug);
  if (!module) return undefined;
  return module.chapters.find(c => c.slug === params.chapterSlug);
}

export default function ChapterPage({ params }: { params: { moduleSlug: string; chapterSlug: string } }) {
  const chapter = getChapter(params);

  if (!chapter) {
    notFound();
  }

  return <ChapterPageClient chapter={chapter} />;
}
