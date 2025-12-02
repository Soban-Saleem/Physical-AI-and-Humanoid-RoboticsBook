import Image from 'next/image';
import Link from 'next/link';
import { Bot } from 'lucide-react';

export function Logo({ size = 'default' }: { size?: 'default' | 'large' }) {
  const sizeClasses = size === 'large' ? 'h-12 w-12' : 'h-8 w-8';
  const textSize = size === 'large' ? 'text-3xl' : 'text-xl';
  return (
    <Link href="/" className="flex items-center gap-2" prefetch={false}>
      <Bot className={`${sizeClasses} text-primary`} />
      <span className={`font-headline font-bold ${textSize} tracking-tighter text-foreground`}>
        PhysAI Handbook
      </span>
    </Link>
  );
}
