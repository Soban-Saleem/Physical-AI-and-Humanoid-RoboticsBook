'use client';

import React from 'react';
import { usePathname } from 'next/navigation';
import Link from 'next/link';
import {
  SidebarContent,
  SidebarMenu,
  SidebarMenuItem,
  SidebarMenuButton,
} from '@/components/ui/sidebar';
import {
  Accordion,
  AccordionContent,
  AccordionItem,
  AccordionTrigger,
} from '@/components/ui/accordion';
import { bookData } from '@/lib/book-data';
import { Logo } from '../shared/logo';
import { cn } from '@/lib/utils';
import { BookCopy, ChevronRight } from 'lucide-react';

export function AppSidebar() {
  const pathname = usePathname();
  const currentModuleSlug = pathname.split('/')[1] || '';

  return (
    <>
      <div
        data-sidebar="header"
        className="flex h-16 items-center gap-2 border-b p-2"
      >
        <div className="p-2">
            <Logo />
        </div>
      </div>
      <SidebarContent>
        <SidebarMenu>
          <Accordion
            type="multiple"
            defaultValue={bookData.map((m) => m.slug)}
            className="w-full"
          >
            {bookData.map((module, index) => (
              <AccordionItem value={module.slug} key={module.slug}>
                <AccordionTrigger
                  className={cn(
                    'px-4 py-2 text-sm font-semibold text-sidebar-foreground/80 hover:bg-sidebar-accent hover:text-sidebar-accent-foreground hover:no-underline rounded-md',
                    'group-data-[collapsible=icon]:px-2'
                  )}
                >
                  <div className="flex items-center gap-2">
                    <BookCopy className="h-4 w-4" />
                    <span className="truncate group-data-[collapsible=icon]:hidden">{module.title}</span>
                  </div>
                </AccordionTrigger>
                <AccordionContent className="group-data-[collapsible=icon]:hidden">
                  <div className="pl-6 pr-2 pt-1">
                    <ul className="space-y-1">
                      {module.chapters.map((chapter) => {
                        const href = `/${module.slug}/${chapter.slug}`;
                        const isActive = pathname === href;
                        return (
                          <li key={chapter.slug}>
                            <Link href={href} prefetch={false}>
                              <div
                                className={cn(
                                  'flex items-center justify-between rounded-md px-3 py-2 text-sm text-sidebar-foreground/70 hover:bg-sidebar-accent hover:text-sidebar-accent-foreground',
                                  isActive && 'bg-sidebar-accent text-sidebar-accent-foreground font-medium'
                                )}
                              >
                                <span className="truncate">{chapter.title}</span>
                                {isActive && <ChevronRight className="h-4 w-4" />}
                              </div>
                            </Link>
                          </li>
                        );
                      })}
                    </ul>
                  </div>
                </AccordionContent>
              </AccordionItem>
            ))}
          </Accordion>
        </SidebarMenu>
      </SidebarContent>
    </>
  );
}
