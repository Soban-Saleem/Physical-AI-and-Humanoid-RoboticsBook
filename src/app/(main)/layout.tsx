import { AppShell } from "@/components/layout/app-shell";
import { PageContextProvider } from "@/context/page-context";
import { TextSelectionPopover } from "@/components/chatbot/text-selection-popover";

export default function MainLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <PageContextProvider>
      <AppShell>
        {children}
        <TextSelectionPopover />
      </AppShell>
    </PageContextProvider>
  );
}
