import { redirect } from 'next/navigation'

// This page is the root of the (main) group, which is the dashboard.
// We redirect to the first chapter of the book.
export default function DashboardPage() {
  redirect('/module-1/introduction-to-physical-ai')
}
