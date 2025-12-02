import { HardwareConfigurator } from '@/components/hardware-configurator/configurator';
import { AppShell } from '@/components/layout/app-shell';
import { HardHat } from 'lucide-react';

export default function HardwarePage() {
  return (
    <AppShell>
      <div className="container mx-auto max-w-6xl py-8 md:py-12">
        <div className="mb-8 space-y-2">
            <h1 className="font-headline text-4xl font-bold tracking-tight text-foreground sm:text-5xl">
                Hardware Configurator
            </h1>
            <p className="text-lg text-muted-foreground">
                Plan your setup for the Physical AI & Humanoid Robotics course. Select components for your workstation, edge kit, and robot lab.
            </p>
        </div>
        <HardwareConfigurator />
      </div>
    </AppShell>
  );
}
