'use client';

import { useState, useEffect } from 'react';
import { ConfigSection } from './config-section';
import { Button } from '../ui/button';
import { useToast } from '@/hooks/use-toast';
import { Save, Server, Bot, Cpu } from 'lucide-react';

const workstationOptions = {
  GPU: ['NVIDIA RTX 4070 Ti (12GB)', 'NVIDIA RTX 3090 (24GB)', 'NVIDIA RTX 4090 (24GB)', 'Cloud: AWS g5.2xlarge'],
  CPU: ['Intel Core i7 (13th Gen+)', 'AMD Ryzen 9', 'Cloud CPU'],
  RAM: ['32 GB DDR5', '64 GB DDR5', 'Cloud RAM'],
  OS: ['Ubuntu 22.04 LTS', 'Windows (with WSL2/Dual-boot)', 'Cloud AMI'],
};

const edgeKitOptions = {
  Brain: ['NVIDIA Jetson Orin Nano (8GB)', 'NVIDIA Jetson Orin NX (16GB)'],
  Vision: ['Intel RealSense D435i', 'Intel RealSense D455'],
  'Voice Interface': ['ReSpeaker USB Mic Array v2.0', 'Standard USB Microphone'],
};

const robotLabOptions = {
  Robot: ['Unitree Go2 Edu (Proxy)', 'Unitree G1 (Miniature)', 'Hiwonder TonyPi Pro (Budget)', 'None (Simulation Only)'],
};

type Configuration = {
  [key: string]: string;
};

type ConfigState = {
  workstation: Configuration;
  edgeKit: Configuration;
  robotLab: Configuration;
};

export function HardwareConfigurator() {
  const [config, setConfig] = useState<ConfigState>({
    workstation: {},
    edgeKit: {},
    robotLab: {},
  });
  const [isClient, setIsClient] = useState(false);
  const { toast } = useToast();

  useEffect(() => {
    setIsClient(true);
    try {
      const savedConfig = localStorage.getItem('hardwareConfig');
      if (savedConfig) {
        setConfig(JSON.parse(savedConfig));
      }
    } catch (error) {
        console.error("Failed to load hardware config from localStorage", error);
    }
  }, []);

  const handleSelection = (category: keyof ConfigState, item: string, value: string) => {
    setConfig(prev => ({
      ...prev,
      [category]: {
        ...prev[category],
        [item]: value,
      },
    }));
  };

  const saveConfig = () => {
    try {
      localStorage.setItem('hardwareConfig', JSON.stringify(config));
      toast({
        title: 'Configuration Saved',
        description: 'Your hardware setup has been saved to this browser.',
      });
    } catch (error) {
        toast({
            variant: 'destructive',
            title: 'Save Failed',
            description: 'Could not save configuration to local storage.',
        });
    }
  };

  if (!isClient) {
    return null; // Or a loading skeleton
  }

  return (
    <div className="space-y-12">
      <ConfigSection
        title="Digital Twin Workstation"
        description="The primary machine for simulation and model training."
        icon={<Server className="h-6 w-6 text-primary" />}
        options={workstationOptions}
        selection={config.workstation}
        onSelect={(item, value) => handleSelection('workstation', item, value)}
      />
      <ConfigSection
        title="Physical AI Edge Kit"
        description="The 'brain without a body' for deploying and testing your code physically."
        icon={<Cpu className="h-6 w-6 text-primary" />}
        options={edgeKitOptions}
        selection={config.edgeKit}
        onSelect={(item, value) => handleSelection('edgeKit', item, value)}
      />
      <ConfigSection
        title="Robot Lab"
        description="The physical robot hardware for real-world application."
        icon={<Bot className="h-6 w-6 text-primary" />}
        options={robotLabOptions}
        selection={config.robotLab}
        onSelect={(item, value) => handleSelection('robotLab', item, value)}
      />
      <div className="flex justify-end pt-4">
        <Button onClick={saveConfig}>
          <Save className="mr-2 h-4 w-4" />
          Save My Configuration
        </Button>
      </div>
    </div>
  );
}
