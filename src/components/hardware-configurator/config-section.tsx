'use client';

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../ui/card';
import { Label } from '../ui/label';
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '../ui/select';

interface ConfigSectionProps {
  title: string;
  description: string;
  icon: React.ReactNode;
  options: { [key: string]: string[] };
  selection: { [key: string]: string };
  onSelect: (item: string, value: string) => void;
}

export function ConfigSection({ title, description, icon, options, selection, onSelect }: ConfigSectionProps) {
  return (
    <Card>
      <CardHeader>
        <div className="flex items-start gap-4">
            {icon}
            <div>
                <CardTitle className="font-headline text-2xl">{title}</CardTitle>
                <CardDescription>{description}</CardDescription>
            </div>
        </div>
      </CardHeader>
      <CardContent>
        <div className="grid grid-cols-1 gap-6 md:grid-cols-2 lg:grid-cols-3">
          {Object.entries(options).map(([item, values]) => (
            <div key={item} className="grid gap-2">
              <Label htmlFor={`${title}-${item}`}>{item}</Label>
              <Select onValueChange={(value) => onSelect(item, value)} value={selection[item]}>
                <SelectTrigger id={`${title}-${item}`}>
                  <SelectValue placeholder={`Select ${item}`} />
                </SelectTrigger>
                <SelectContent>
                  {values.map((value) => (
                    <SelectItem key={value} value={value}>
                      {value}
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
            </div>
          ))}
        </div>
      </CardContent>
    </Card>
  );
}
