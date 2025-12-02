'use server';

/**
 * @fileOverview An AI agent for suggesting code examples for robotics concepts.
 *
 * - suggestCodeExamples - A function that suggests code examples.
 * - SuggestCodeExamplesInput - The input type for the suggestCodeExamples function.
 * - SuggestCodeExamplesOutput - The return type for the suggestCodeExamples function.
 */

import {ai} from '@/ai/genkit';
import {z} from 'genkit';

const SuggestCodeExamplesInputSchema = z.object({
  roboticsConcept: z.string().describe('The robotics concept to generate code examples for.'),
  programmingLanguage: z.string().describe('The programming language for the code example (e.g., Python, C++).'),
  robotOperatingSystem: z.string().optional().describe('The Robot Operating System (ROS) version (e.g., ROS 2).'),
  codeType: z.string().optional().describe('The type of code, such as a service, node, etc.'),
});
export type SuggestCodeExamplesInput = z.infer<typeof SuggestCodeExamplesInputSchema>;

const SuggestCodeExamplesOutputSchema = z.object({
  codeExample: z.string().describe('A code example illustrating the robotics concept.'),
  explanation: z.string().describe('An explanation of the code example.'),
});
export type SuggestCodeExamplesOutput = z.infer<typeof SuggestCodeExamplesOutputSchema>;

export async function suggestCodeExamples(input: SuggestCodeExamplesInput): Promise<SuggestCodeExamplesOutput> {
  return suggestCodeExamplesFlow(input);
}

const prompt = ai.definePrompt({
  name: 'suggestCodeExamplesPrompt',
  input: {schema: SuggestCodeExamplesInputSchema},
  output: {schema: SuggestCodeExamplesOutputSchema},
  prompt: `You are an expert robotics software engineer. Generate a code example for the following robotics concept using the specified programming language and Robot Operating System.

Robotics Concept: {{{roboticsConcept}}}
Programming Language: {{{programmingLanguage}}}
Robot Operating System: {{robotOperatingSystem}}
Code Type: {{codeType}}

Provide a well-commented code example and an explanation of how it works.`, 
});

const suggestCodeExamplesFlow = ai.defineFlow(
  {
    name: 'suggestCodeExamplesFlow',
    inputSchema: SuggestCodeExamplesInputSchema,
    outputSchema: SuggestCodeExamplesOutputSchema,
  },
  async input => {
    const {output} = await prompt(input);
    return output!;
  }
);
