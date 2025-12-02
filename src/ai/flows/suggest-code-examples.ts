// Server actions disabled for static export

/**
 * @fileOverview An AI agent for suggesting code examples for robotics concepts.
 *
 * - suggestCodeExamples - A function that suggests code examples.
 * - SuggestCodeExamplesInput - The input type for the suggestCodeExamples function.
 * - SuggestCodeExamplesOutput - The return type for the suggestCodeExamples function.
 */

import { model } from '@/ai/genkit';
import { z } from 'zod';

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
  try {
    const prompt = `You are an expert robotics software engineer. Generate a code example for the following robotics concept using the specified programming language and Robot Operating System.

Robotics Concept: ${input.roboticsConcept}
Programming Language: ${input.programmingLanguage}
Robot Operating System: ${input.robotOperatingSystem || 'ROS 2'}
Code Type: ${input.codeType || 'basic implementation'}

Provide a well-commented code example and an explanation of how it works.

Format your response as:
CODE EXAMPLE:
[your code here]

EXPLANATION:
[your explanation here]`;

    const result = await model.generateContent(prompt);
    const response = await result.response;
    const text = response.text();

    // Parse the response to extract code and explanation
    const parts = text.split('EXPLANATION:');
    const codeExample = parts[0].replace('CODE EXAMPLE:', '').trim();
    const explanation = parts[1] ? parts[1].trim() : 'Code example for ' + input.roboticsConcept;

    return { codeExample, explanation };
  } catch (error) {
    console.error('Error in suggestCodeExamples:', error);
    return {
      codeExample: `# Example code for ${input.roboticsConcept}\n# Implementation would go here`,
      explanation: `Example implementation for ${input.roboticsConcept} concept.`
    };
  }
}
