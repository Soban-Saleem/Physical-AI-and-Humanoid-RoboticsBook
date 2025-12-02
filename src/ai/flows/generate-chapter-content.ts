'use server';

/**
 * @fileOverview AI-powered chapter content generation flow.
 *
 * - generateChapterContent - A function that generates textbook chapter content using AI.
 * - GenerateChapterContentInput - The input type for the generateChapterContent function.
 * - GenerateChapterContentOutput - The return type for the generateChapterContent function.
 */

import {ai} from '@/ai/genkit';
import {z} from 'genkit';

const GenerateChapterContentInputSchema = z.object({
  topic: z.string().describe('The topic of the chapter to generate.'),
  outline: z.string().optional().describe('An optional outline to guide content generation.'),
  existingContent: z.string().optional().describe('Existing content to expand upon.'),
});
export type GenerateChapterContentInput = z.infer<typeof GenerateChapterContentInputSchema>;

const GenerateChapterContentOutputSchema = z.object({
  content: z.string().describe('The generated chapter content.'),
});
export type GenerateChapterContentOutput = z.infer<typeof GenerateChapterContentOutputSchema>;

export async function generateChapterContent(input: GenerateChapterContentInput): Promise<GenerateChapterContentOutput> {
  return generateChapterContentFlow(input);
}

const prompt = ai.definePrompt({
  name: 'generateChapterContentPrompt',
  input: {schema: GenerateChapterContentInputSchema},
  output: {schema: GenerateChapterContentOutputSchema},
  prompt: `You are a helpful AI assistant that specializes in generating content for technical textbooks, specifically for Physical AI & Humanoid Robotics.

  Based on the topic {{topic}}, generate a chapter draft. Consider the outline, if provided, to structure the content. If there is existing content, expand and improve upon it.

  Outline (if provided): {{{outline}}}
  Existing Content (if provided): {{{existingContent}}}

  Ensure the content is well-structured, technically accurate, and suitable for a textbook format. Use examples, code snippets, and clear explanations where appropriate.
  The textbook is titled "PhysAI Handbook" and is meant to teach Physical AI & Humanoid Robotics.
  Pay close attention to the spec kit plus constitution.
  Follow all instructions to the letter.
`,
});

const generateChapterContentFlow = ai.defineFlow(
  {
    name: 'generateChapterContentFlow',
    inputSchema: GenerateChapterContentInputSchema,
    outputSchema: GenerateChapterContentOutputSchema,
  },
  async input => {
    const {output} = await prompt(input);
    return output!;
  }
);
