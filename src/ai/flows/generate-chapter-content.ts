// Server actions disabled for static export

/**
 * @fileOverview AI-powered chapter content generation flow.
 *
 * - generateChapterContent - A function that generates textbook chapter content using AI.
 * - GenerateChapterContentInput - The input type for the generateChapterContent function.
 * - GenerateChapterContentOutput - The return type for the generateChapterContent function.
 */

import { model } from '@/ai/genkit';
import { z } from 'zod';

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
  try {
    const prompt = `You are a helpful AI assistant that specializes in generating content for technical textbooks, specifically for Physical AI & Humanoid Robotics.

Based on the topic "${input.topic}", generate a chapter draft. Consider the outline, if provided, to structure the content. If there is existing content, expand and improve upon it.

${input.outline ? `Outline: ${input.outline}\n` : ''}
${input.existingContent ? `Existing Content: ${input.existingContent}\n` : ''}

Ensure the content is well-structured, technically accurate, and suitable for a textbook format. Use examples, code snippets, and clear explanations where appropriate.
The textbook is titled "PhysAI Handbook" and is meant to teach Physical AI & Humanoid Robotics.
Pay close attention to the spec kit plus constitution.
Follow all instructions to the letter.`;

    const result = await model.generateContent(prompt);
    const response = await result.response;
    const content = response.text();

    return { content };
  } catch (error) {
    console.error('Error in generateChapterContent:', error);
    return { content: input.existingContent || "Error generating content. Please try again." };
  }
}
