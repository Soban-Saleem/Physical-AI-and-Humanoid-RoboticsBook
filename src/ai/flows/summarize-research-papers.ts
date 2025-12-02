// Server actions disabled for static export

/**
 * @fileOverview A flow to summarize research papers related to Physical AI and Humanoid Robotics.
 *
 * - summarizeResearchPaper - A function that handles the summarization of a research paper.
 * - SummarizeResearchPaperInput - The input type for the summarizeResearchPaper function.
 * - SummarizeResearchPaperOutput - The return type for the summarizeResearchPaper function.
 */

import { model } from '@/ai/genkit';
import { z } from 'zod';

const SummarizeResearchPaperInputSchema = z.object({
  paperText: z
    .string()
    .describe('The text content of the research paper to be summarized.'),
});
export type SummarizeResearchPaperInput = z.infer<
  typeof SummarizeResearchPaperInputSchema
>;

const SummarizeResearchPaperOutputSchema = z.object({
  summary: z
    .string()
    .describe('A concise summary of the research paper, extracting key information.'),
});
export type SummarizeResearchPaperOutput = z.infer<
  typeof SummarizeResearchPaperOutputSchema
>;

export async function summarizeResearchPaper(
  input: SummarizeResearchPaperInput
): Promise<SummarizeResearchPaperOutput> {
  try {
    const prompt = `You are an AI assistant helping an author write a textbook on Physical AI and Humanoid Robotics.
Your task is to summarize the key information from a research paper provided to you. Focus on extracting the core findings, methodologies, and conclusions.

Research Paper Text:
${input.paperText}

Please provide a concise summary covering:
1. Main contributions
2. Key methodology  
3. Results and findings
4. Relevance to Physical AI/Robotics`;

    const result = await model.generateContent(prompt);
    const response = await result.response;
    const summary = response.text();

    return { summary };
  } catch (error) {
    console.error('Error in summarizeResearchPaper:', error);
    return { summary: "Error generating summary. Please try again." };
  }
}
