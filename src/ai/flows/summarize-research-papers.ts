'use server';

/**
 * @fileOverview A flow to summarize research papers related to Physical AI and Humanoid Robotics.
 *
 * - summarizeResearchPaper - A function that handles the summarization of a research paper.
 * - SummarizeResearchPaperInput - The input type for the summarizeResearchPaper function.
 * - SummarizeResearchPaperOutput - The return type for the summarizeResearchPaper function.
 */

import {ai} from '@/ai/genkit';
import {z} from 'genkit';

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
  return summarizeResearchPaperFlow(input);
}

const summarizeResearchPaperPrompt = ai.definePrompt({
  name: 'summarizeResearchPaperPrompt',
  input: {schema: SummarizeResearchPaperInputSchema},
  output: {schema: SummarizeResearchPaperOutputSchema},
  prompt: `You are an AI assistant helping an author write a textbook on Physical AI and Humanoid Robotics.
Your task is to summarize the key information from a research paper provided to you. Focus on extracting the core findings, methodologies, and conclusions.

Research Paper Text:
{{{paperText}}}`,
});

const summarizeResearchPaperFlow = ai.defineFlow(
  {
    name: 'summarizeResearchPaperFlow',
    inputSchema: SummarizeResearchPaperInputSchema,
    outputSchema: SummarizeResearchPaperOutputSchema,
  },
  async input => {
    const {output} = await summarizeResearchPaperPrompt(input);
    return output!;
  }
);
