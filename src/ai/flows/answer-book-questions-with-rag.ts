'use server';
/**
 * @fileOverview A RAG chatbot for answering questions about the textbook content.
 *
 * - answerBookQuestionsWithRAG - A function that handles the question answering process.
 * - AnswerBookQuestionsWithRAGInput - The input type for the answerBookQuestionsWithRAG function.
 * - AnswerBookQuestionsWithRAGOutput - The return type for the answerBookQuestionsWithRAG function.
 */

import {ai} from '@/ai/genkit';
import {z} from 'genkit';

const MessageSchema = z.object({
  role: z.enum(['user', 'assistant']),
  content: z.string(),
});

const AnswerBookQuestionsWithRAGInputSchema = z.object({
  question: z.string().describe('The question to ask about the book content.'),
  selectedText: z.string().optional().describe('The text selected by the user from the book content.'),
  history: z.array(MessageSchema).optional().describe('The history of the conversation.'),
});
export type AnswerBookQuestionsWithRAGInput = z.infer<typeof AnswerBookQuestionsWithRAGInputSchema>;

const AnswerBookQuestionsWithRAGOutputSchema = z.object({
  answer: z.string().describe('The answer to the question about the book content.'),
});
export type AnswerBookQuestionsWithRAGOutput = z.infer<typeof AnswerBookQuestionsWithRAGOutputSchema>;

export async function answerBookQuestionsWithRAG(input: AnswerBookQuestionsWithRAGInput): Promise<AnswerBookQuestionsWithRAGOutput> {
  return answerBookQuestionsWithRAGFlow(input);
}

const prompt = ai.definePrompt({
  name: 'answerBookQuestionsWithRAGPrompt',
  input: {schema: AnswerBookQuestionsWithRAGInputSchema},
  output: {schema: AnswerBookQuestionsWithRAGOutputSchema},
  prompt: `You are a helpful chatbot answering questions about the content of a Physical AI & Humanoid Robotics textbook.

  Use the conversation history to provide context for the user's question.

  {{#if history}}
  Conversation History:
  {{#each history}}
  {{this.role}}: {{{this.content}}}
  {{/each}}
  {{/if}}

  Answer the question based on the following information:

  {{#if selectedText}}
  Selected Text:
  {{selectedText}}
  {{/if}}

  Question: {{{question}}}
  `,
});

const answerBookQuestionsWithRAGFlow = ai.defineFlow(
  {
    name: 'answerBookQuestionsWithRAGFlow',
    inputSchema: AnswerBookQuestionsWithRAGInputSchema,
    outputSchema: AnswerBookQuestionsWithRAGOutputSchema,
  },
  async input => {
    const {output} = await prompt(input);
    return output!;
  }
);
