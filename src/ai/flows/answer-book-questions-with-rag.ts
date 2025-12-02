'use server';
/**
 * @fileOverview A RAG chatbot for answering questions about the textbook content.
 *
 * - answerBookQuestionsWithRAG - A function that handles the question answering process.
 * - AnswerBookQuestionsWithRAGInput - The input type for the answerBookQuestionsWithRAG function.
 * - AnswerBookQuestionsWithRAGOutput - The return type for the answerBookQuestionsWithRAG function.
 */

import { model } from '@/ai/genkit';
import { z } from 'zod';

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
  try {
    const historyText = input.history?.map(msg => `${msg.role}: ${msg.content}`).join('\n') || '';
    
    const prompt = `You are a helpful chatbot answering questions about the content of a Physical AI & Humanoid Robotics textbook.

${historyText ? `Conversation History:\n${historyText}\n` : ''}

${input.selectedText ? `Selected Text:\n${input.selectedText}\n` : ''}

Question: ${input.question}

Please provide a helpful, accurate answer based on the context provided.`;

    const result = await model.generateContent(prompt);
    const response = await result.response;
    const answer = response.text();

    return { answer };
  } catch (error) {
    console.error('Error in answerBookQuestionsWithRAG:', error);
    return { answer: "I'm sorry, I encountered an error while processing your question. Please try again." };
  }
}
