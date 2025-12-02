'use server';

/**
 * @fileOverview A flow for translating text to a specified language.
 *
 * - translateText - A function that handles the translation of text.
 * - TranslateTextInput - The input type for the translateText function.
 * - TranslateTextOutput - The return type for the translateText function.
 */

import { model } from '@/ai/genkit';
import { z } from 'zod';

const TranslateTextInputSchema = z.object({
  textToTranslate: z.string().describe('The text content to be translated.'),
  targetLanguage: z.string().describe('The target language for the translation (e.g., "Urdu", "Spanish").'),
});
export type TranslateTextInput = z.infer<typeof TranslateTextInputSchema>;

const TranslateTextOutputSchema = z.object({
  translatedText: z.string().describe('The translated text.'),
});
export type TranslateTextOutput = z.infer<typeof TranslateTextOutputSchema>;

export async function translateText(input: TranslateTextInput): Promise<TranslateTextOutput> {
  try {
    const prompt = `Translate the following text into ${input.targetLanguage}. Preserve the original formatting, including markdown for headings, lists, and bold text. Do not add any extra commentary or explanations, just provide the direct translation.

Text to translate:
${input.textToTranslate}`;

    const result = await model.generateContent(prompt);
    const response = await result.response;
    const translatedText = response.text();

    return { translatedText };
  } catch (error) {
    console.error('Error in translateText:', error);
    return { translatedText: input.textToTranslate }; // Return original text on error
  }
}
