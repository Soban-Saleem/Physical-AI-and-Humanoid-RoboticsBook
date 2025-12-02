import { GoogleGenerativeAI } from '@google/generative-ai';

const apiKey = process.env.GOOGLE_GENAI_API_KEY || process.env.NEXT_PUBLIC_GOOGLE_GENAI_API_KEY;

if (!apiKey) {
  throw new Error('GOOGLE_GENAI_API_KEY is not set');
}

export const genAI = new GoogleGenerativeAI(apiKey);

export const model = genAI.getGenerativeModel({
  model: 'gemini-pro',
});
