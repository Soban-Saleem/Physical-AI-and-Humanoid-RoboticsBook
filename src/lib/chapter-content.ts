'use server';
import fs from 'fs/promises';
import path from 'path';

export async function getChapterContent(contentPath: string): Promise<string> {
    const filePath = path.join(process.cwd(), 'docs', contentPath);
    try {
        const content = await fs.readFile(filePath, 'utf-8');
        return content;
    } catch (error) {
        console.error(`Error reading file at ${filePath}:`, error);
        throw new Error(`Could not fetch content for ${contentPath}`);
    }
}
