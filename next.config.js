/** @type {import('next').NextConfig} */
const nextConfig = {
  images: {
    domains: ['localhost'],
  },
  env: {
    GOOGLE_GENAI_API_KEY: process.env.GOOGLE_GENAI_API_KEY,
  }
}

module.exports = nextConfig