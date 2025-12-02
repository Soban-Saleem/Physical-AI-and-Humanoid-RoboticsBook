/** @type {import('next').NextConfig} */
const nextConfig = {
  output: 'export',
  trailingSlash: true,
  images: {
    unoptimized: true,
    domains: ['localhost'],
  },
  env: {
    GOOGLE_GENAI_API_KEY: process.env.GOOGLE_GENAI_API_KEY,
    NEXT_PUBLIC_GOOGLE_GENAI_API_KEY: process.env.NEXT_PUBLIC_GOOGLE_GENAI_API_KEY,
  },
  assetPrefix: process.env.NODE_ENV === 'production' ? '/Physical-AI-and-Humanoid-RoboticsBook' : '',
  basePath: process.env.NODE_ENV === 'production' ? '/Physical-AI-and-Humanoid-RoboticsBook' : '',
}

module.exports = nextConfig