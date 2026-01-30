/** @type {import('next').NextConfig} */
const nextConfig = {
  // Allow images from external sources
  images: {
    domains: ['localhost'],
    remotePatterns: [
      {
        protocol: 'https',
        hostname: '**',
      },
    ],
  },

  // Experimental features
  experimental: {
    serverActions: {
      bodySizeLimit: '2mb',
    },
  },

  // Force fresh build ID to prevent cache issues
  generateBuildId: async () => {
    return `build-${Date.now()}`
  },
}

module.exports = nextConfig
