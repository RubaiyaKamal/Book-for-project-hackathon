/** @type {import('next').NextConfig} */
const nextConfig = {
  // Output configuration for deployment
  output: 'standalone',

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
}

module.exports = nextConfig
