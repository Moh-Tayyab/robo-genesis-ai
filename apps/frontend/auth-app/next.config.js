/** @type {import('next').NextConfig} */
const nextConfig = {
  transpilePackages: ["@repo/auth-config", "@repo/auth-database"],
};

module.exports = nextConfig;