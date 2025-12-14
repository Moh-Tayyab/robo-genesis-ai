import type { NextConfig } from "next";

const nextConfig: NextConfig = {
  transpilePackages: ["@repo/auth-config", "@repo/auth-database"],
  images: {
    // Use the default loader to avoid sharp issues on Windows
    loader: "default",
  },
};

export default nextConfig;
