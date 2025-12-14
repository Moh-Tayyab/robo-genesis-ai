#!/bin/bash
# vercel-build.sh - Custom build script for Vercel deployment

# Install pnpm if not available
if ! command -v pnpm &> /dev/null; then
  npm install -g pnpm
fi

# Install root dependencies
cd ../..
pnpm install

# Build all dependencies
pnpm build

# Go back to the app directory and build
cd apps/auth
npm run build