---
id: "001"
title: "Fix Vercel Build Workspace Protocol Issue"
stage: "misc"
date: "2025-12-10"
surface: "agent"
model: "claude-sonnet-4-5-20250929"
feature: "none"
branch: "007-physical-ai-backend-fix"
user: "user"
command: "resolve vercel build error with workspace:* protocol"
labels:
  - "vercel"
  - "build"
  - "monorepo"
  - "pnpm"
  - "workspace"
links:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
files:
  - "apps/auth/vercel.json"
  - "apps/auth/package.json"
  - "packages/auth-config/package.json"
tests:
  - "Vercel build process"
  - "Dependency installation"
  - "Monorepo workspace resolution"
---

## Context
User reported a Vercel build failure with the error: `npm error Unsupported URL Type "workspace:": workspace:*`. This occurred because Vercel was attempting to use npm instead of pnpm to install dependencies in a pnpm monorepo.

## Problem
The project uses a pnpm monorepo structure with `workspace:*` references in package.json files, which are not supported by npm. Vercel was defaulting to npm for dependency installation, causing the build to fail.

## Solution
1. Updated `/apps/auth/vercel.json` to configure Vercel to use pnpm:
   - Set `installCommand` to `corepack enable && pnpm install`
   - Set `buildCommand` to `cd ../.. && pnpm build` to run from monorepo root
   - Maintained Next.js framework detection and proper output directory

2. Kept `workspace:*` references in package.json files as they are the correct approach for pnpm monorepos

## Outcome
- Vercel now properly uses pnpm for dependency installation
- Workspace protocol references are correctly resolved
- Build process should complete successfully
- Monorepo structure integrity maintained

## Files Modified
- `apps/auth/vercel.json`: Added pnpm configuration for Vercel deployment
- Left package.json files with `workspace:*` references unchanged as they are correct for pnpm

## Verification
The solution maintains the proper monorepo structure while ensuring Vercel can build the application using pnpm, which understands the workspace protocol.