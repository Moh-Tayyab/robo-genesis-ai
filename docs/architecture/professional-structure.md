# Professional Folder Structure for Robo-Genesis-AI

## Overview

This document outlines the professional folder structure implemented for the Robo-Genesis-AI project. The structure follows modern monorepo best practices with clear separation of concerns.

## Structure

```
robo-genesis-ai/
├── apps/                          # Application packages
│   ├── frontend/                  # Frontend applications
│   │   ├── docs-app/              # Documentation site
│   │   ├── auth-app/              # Authentication application
│   │   └── dashboard/             # Admin dashboard
│   └── backend/                   # Backend services
│       ├── api/                   # Main API service
│       ├── chatkit/               # Chatkit backend service
│       └── auth/                  # Authentication service
├── packages/                      # Shared packages and libraries
│   ├── ui-lib/                    # UI component library
│   ├── auth-config-lib/           # Authentication configuration
│   ├── auth-db/                   # Authentication database layer
│   ├── types/                     # Shared TypeScript types
│   ├── utils/                     # Shared utilities
│   └── constants/                 # Shared constants
├── services/                      # Standalone services
│   ├── database/                  # Database configurations
│   ├── ai-agents/                 # AI agent implementations
│   └── robotics-sim/              # Robotics simulation services
├── specs/                         # Feature specifications
├── docs/                          # Documentation
├── tests/                         # Test files
├── scripts/                       # Build and deployment scripts
├── config/                        # Configuration files
├── assets/                        # Static assets
├── .github/                       # GitHub configurations
├── history/                       # Development history
└── tools/                         # Development tools
```

## Benefits

1. **Clear Separation of Concerns**: Frontend, backend, and shared code are clearly separated
2. **Scalability**: Easy to add new applications, services, or packages
3. **Maintainability**: Related code is grouped together logically
4. **Team Collaboration**: Different teams can work on different parts without conflicts
5. **Tooling Support**: Works well with monorepo tools like Turbo and pnpm

## Migration Notes

- All original applications have been moved to their new locations
- Package names have been made more descriptive
- AI agents and skills have been moved to appropriate service/config directories
- Workspace configuration has been updated to reflect new paths