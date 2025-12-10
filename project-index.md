# Physical AI & Humanoid Robotics Textbook - Project Index

## Project Structure

This repository contains a comprehensive textbook on Physical AI and Humanoid Robotics with the following main components:

### 1. Book Source Content
- `book-source/docs/` - Contains all chapter content organized by module:
  - `01-Physical-AI-Foundations/` - Introduction and foundational concepts
  - `02-ROS2-Fundamentals/` - ROS 2 basics and core concepts
  - `03-Simulation-Systems/` - Digital twin and simulation frameworks
  - `04-NVIDIA-Isaac-AI/` - NVIDIA Isaac robotics platform
  - `05-Vision-Language-Action/` - VLA models for robotics
  - `06-Capstone/` - Advanced projects and integration

### 2. Backend Services
- `rag-backend/` - Retrieval-Augmented Generation backend
  - `app/` - FastAPI application
  - `embeddings/` - Embedding processing components
  - `vector-store/` - Vector database (Qdrant) components
  - `db/` - Database connection and management

### 3. Authentication System
- `auth/` - User authentication and profile management
  - `betterauth/` - Better-Auth implementation

### 4. Monorepo Components
- `monorepo/` - Development tools and automation
  - `agents/` - Claude Code agents
  - `skills/` - Reusable AI skills
  - `workflows/` - Automation workflows

### 5. Development Infrastructure
- `.claude/` - Claude Code configuration and tools
- `specs/` - Feature specifications and plans
- `history/` - Historical records and ADRs
- `tests/` - Test suites
- `docs/` - Documentation (legacy)

## Getting Started

1. **Frontend**: Navigate to `frontend/` and run `npm install && npm start`
2. **Backend**: Navigate to `rag-backend/` and run the FastAPI application
3. **Auth**: Authentication service is available in `auth/betterauth/`
4. **Development**: Use Claude Code agents in `monorepo/agents/`

## Contributing

See `CONTRIBUTING.md` for contribution guidelines.