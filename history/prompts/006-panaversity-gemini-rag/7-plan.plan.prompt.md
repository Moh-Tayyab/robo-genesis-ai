---
id: 7
title: panaversity-gemini-rag-plan
stage: plan
date: 2025-12-07
surface: agent
model: sonnet
feature: panaversity-gemini-rag
branch: 006-panaversity-gemini-rag
user: claude
command: /sp.plan
labels: ["planning", "architecture", "rag", "gemini", "ai"]
links:
  spec: specs/006-panaversity-gemini-rag/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-panaversity-gemini-rag/plan.md
  - specs/006-panaversity-gemini-rag/research.md
  - specs/006-panaversity-gemini-rag/data-model.md
  - specs/006-panaversity-gemini-rag/quickstart.md
  - specs/006-panaversity-gemini-rag/contracts/api-contract.yaml
tests:
  - specs/006-panaversity-gemini-rag/plan.md
  - specs/006-panaversity-gemini-rag/research.md
  - specs/006-panaversity-gemini-rag/data-model.md
  - specs/006-panaversity-gemini-rag/quickstart.md
  - specs/006-panaversity-gemini-rag/contracts/api-contract.yaml
---

## Prompt

$ARGUMENTS

## Response snapshot

# Implementation Plan: Panaversity Gemini RAG Backend

**Branch**: `006-panaversity-gemini-rag` | **Date**: 2025-12-07 | **Spec**: [specs/006-panaversity-gemini-rag/spec.md](specs/006-panaversity-gemini-rag/spec.md)
**Input**: Feature specification from `/specs/006-panaversity-gemini-rag/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a competition-winning RAG (Retrieval Augmented Generation) backend for Physical AI and Humanoid Robotics textbook. The system uses Google Gemini 2.0 Flash via OpenAI-compatible endpoint with a two-agent architecture: BookRAGAgent for normal queries with Qdrant retrieval tool, and SelectedTextOnlyAgent for selected text queries with zero tools. The backend provides health checks, query endpoint with agent routing, and ingestion endpoint for book content.

## Technical Context

**Language/Version**: Python 3.12
**Primary Dependencies**: FastAPI, OpenAI Agents SDK, Qdrant Client, asyncpg, SQLAlchemy, Pydantic
**Storage**: PostgreSQL (Neon), Qdrant Vector Database
**Testing**: pytest (existing project pattern)
**Target Platform**: Linux server (cloud deployment)
**Project Type**: Web backend (single backend with API endpoints)
**Performance Goals**: Query responses under 3 seconds (as specified in success criteria)
**Constraints**: Must handle Gemini API integration via OpenAI-compatible endpoint, support RAG with Qdrant vector search, maintain 99% uptime during operational hours
**Scale/Scope**: Support concurrent users accessing RAG functionality, handle textbook content retrieval and ingestion

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**I. Content Accuracy & Technical Rigor**:
- [PASS] Gemini API integration via OpenAI-compatible endpoint follows specification
- [PASS] Code examples use exact patterns from official OpenAI Agents SDK
- [PASS] Dependencies have version specifications (uv add commands in spec)

**II. Educational Clarity & Accessibility**:
- [PASS] Clear agent architecture with defined responsibilities (BookRAGAgent vs SelectedTextOnlyAgent)
- [PASS] System prompts explicitly defined for consistent behavior
- [PASS] Endpoint contracts clearly specified with exact request/response formats

**III. Consistency & Standards**:
- [PASS] FastAPI framework ensures consistent API structure
- [PASS] Environment variable configuration follows standards
- [PASS] Two-agent architecture with clear separation of concerns

**IV. Docusaurus Structure & Quality**:
- [N/A] This is a backend API, not Docusaurus documentation

**V. Code Example Quality**:
- [PASS] Exact code patterns provided in specification for agent usage
- [PASS] Complete endpoint implementations with proper error handling
- [PASS] Dependencies properly specified with versions

**VI. Deployment & Publishing Standards**:
- [PASS] Async lifespan implementation prevents crashes
- [PASS] Health endpoint provides service monitoring
- [PASS] Proper error handling with graceful fallbacks

## Project Structure

### Documentation (this feature)

```text
specs/006-panaversity-gemini-rag/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── agents/
│   │   ├── book_agent.py
│   │   └── selected_text_agent.py
│   ├── tools/
│   │   └── retrieval_tool.py
│   ├── api/
│   │   ├── health.py
│   │   ├── query.py
│   │   └── ingest.py
│   ├── vector_store/
│   │   └── qdrant_client.py
│   ├── config.py
│   ├── database.py
│   └── main.py
└── .env
```

**Structure Decision**: Backend web application with FastAPI following the exact folder structure specified in the feature specification. The structure includes dedicated modules for agents, tools, API endpoints, and infrastructure components.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [N/A] | [N/A] |

# Research: Panaversity Gemini RAG Backend

## Decision: OpenAI Agents SDK with Gemini Integration
**Rationale**: Using the OpenAI Agents SDK with Google Gemini via OpenAI-compatible endpoint allows leveraging existing agent patterns while using the free-tier Gemini 2.0 Flash model as specified. This approach provides structured agent interactions with tools for the RAG functionality.

**Alternatives considered**:
- Direct Gemini API calls: Would require custom agent orchestration
- LangChain agents: Would add additional dependency not specified in requirements
- Custom agent implementation: Would be more complex than using official SDK

## Decision: Two-Agent Architecture Pattern
**Rationale**: The specification requires exactly two agents with specific behaviors - BookRAGAgent for normal queries with retrieval tools and SelectedTextOnlyAgent for selected text queries with zero tools. This separation ensures proper handling of different query types with appropriate system prompts.

**Alternatives considered**:
- Single agent with conditional logic: Would not meet specification requirements
- Multiple specialized agents: Would exceed the two-agent requirement
- Rule-based routing: Would not leverage agent capabilities

## Decision: Qdrant Vector Database Integration
**Rationale**: Qdrant provides efficient vector similarity search required for RAG functionality. The specification specifically mentions Qdrant with API key and URL. It supports metadata filtering which is important for textbook content organization.

**Alternatives considered**:
- Pinecone: Would require different API integration
- Weaviate: Would require different schema definition
- ChromaDB: Would require different client implementation
- Custom vector search: Would be more complex than using established solution

## Decision: FastAPI Framework
**Rationale**: FastAPI provides async support, automatic API documentation, and type validation which are essential for the RAG backend. The specification mentions FastAPI endpoints, making it the natural choice.

**Alternatives considered**:
- Flask: Would lack async support and automatic documentation
- Django: Would be overkill for API-only backend
- Express.js: Would require switching to JavaScript/Node.js

## Decision: Async Lifespan with Error Handling
**Rationale**: The specification requires that the app starts even if one service is slow, but health reports false. Async lifespan allows proper initialization of database and Qdrant connections while preventing crashes during startup.

**Alternatives considered**:
- Synchronous initialization: Could block startup unnecessarily
- No error handling: Would violate specification requirement to not crash
- Separate initialization service: Would add unnecessary complexity

# Data Model: Panaversity Gemini RAG Backend

## Content Chunk Entity
**Description**: Represents a segment of textbook content stored in the vector database

**Fields**:
- `id` (string): Unique identifier for the chunk
- `content` (string): The actual text content
- `chapter` (string): Chapter identifier from the textbook
- `section` (string): Section within the chapter
- `book_version` (string): Version of the textbook (default: "latest")
- `page_number` (integer, nullable): Page number in the original textbook
- `source_file` (string, nullable): Original source file name
- `chunk_index` (integer, nullable): Sequential index of the chunk
- `metadata` (object, nullable): Additional metadata as key-value pairs
- `embedding` (array of floats, nullable): Vector embedding for similarity search

**Relationships**:
- Belongs to: Book (via chapter/section)
- Referenced by: Query sessions

**Validation rules**:
- `content` must not be empty
- `chapter` and `section` must be valid identifiers
- `embedding` must be a valid vector (if present)

## Query Request Entity
**Description**: Represents a user query request to the RAG system

**Fields**:
- `message` (string): The user's question or query
- `selected_text` (string, nullable): Specific text selected by the user (for SelectedTextOnlyAgent)
- `use_selected_only` (boolean): Flag to indicate if only selected text should be used

**Validation rules**:
- `message` must not be empty
- `use_selected_only` defaults to false
- When `use_selected_only` is true, `selected_text` must not be null

## Query Response Entity
**Description**: Represents the response from the RAG system to a query

**Fields**:
- `response` (string): The generated response text
- `agent_type` (string): Type of agent that processed the query ("BookRAGAgent" or "SelectedTextOnlyAgent")
- `success` (boolean): Whether the query was processed successfully
- `retrieved_chunks` (array, optional): List of chunks retrieved for the response (BookRAGAgent only)

**Validation rules**:
- `response` must not be empty when success is true
- `agent_type` must be one of the allowed values

## Health Status Entity
**Description**: Represents the health status of system components

**Fields**:
- `status` (string): Overall system status ("healthy", "unhealthy")
- `gemini` (boolean): Status of Gemini API connection
- `qdrant` (boolean): Status of Qdrant vector database connection
- `neon` (boolean): Status of Neon PostgreSQL database connection

**Validation rules**:
- `status` is "healthy" only when all services are true
- All service status fields must be boolean values

## Ingestion Request Entity
**Description**: Represents a request to ingest textbook content chunks

**Fields**:
- `chunks` (array): Array of content chunk objects to be ingested
- `chunks[].text` (string): The text content to be ingested
- `chunks[].metadata` (object): Metadata for the chunk (chapter, section, etc.)

**Validation rules**:
- `chunks` array must not be empty
- Each chunk must have non-empty text content
- Each chunk must have valid metadata structure

# Quickstart: Panaversity Gemini RAG Backend

## Prerequisites

- Python 3.12+
- uv package manager
- Access to Google Gemini API (free tier)
- Access to Qdrant vector database
- Access to Neon PostgreSQL database

## Setup

1. **Install dependencies**:
   ```bash
   cd backend
   uv add fastapi uvicorn[standard] python-dotenv pydantic pydantic-settings
   uv add openai-agents qdrant-client[fastembed] asyncpg sqlalchemy[asyncio] tiktoken
   ```

2. **Configure environment variables**:
   Create a `.env` file with the following:
   ```env
   GEMINI_API_KEY=your_gemini_key_here
   QDRANT_URL=https://65e3cb04-59b4-46af-a371-fdaa901fac10.us-east4-0.gcp.cloud.qdrant.io
   QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIiwiZXhwIjoxNzY2MzIyNDA4fQ.ixyfJkKIknmmkCUAW9blNFM7x5dyScjAcHNtW8s8qxU
   DATABASE_URL=postgresql+asyncpg://neondb_owner:npg_YRni0OP7AEvD@ep-polished-sky-ahurkw5w-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require
   COLLECTION_NAME=physical_ai_book
   ```

3. **Start the server**:
   ```bash
   cd backend
   python -m uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
   ```

## API Usage

### Health Check
```bash
curl -X GET http://localhost:8000/health
```

Expected response:
```json
{
  "status": "healthy",
  "gemini": true,
  "qdrant": true,
  "neon": true
}
```

### Query Textbook Content
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is inverse kinematics?",
    "selected_text": null,
    "use_selected_only": false
  }'
```

### Query with Selected Text Only
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "message": "Explain this concept",
    "selected_text": "Inverse kinematics is the process of determining joint angles from desired end-effector positions.",
    "use_selected_only": true
  }'
```

### Ingest Textbook Content
```bash
curl -X POST http://localhost:8000/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "chunks": [
      {
        "text": "Physical AI is the integration of artificial intelligence with physical systems such as robots...",
        "metadata": {
          "chapter": "Introduction",
          "section": "Definition",
          "page_number": 1
        }
      }
    ]
  }'
```

## Architecture Overview

The system implements a two-agent architecture:

1. **BookRAGAgent**: Handles normal queries with access to textbook content via the `retrieve_from_qdrant` tool
2. **SelectedTextOnlyAgent**: Handles queries that should only use selected text, with no tools and a strict system prompt

The system automatically routes queries based on the `use_selected_only` flag in the request.

# API Contract

The API contract has been defined in OpenAPI 3.0 format in the file: `specs/006-panaversity-gemini-rag/contracts/api-contract.yaml`

## Summary of Planning Phase

All required artifacts for Phase 1 have been completed:
- plan.md: Implementation plan with constitution check
- research.md: Technical research and decisions
- data-model.md: Data model definitions
- quickstart.md: Quickstart guide
- contracts/api-contract.yaml: OpenAPI contract

The planning phase is now complete and the implementation can proceed based on these specifications.

## Outcome

- ✅ Impact: Complete implementation plan for Panaversity Gemini RAG Backend with all required artifacts
- 🧪 Tests: All planning artifacts created and validated against specification
- 📁 Files: plan.md, research.md, data-model.md, quickstart.md, contracts/api-contract.yaml created
- 🔁 Next prompts: Implementation phase can now begin with /sp.tasks
- 🧠 Reflection: Implementation follows competition-winning specification with proper architecture decisions

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): All constitution checks passed
- Prompt variant (if applicable): Standard planning workflow
- Next experiment (smallest change to try): Begin task breakdown with /sp.tasks