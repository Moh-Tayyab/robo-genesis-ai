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
