# Implementation Plan: RAG Chatbot Backend

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-06 | **Spec**: [link]

**Input**: Feature specification from `/specs/001-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a RAG (Retrieval-Augmented Generation) chatbot backend that integrates with the Docusaurus textbook frontend. The system will use OpenAI's embedding models, Qdrant for vector storage, and Neon Postgres for chat history persistence. The backend will provide both full-book RAG and selected-text Q&A capabilities with ≤1s retrieval latency and ≥90% accuracy on book Q&A pairs.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, OpenAI SDK, Qdrant, Neon Postgres, Pydantic, LangChain
**Storage**: Neon Postgres (chat history), Qdrant Cloud (vector embeddings)
**Testing**: pytest with 80%+ coverage, integration tests with pre-canned Q&A pairs
**Target Platform**: Linux server (Docker container), deployed on Fly.io or Render
**Project Type**: Web application backend service
**Performance Goals**: ≤ 1s p99 retrieval latency, ≥ 90% accuracy on 20 book-Q&A pairs, 1000+ req/s
**Constraints**: ≤ 250ms embedding latency, ≤ 300ms Qdrant search, ≤ 500ms LLM response, ≤ 4000 chars for selected text
**Scale/Scope**: Support 10k+ textbook pages, 1M+ vector embeddings, 10k concurrent users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Content Accuracy & Technical Rigor**: All technical claims about embedding models, vector search, and RAG pipeline accuracy will be validated against authoritative sources
- **Educational Clarity & Accessibility**: API endpoints will be well-documented with clear examples for frontend integration
- **Consistency & Standards**: Code will follow Python PEP 8 standards, with consistent naming and documentation
- **Docusaurus Structure & Quality**: API documentation will be generated and integrated with the textbook
- **Code Example Quality**: All backend code will be tested and functional with safety considerations for API usage
- **Deployment & Publishing Standards**: Service will be containerized and meet performance targets

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
app/
├── main.py              # FastAPI application entry point
├── config.py            # Configuration and settings
├── models/              # Pydantic models for API requests/responses
│   ├── chat.py          # Chat-related data models
│   ├── embedding.py     # Embedding models
│   └── database.py      # Database models
├── services/            # Business logic services
│   ├── embedding_service.py    # Embedding generation and management
│   ├── retrieval_service.py    # Vector search and RAG logic
│   ├── database_service.py     # Database operations
│   └── chat_service.py         # Chat session management
├── api/                 # API route definitions
│   ├── v1/              # API version 1
│   │   ├── chat.py      # Chat endpoints
│   │   ├── ingest.py    # Ingestion endpoints
│   │   └── health.py    # Health check endpoints
│   └── deps.py          # Dependency injection
├── core/                # Core utilities
│   ├── security.py      # Security and rate limiting
│   ├── logging.py       # Logging configuration
│   └── middleware.py    # Request/response middleware
└── tests/               # Test suite
    ├── unit/            # Unit tests
    ├── integration/     # Integration tests
    └── e2e/             # End-to-end tests
```

**Structure Decision**: Web application backend service with modular architecture following FastAPI best practices. The structure separates concerns into models, services, API routes, and core utilities for maintainability and testability.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple external dependencies | RAG pipeline requires specialized libraries for embeddings, vector storage, and LLM integration | Building from scratch would be time-prohibitive and error-prone |
| Multiple data stores | Vector storage (Qdrant) optimized for similarity search, relational storage (Neon) optimized for structured data | Single store would compromise performance for one of the use cases |