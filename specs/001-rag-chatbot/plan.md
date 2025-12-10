# Implementation Plan: RAG Chatbot Backend

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-06 | **Spec**: [specs/001-rag-chatbot/spec.md](../001-rag-chatbot/spec.md)
**Input**: User description: "Physical AI & Humanoid Robotics – Integrated RAG Chatbot Backend (implements Module-level chatbot that answers ONLY from book content + selected-text Q&A)"

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a RAG chatbot backend using OpenAI Agents SDK with FastAPI, Neon Serverless Postgres, and Qdrant Cloud. The system will support both full-book Q&A and selected-text Q&A modes with proper context isolation to prevent leakage between modes. Implementation will use top-8 chunk retrieval with HyDE and MMR for accuracy, with proper agent delegation between BookAgent and SelectedTextAgent.

## Technical Context

### Current State
- Existing textbook content in Docusaurus format
- Need for RAG-based Q&A system to help students with textbook content
- Requirement for both full-book Q&A and selected-text Q&A modes with zero leakage
- Need for persistent storage of chat history and retrieval metadata
- Requirement for rate limiting and monitoring

### Technology Stack
- **Language/Version**: Python 3.11+ (as required for uv package manager)
- **Backend Framework**: FastAPI (as specified in requirements)
- **AI Framework**: OpenAI Agents SDK (official latest way, no LangChain/LLamaIndex)
- **Package Manager**: uv (as specified in requirements)
- **Database**: Neon Serverless Postgres (via connection pooling)
- **Vector Database**: Qdrant Cloud Free Tier (hybrid search enabled)
- **OpenAI Model**: gpt-4o-mini (as specified in spec)
- **Embedding Model**: text-embedding-3-small (as specified in spec)
- **Primary Dependencies**: FastAPI, OpenAI Agents SDK, Qdrant Client, Neon Postgres, uv
- **Testing**: pytest with >= 80% coverage
- **Target Platform**: Linux server (Docker container)

### Known Unknowns (NEEDS CLARIFICATION)
- **OpenAI Agents SDK Integration**: How to properly implement the BookAgent with the specified tools and delegation pattern
- **SelectedTextAgent Implementation**: How to create a specialized agent that works exclusively with selected text
- **HyDE (Hypothetical Document Embeddings) Implementation**: How to properly implement HyDE for better retrieval accuracy
- **MMR (Maximum Marginal Relevance) Integration**: How to implement MMR for top-8 chunk selection
- **Connection Pooling with Neon**: Specific implementation details for Neon Postgres connection pooling
- **Qdrant Hybrid Search**: How to leverage hybrid search capabilities in Qdrant Cloud
- **Proper Tool Delegation**: How to implement the handoff between main BookAgent and SelectedTextAgent

### Architecture Overview
- Main BookAgent with two primary tools:
  - retrieve_context(query: str, use_selected: bool = False, selected_text: str = "")
  - finalize_answer(context: str, question: str, is_selected_mode: bool)
- Specialized SelectedTextAgent for selected-text mode
- Vector ingestion pipeline with metadata (chapter, section, book_version)
- Async FastAPI endpoints with proper error handling
- Neon Postgres connection pooling for chat history and metadata
- Qdrant for vector storage and retrieval

### Performance Goals
- ≤ 1 second retrieval latency for 95% of requests
- ≥ 90% accuracy on 20 book-Q&A pairs during testing
- Top-8 chunk retrieval with HyDE + MMR for accuracy
- Proper rate limiting (30 req/min per IP, 10 req/min per session)

### Constraints
- ≤ 1 s retrieval latency requirement
- Selected text mode must answer EXCLUSIVELY from selected text only
- If question can't be answered from selection → reply "I can only answer based on the selected text, and the answer is not present there."
- ≤ 4000 chars selected text length
- ≤ 250ms embedding latency
- No runtime web search (only uses book content)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principles Verification
- **I. Content Accuracy & Technical Rigor**: Implementation will use proper OpenAI Agents SDK, validated vector retrieval methods, and accurate metadata handling
- **II. Educational Clarity & Accessibility**: API endpoints will be well-documented and clearly differentiated between full-book and selected-text modes
- **III. Consistency & Standards**: Will follow FastAPI best practices and standard OpenAPI specifications
- **IV. Docusaurus Structure & Quality**: Backend will provide proper endpoints for frontend integration
- **V. Code Example Quality**: Implementation will include proper error handling, documentation, and type hints
- **VI. Deployment & Publishing Standards**: Will include health checks, monitoring endpoints, and proper configuration management

### Gate Evaluation
- ✅ **Technical Rigor**: Plan addresses proper OpenAI Agents SDK usage and RAG implementation
- ✅ **Educational Clarity**: Clear separation between full-book and selected-text modes
- ✅ **Standards Compliance**: Following FastAPI, OpenAPI, and standard Python practices
- ✅ **Production Quality**: Includes monitoring, health checks, and error handling

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
backend/
├── src/
│   ├── models/          # Pydantic models and database schemas
│   ├── services/        # Business logic and agent implementations
│   ├── api/             # FastAPI endpoints
│   ├── database/        # Database connection and repository patterns
│   ├── vector_store/    # Qdrant integration and vector operations
│   └── utils/           # Utility functions and helpers
├── tests/
│   ├── unit/            # Unit tests for individual components
│   ├── integration/     # Integration tests for API endpoints
│   └── contract/        # Contract tests for API compliance
├── requirements.txt     # Python dependencies (managed via uv)
├── pyproject.toml       # Project configuration for uv
└── docker/              # Docker configuration files
```

**Structure Decision**: Backend service structure chosen to separate concerns with clear boundaries between models, services, API, database, and vector store layers. This enables proper testing and maintainability while following Python/ FastAPI best practices.

## Phase 0: Research & Analysis

### Research Tasks

#### 1. OpenAI Agents SDK Research
- Research proper implementation of OpenAI Agents with custom tools
- Understand the agent delegation pattern between BookAgent and SelectedTextAgent
- Document best practices for tool creation and usage

#### 2. uv Package Manager Research
- Research uv as package manager and how to properly initialize project
- Understand dependency management and lock file creation
- Document best practices for Python project setup with uv

#### 3. Qdrant Cloud Hybrid Search Research
- Research hybrid search capabilities in Qdrant Cloud
- Understand how to implement proper vector and keyword search combination
- Document metadata handling for chapter, section, book_version

#### 4. HyDE and MMR Implementation Research
- Research Hypothetical Document Embeddings implementation
- Research Maximum Marginal Relevance for chunk selection
- Document how to achieve top-8 chunk selection with proper diversity

#### 5. Neon Postgres Connection Pooling Research
- Research connection pooling best practices with Neon Serverless
- Understand async database operations with FastAPI
- Document proper session management and connection handling

## Phase 1: Design & Architecture

### 1. Data Model Design

#### Entities
- **User**: Anonymous user identified by UUID for tracking purposes
- **Session**: Chat session with metadata about the conversation
- **Message**: Individual user or assistant message in a conversation
- **Retrieval**: Record of which content chunks were retrieved for a message
- **DocumentChunk**: Vector-embedded content with metadata (chapter, section, book_version)

#### Relationships
- User has many Sessions
- Session has many Messages
- Message has many Retrievals
- Retrieval connects to DocumentChunks

### 2. API Contract Design

#### Endpoints
- `POST /chat` - Full-book RAG endpoint
- `POST /chat/selected` - Selected-text RAG endpoint
- `POST /ingest` - Content ingestion endpoint
- `GET /health` - Health monitoring endpoint

#### Request/Response Models
- Chat requests with question and optional selected text
- Chat responses with answer and source citations
- Ingest requests with document content and metadata
- Health responses with system status

### 3. Agent Architecture Design

#### BookAgent
- Main orchestrator agent
- Uses retrieve_context and finalize_answer tools
- Delegates to SelectedTextAgent when use_selected_only = True

#### SelectedTextAgent
- Specialized agent for selected-text mode
- Works exclusively with provided selected text
- Returns "I can only answer based on the selected text, and the answer is not present there." when question can't be answered

#### Tools
- retrieve_context: Handles vector retrieval with proper parameters
- finalize_answer: Processes context and generates final response

## Phase 2: Implementation Plan

### 1. Project Setup
- Initialize project with uv
- Add required dependencies
- Set up configuration and environment management

### 2. Database Layer
- Implement Neon Postgres connection pooling
- Create data models and repository classes
- Implement async database operations

### 3. Vector Store Layer
- Implement Qdrant client configuration
- Create document chunking and ingestion pipeline
- Implement HyDE and MMR for retrieval

### 4. Agent Layer
- Implement BookAgent with proper tool delegation
- Create SelectedTextAgent for selected-text mode
- Implement tool functions for retrieval and finalization

### 5. API Layer
- Create FastAPI application with proper routing
- Implement endpoints with validation and error handling
- Add rate limiting and monitoring

### 6. Integration & Testing
- Integrate all layers together
- Implement comprehensive tests
- Performance and accuracy validation

## Phase 3: Deployment & Operations

### 1. Configuration Management
- Environment variable handling
- Service configuration for different environments
- Secret management for API keys

### 2. Monitoring & Observability
- Health check endpoints
- Performance metrics
- Error tracking and logging

### 3. Security & Rate Limiting
- IP-based rate limiting
- Session-based rate limiting
- Input validation and sanitization

## Risks & Mitigation

### 1. Performance Risks
- **Risk**: Vector retrieval latency exceeds 1s requirement
- **Mitigation**: Optimize Qdrant queries, implement caching, use proper indexing

### 2. Accuracy Risks
- **Risk**: Hallucination in responses despite RAG constraints
- **Mitigation**: Proper context isolation, HyDE implementation, MMR for diverse retrieval

### 3. Scalability Risks
- **Risk**: Database connection limits under high load
- **Mitigation**: Proper connection pooling, async operations, load testing

### 4. Selected Text Isolation
- **Risk**: Leakage between selected-text mode and full-book mode
- **Mitigation**: Strict context isolation, separate agent implementation, comprehensive testing

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| OpenAI Agents SDK | Need proper agent orchestration with tool delegation | Direct OpenAI API calls insufficient for required agent handoff pattern |
| Qdrant hybrid search | Need advanced retrieval capabilities beyond basic vector search | Simple vector search may not provide required accuracy |
