# Implementation Tasks: Panaversity Gemini RAG Backend

**Feature**: Panaversity Gemini RAG Backend
**Branch**: `006-panaversity-gemini-rag`
**Spec**: [specs/006-panaversity-gemini-rag/spec.md](specs/006-panaversity-gemini-rag/spec.md)
**Plan**: [specs/006-panaversity-gemini-rag/plan.md](specs/006-panaversity-gemini-rag/plan.md)

## Implementation Strategy

The implementation follows a user story-driven approach where each story represents a complete, independently testable increment. We'll implement the foundational components first (setup and infrastructure), then develop each user story in priority order (P1, P2, P3), and finish with polish and cross-cutting concerns.

**MVP Scope**: User Story 1 (Student asks questions about textbook content) - This provides core RAG functionality with BookRAGAgent and basic query endpoint.

## Phase 1: Setup & Project Initialization

**Goal**: Initialize project structure and install dependencies per specification

- [X] T001 Create backend directory structure as specified in plan
- [X] T002 Install required dependencies using uv commands from spec
- [X] T003 Create .env file with required environment variables from spec
- [X] T004 Set up initial FastAPI application structure in main.py

## Phase 2: Foundational Infrastructure

**Goal**: Implement foundational components that all user stories depend on

- [X] T005 [P] Implement configuration management with Pydantic Settings per plan
- [X] T006 [P] Implement database connection and session management with Neon PostgreSQL
- [X] T007 [P] Implement Qdrant client for vector database operations
- [X] T008 [P] Implement async lifespan with error handling for graceful startup/shutdown
- [X] T009 [P] Create content chunk model and validation per data model
- [X] T010 [P] Implement Gemini API integration with OpenAI-compatible endpoint

## Phase 3: [US1] Student asks questions about Physical AI textbook content (P1)

**Goal**: Enable students to ask questions about textbook content and receive accurate answers based on the book using RAG

**Independent Test**: A student can ask a question about the textbook content and receive a response that is accurate and grounded in the book's information. The system retrieves relevant content from the vector database and generates a response using the Gemini model.

- [X] T011 [P] [US1] Implement BookRAGAgent with Gemini integration per spec
- [X] T012 [P] [US1] Implement retrieve_from_qdrant tool for Qdrant vector search
- [X] T013 [US1] Create query request/response models per data model
- [X] T014 [US1] Implement POST /query endpoint that accepts exact request format from spec
- [X] T015 [US1] Implement agent routing logic to use BookRAGAgent by default
- [X] T016 [US1] Test User Story 1 acceptance scenarios 1-3

## Phase 4: [US2] Student queries specific selected text only (P2)

**Goal**: Allow students to ask questions about specific selected text with responses based only on that text

**Independent Test**: A student can select specific text and ask a question about it, and the system will respond only based on that text, providing the exact fallback message when the answer is not present in the selected text.

- [X] T017 [P] [US2] Implement SelectedTextOnlyAgent with exact system prompt from spec
- [X] T018 [US2] Ensure SelectedTextOnlyAgent has zero tools per spec
- [X] T019 [US2] Update query endpoint to route to SelectedTextOnlyAgent when use_selected_only=true
- [X] T020 [US2] Test User Story 2 acceptance scenarios 1-3

## Phase 5: [US3] Content creator ingests new textbook content (P2)

**Goal**: Enable educators or content creators to add new content chunks for students to query

**Independent Test**: A content creator can upload a list of content chunks with metadata, and those chunks become available for RAG queries immediately.

- [X] T021 [P] [US3] Create ingestion request/response models per data model
- [X] T022 [P] [US3] Implement POST /ingest endpoint that accepts JSON list of chunks
- [X] T023 [US3] Implement fastembed integration for generating vector embeddings
- [X] T024 [US3] Implement upload to Qdrant with proper metadata filtering per spec
- [X] T025 [US3] Test User Story 3 acceptance scenarios 1-3

## Phase 6: [US4] System health monitoring (P3)

**Goal**: Enable system administrators to monitor health of all services (Gemini API, Qdrant, Neon database)

**Independent Test**: An admin can call the health endpoint and receive a status report indicating which services are operational and which are experiencing issues.

- [X] T026 [US4] Implement GET /health endpoint with exact response format from spec
- [X] T027 [US4] Implement Gemini API health check without crashing app
- [X] T028 [US4] Implement Qdrant health check without crashing app
- [X] T029 [US4] Implement Neon database health check without crashing app
- [X] T030 [US4] Test User Story 4 acceptance scenarios 1-3

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Final implementation details and quality improvements

- [X] T031 Implement error handling with graceful fallbacks per spec requirement R8
- [X] T032 Add proper logging throughout the application
- [X] T033 Optimize for query response time under 3 seconds per success criteria
- [X] T034 Perform final integration testing across all user stories
- [X] T035 Update documentation and quickstart guide with implementation details

## Dependencies

**User Story Completion Order**: US1 (P1) → US2 (P2) → US3 (P2) → US4 (P3)

**Technical Dependencies**:
- T005 (Configuration) blocks T006-T007
- T006-T007 (Infrastructure) blocks T011-T012 (Agents)
- T011-T012 (Agents) blocks T014 (Query endpoint)
- T014 (Query endpoint) required for US1 completion
- T022 (Ingest endpoint) required for US3 completion

## Parallel Execution Examples

**User Story 1 (P1)**: T011 & T012 can run in parallel, followed by T013-T016 sequentially

**User Story 2 (P2)**: T017 & T018 can run in parallel, followed by T019-T020

**User Story 3 (P2)**: T021 & T023 can run in parallel, followed by T022 & T024