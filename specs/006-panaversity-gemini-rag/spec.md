# Feature Specification: Panaversity Gemini RAG Backend

**Feature Branch**: `006-panaversity-gemini-rag`
**Created**: 2025-12-07
**Status**: Draft
**Input**: User description: "PANAVERSITY PHYSICAL AI BOOK – FINAL COMPETITION-WINNING BACKEND SPECIFICATION (250/250 LOCKED)

This is the absolute final, judge-approved, zero-error specification. No deviation allowed.

1. Model (FREE TIER – MUST USE THIS EXACT WAY)
   → Google Gemini 2.0 Flash (or gemini-1.5-flash-latest) via OpenAI-compatible endpoint
   → Exact OpenAI Agents SDK usage (copy-paste from official example):
     ```python
     from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel
     client = AsyncOpenAI(
         api_key=GEMINI_API_KEY,
         base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
     )
     model = OpenAIChatCompletionsModel(model="gemini-2.0-flash", openai_client=client)
     result = await Runner.run(starting_agent=agent, input=user_input)
     return result.final_output

Required FastAPI Endpoints (EXACT PATHS – NO CHANGE)
GET  /health                → {"status":"healthy","gemini":true,"qdrant":true,"neon":true}
POST /query                 → Main RAG chat endpoint (frontend uses this)
POST /ingest                → Ingest book chunks (must work perfectly)
/query Request Body (EXACT FORMAT)JSON{
  "message": "string",
  "selected_text": "string | null",
  "use_selected_only": false
}
Two-Agent Architecture (MUST BE EXACTLY THIS)
• BookRAGAgent (normal queries)
→ Tools: retrieve_from_qdrant(query: str)
→ Uses Gemini via OpenAI-compatible client• SelectedTextOnlyAgent (handoff when use_selected_only=true)
→ ZERO tools allowed
→ System prompt (copy-paste this):
"You are an expert assistant. You are strictly forbidden to use any knowledge except the selected text below.
If the answer is not present in the selected text, reply exactly:
'I can only answer based on the selected text, and the answer is not present there.'
Selected text: {selected_text}
Question: {message}"
Environment Variables (.env – MUST BE LOADED)envGEMINI_API_KEY=your_gemini_key_here
QDRANT_URL=https://65e3cb04-59b4-46af-a371-fdaa901fac10.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIiwiZXhwIjoxNzY2MzIyNDA4fQ.ixyfJkKIknmmkCUAW9blNFM7x5dyScjAcHNtW8s8qxU
DATABASE_URL=postgresql+asyncpg://neondb_owner:npg_YRni0OP7AEvD@ep-polished-sky-ahurkw5w-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require
COLLECTION_NAME=physical_ai_book
uv Commands (run exactly these)Bashuv add fastapi uvicorn[standard] python-dotenv pydantic pydantic-settings
uv add openai-agents qdrant-client[fastembed] asyncpg sqlalchemy[asyncio] tiktoken
Final Folder Structure (keep only these, delete everything else)textbackend/
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
│   ├── vector_store/qdrant_client.py
│   ├── config.py
│   ├── database.py
│   └── main.py
└── .env
Lifespan & Health Check (MUST NOT CRASH)
→ Async lifespan with proper try/except
→ /health returns 200 + true for all services
→ App starts even if one service slow (but health reports false)
Ingestion (/ingest) Requirements
→ Accept JSON list of chunks with text + metadata
→ Use fastembed (no API key needed)
→ Upload to Qdrant with proper metadata filtering
Final Test Cases (MUST PASS 100%)
Normal query → correct answer from full book
Selected text that contradicts book → answer follows selected text only
Selected text with no answer → exact fallback message
/health → 200 OK with all true


This specification is FINAL, COMPLETE, and has already won the competition for 7 participants."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student asks questions about Physical AI textbook content (Priority: P1)

Students using the Physical AI & Humanoid Robotics textbook need to ask questions about the content and receive accurate answers based on the book. The system should leverage RAG (Retrieval Augmented Generation) to provide contextually relevant responses from the book's content.

**Why this priority**: This is the core functionality of the RAG system - enabling students to get accurate answers to their questions from the textbook content, which is the primary value proposition of the feature.

**Independent Test**: A student can ask a question about the textbook content and receive a response that is accurate and grounded in the book's information. The system retrieves relevant content from the vector database and generates a response using the Gemini model.

**Acceptance Scenarios**:

1. **Given** a student has access to the textbook content, **When** they ask a question about Physical AI concepts, **Then** the system provides an accurate answer based on the book's content with relevant citations
2. **Given** a student asks a question about specific chapters or topics in the book, **When** they submit their query to the system, **Then** the system retrieves relevant chunks and generates a comprehensive response
3. **Given** a student asks a question that requires complex reasoning from multiple parts of the book, **When** they submit their query, **Then** the system provides a well-structured response incorporating information from different sections

---

### User Story 2 - Student queries specific selected text only (Priority: P2)

Students need to ask questions about specific text they have selected/highlighted, and the system should answer only based on that specific text, ignoring the broader book content.

**Why this priority**: This enables focused analysis of specific content sections, which is important for detailed study and comprehension of particular concepts without being influenced by broader context.

**Independent Test**: A student can select specific text and ask a question about it, and the system will respond only based on that text, providing the exact fallback message when the answer is not present in the selected text.

**Acceptance Scenarios**:

1. **Given** a student has selected specific text in the textbook, **When** they ask a question that can be answered from the selected text, **Then** the system provides an answer based solely on that text
2. **Given** a student has selected specific text that contradicts information elsewhere in the book, **When** they ask a question with use_selected_only=true, **Then** the system follows the selected text only, not the broader book knowledge
3. **Given** a student has selected text that doesn't contain the answer to their question, **When** they ask a question with use_selected_only=true, **Then** the system responds with exactly: "I can only answer based on the selected text, and the answer is not present there."

---

### User Story 3 - Content creator ingests new textbook content (Priority: P2)

Educators or content creators need to add new content chunks to the system so that students can ask questions about updated material.

**Why this priority**: This enables continuous content updates and expansion of the knowledge base that students can query, making the system more valuable over time.

**Independent Test**: A content creator can upload a list of content chunks with metadata, and those chunks become available for RAG queries immediately.

**Acceptance Scenarios**:

1. **Given** content chunks with text and metadata are provided, **When** they are submitted to the ingestion endpoint, **Then** they are successfully stored in Qdrant with proper metadata for filtering
2. **Given** new content has been ingested, **When** a student asks questions about that content, **Then** the system can retrieve and respond using the newly added information
3. **Given** multiple content chunks with various metadata, **When** they are ingested in batch, **Then** all chunks are stored correctly with metadata preserved

---

### User Story 4 - System health monitoring (Priority: P3)

System administrators need to monitor the health of all services (Gemini API, Qdrant vector store, Neon database) to ensure the RAG system is operational.

**Why this priority**: Critical for maintaining system reliability and quickly identifying service outages or performance issues that affect student experience.

**Independent Test**: An admin can call the health endpoint and receive a status report indicating which services are operational and which are experiencing issues.

**Acceptance Scenarios**:

1. **Given** all services are operational, **When** the health endpoint is called, **Then** it returns status "healthy" with all services showing as true (gemini: true, qdrant: true, neon: true)
2. **Given** one service is down, **When** the health endpoint is called, **Then** it returns appropriate status with the failed service showing as false while the app continues running
3. **Given** services are slow to respond, **When** the health endpoint is called, **Then** the app starts normally but health reports the slow service appropriately

---

## Functional Requirements *(mandatory)*

### R1: Gemini Integration
The system MUST use Google Gemini 2.0 Flash via OpenAI-compatible endpoint with the specified configuration for all LLM interactions.

### R2: Two-Agent Architecture
The system MUST implement exactly two agents:
- BookRAGAgent for normal queries with access to retrieve_from_qdrant tool
- SelectedTextOnlyAgent for selected text queries with NO tools and the specified system prompt

### R3: Query Endpoint
The system MUST provide a POST /query endpoint that:
- Accepts the exact request body format: {message: string, selected_text: string | null, use_selected_only: boolean}
- Routes to appropriate agent based on use_selected_only flag
- Returns responses in a format suitable for the frontend

### R4: Ingestion Endpoint
The system MUST provide a POST /ingest endpoint that accepts JSON list of content chunks and stores them in Qdrant with proper metadata.

### R5: Health Endpoint
The system MUST provide a GET /health endpoint that returns service health status without crashing the application.

### R6: Environment Configuration
The system MUST load all required environment variables (GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL) and handle missing values gracefully.

### R7: Lifespan Management
The system MUST implement async lifespan with proper error handling to prevent crashes during startup/shutdown.

### R8: Error Handling
The system MUST provide graceful fallbacks and error messages when individual services are unavailable.

## Non-Functional Requirements *(optional)*

### Performance Requirements
- Query responses should return within 3 seconds for interactive use
- Ingestion of content chunks should complete within acceptable batch processing times

### Security Requirements
- API keys and credentials must be stored securely in environment variables
- No sensitive data should be exposed in logs or responses

### Scalability Requirements
- System should support concurrent users accessing the RAG functionality

## Assumptions *(optional)*

- The Qdrant vector store and Neon database are properly configured and accessible
- The Gemini API endpoint is stable and responsive
- The frontend application is configured to communicate with the backend endpoints
- Content chunks are properly formatted with appropriate metadata
- Students have appropriate access permissions to the textbook content

## Dependencies *(optional)*

- Google Gemini 2.0 Flash API (via OpenAI-compatible endpoint)
- Qdrant vector database for content storage and retrieval
- Neon database for application data
- FastAPI for web framework
- OpenAI Agents SDK for agent architecture

## Success Criteria *(mandatory)*

### Primary Success Metrics
1. Students can ask questions about textbook content and receive accurate, contextually relevant answers
2. The system correctly handles both general queries and selected text-only queries with the specified behavior
3. Content creators can successfully ingest new textbook content for querying
4. All services (Gemini, Qdrant, Neon) are properly monitored and reported in health checks
5. The system provides the exact fallback message when answers are not available in selected text

### Measurable Outcomes
1. Query response time is under 3 seconds for typical textbook questions
2. Accuracy of responses is high enough to support student learning (90%+ of responses are relevant and accurate)
3. Content ingestion completes successfully for 100% of properly formatted content chunks
4. Health endpoint returns status within 1 second
5. System maintains 99% uptime during operational hours
6. The system handles both agent types correctly: normal queries use BookRAGAgent with RAG, selected text queries use SelectedTextOnlyAgent with zero tools

## Clarifications

### Session 2025-12-07

- Q: What should be the target query response time for better user experience? → A: Under 3 seconds

## Key Entities *(optional)*

### Content Chunk
- Text content from textbook
- Metadata including chapter, section, page number, source file
- Vector embeddings for semantic search

### Query Session
- Student's question
- Retrieved context chunks
- Generated response
- Source citations

### Agent
- BookRAGAgent for general knowledge queries
- SelectedTextOnlyAgent for specific text queries
- Tools and system prompts as specified