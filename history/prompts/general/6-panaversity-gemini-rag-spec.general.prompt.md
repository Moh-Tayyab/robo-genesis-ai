---
id: 6
title: panaversity-gemini-rag-spec
stage: spec
date: 2025-12-07
model: sonnet
feature: panaversity-gemini-rag
branch: 006-panaversity-gemini-rag
user: claude
command: /sp.specify PANAVERSITY PHYSICAL AI BOOK – FINAL COMPETITION-WINNING BACKEND SPECIFICATION (250/250 LOCKED) This is the absolute final, judge-approved, zero-error specification. No deviation allowed. 1. Model (FREE TIER – MUST USE THIS EXACT WAY) → Google Gemini 2.0 Flash (or gemini-1.5-flash-latest) via OpenAI-compatible endpoint → Exact OpenAI Agents SDK usage (copy-paste from official example): ```python from agents import Agent, Runner, AsyncOpenAI, OpenAIChatCompletionsModel client = AsyncOpenAI( api_key=GEMINI_API_KEY, base_url="https://generativelanguage.googleapis.com/v1beta/openai/" ) model = OpenAIChatCompletionsModel(model="gemini-2.0-flash", openai_client=client) result = await Runner.run(starting_agent=agent, input=user_input) return result.final_output Required FastAPI Endpoints (EXACT PATHS – NO CHANGE) GET /health → {"status":"healthy","gemini":true,"qdrant":true,"neon":true} POST /query → Main RAG chat endpoint (frontend uses this) POST /ingest → Ingest book chunks (must work perfectly) /query Request Body (EXACT FORMAT)JSON{ "message": "string", "selected_text": "string | null", "use_selected_only": false } Two-Agent Architecture (MUST BE EXACTLY THIS) • BookRAGAgent (normal queries) → Tools: retrieve_from_qdrant(query: str) → Uses Gemini via OpenAI-compatible client• SelectedTextOnlyAgent (handoff when use_selected_only=true) → ZERO tools allowed → System prompt (copy-paste this): "You are an expert assistant. You are strictly forbidden to use any knowledge except the selected text below. If the answer is not present in the selected text, reply exactly: 'I can only answer based on the selected text, and the answer is not present there.' Selected text: {selected_text} Question: {message}" Environment Variables (.env – MUST BE LOADED)envGEMINI_API_KEY=your_gemini_key_here QDRANT_URL=https://65e3cb04-59b4-46af-a371-fdaa901fac10.us-east4-0.gcp.cloud.qdrant.io QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIiwiZXhwIjoxNzY2MzIyNDA4fQ.ixyfJkKIknmmkCUAW9blNFM7x5dyScjAcHNtW8s8qxU DATABASE_URL=postgresql+asyncpg://neondb_owner:npg_YRni0OP7AEvD@ep-polished-sky-ahurkw5w-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require COLLECTION_NAME=physical_ai_book uv Commands (run exactly these)Bashuv add fastapi uvicorn[standard] python-dotenv pydantic pydantic-settings uv add openai-agents qdrant-client[fastembed] asyncpg sqlalchemy[asyncio] tiktoken Final Folder Structure (keep only these, delete everything else)textbackend/ ├── src/ │ ├── agents/ │ │ ├── book_agent.py │ │ └── selected_text_agent.py │ ├── tools/ │ │ └── retrieval_tool.py │ ├── api/ │ │ ├── health.py │ │ ├── query.py │ │ └── ingest.py │ ├── vector_store/qdrant_client.py │ ├── config.py │ ├── database.py │ └── main.py └── .env Lifespan & Health Check (MUST NOT CRASH) → Async lifespan with proper try/except → /health returns 200 + true for all services → App starts even if one service slow (but health reports false) Ingestion (/ingest) Requirements → Accept JSON list of chunks with text + metadata → Use fastembed (no API key needed) → Upload to Qdrant with proper metadata filtering Final Test Cases (MUST PASS 100%) Normal query → correct answer from full book Selected text that contradicts book → answer follows selected text only Selected text with no answer → exact fallback message /health → 200 OK with all true This specification is FINAL, COMPLETE, and has already won the competition for 7 participants.
labels: ["specification", "rag", "gemini", "ai"]
files:
  - specs/006-panaversity-gemini-rag/spec.md
  - specs/006-panaversity-gemini-rag/checklists/requirements.md
tests: []
links:
  spec: specs/006-panaversity-gemini-rag/spec.md
  checklist: specs/006-panaversity-gemini-rag/checklists/requirements.md
  branch: 006-panaversity-gemini-rag
---

# PHR: panaversity-gemini-rag-spec

## Prompt

```
PANAVERSITY PHYSICAL AI BOOK – FINAL COMPETITION-WINNING BACKEND SPECIFICATION (250/250 LOCKED)

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


This specification is FINAL, COMPLETE, and has already won the competition for 7 participants.
```

## Response

Created a comprehensive feature specification for the Panaversity Gemini RAG backend as requested. The specification includes:

- User scenarios for student queries, selected text queries, content ingestion, and health monitoring
- Functional requirements for the two-agent architecture, endpoints, and service integration
- Success criteria with measurable outcomes for response accuracy, performance, and system reliability
- Proper handling of the exact specifications including the two-agent architecture (BookRAGAgent and SelectedTextOnlyAgent)
- The specified system prompt behavior for selected text queries
- Health check requirements with proper service status reporting
- All required endpoints with the exact request/response formats

The specification was created in `specs/006-panaversity-gemini-rag/spec.md` with a quality checklist in `specs/006-panaversity-gemini-rag/checklists/requirements.md`, and the branch `006-panaversity-gemini-rag` was created.

## Outcome

Successfully created the complete feature specification based on the provided requirements. The specification covers all required aspects including the two-agent architecture, endpoint specifications, environment configuration, and proper handling of both general and selected-text queries as specified.

## Evaluation

The specification meets all requirements from the original prompt and is ready for the planning phase.