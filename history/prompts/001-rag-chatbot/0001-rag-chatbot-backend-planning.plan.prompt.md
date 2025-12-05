
---
id: 0001
title: RAG Chatbot Backend Planning
stage: plan
date_iso: '2025-12-06'
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-chatbot
branch: feat/001-rag-chatbot
user: Claude
command: /sp.plan
labels:
  - rag
  - chatbot
  - backend
  - planning
  - architecture
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - specs/001-rag-chatbot/plan.md
  - specs/001-rag-chatbot/research.md
  - specs/001-rag-chatbot/data-model.md
  - specs/001-rag-chatbot/quickstart.md
  - specs/001-rag-chatbot/contracts/api-spec.md
  - CLAUDE.md
tests_yaml: |
  - None
---

# RAG Chatbot Backend Planning

## Prompt Text

```
/sp.plan
Project: Physical AI & Humanoid Robotics – Integrated RAG Chatbot Backend
(implements Module-level chatbot that answers ONLY from book content + selected-text Q&A)

Business Requirements (from spec):
- Embed RAG chatbot in Docusaurus textbook; OpenAI Agents SDK, FastAPI, Neon Postgres, Qdrant Cloud Free Tier.
- ≤ 1 s retrieval latency, ≥ 90 % accuracy on 20 book-Q&A pairs, selected-text Q&A with “not in selection” fallback.
- Store anonymised {userId, chatHistory, retrievalMetadata} in Neon; no runtime web search; 2-week hackathon deadline.

Technical Requirements Plan
1. Architecture & Repo Layout
   - `/app/` – FastAPI (Python 3.11) micro-service, Docker-ready
   - `/scripts/` – chunker + ingest CLI (Markdown → Qdrant)
   - `/specs/001-rag-chatbot/` – live spec, checklist, ADR if any
   - `/.github/workflows/` – CI: pytest, lint, build & deploy FastAPI container

2. Data Models (Neon Postgres)
   - users (userId UUID PK, createdAt)
   - sessions (sessionId UUID PK, userId FK, startedAt, endedAt)
   - messages (msgId UUID PK, sessionId FK, role enum('user','assistant'), text, createdAt)
   - retrievals (retrievalId UUID PK, msgId FK, chunkId, chapter, section, url, latencyMs, createdAt)

3. Qdrant Cloud Schema (Free Tier)
   - collection: “book-chunks”
   - vectors: 1536-dim (text-embedding-3-small)
   - payload: {chunkId, chapter, section, url, difficulty, lang, textHash}

4. Core API Endpoints
   - POST /chat – full-book RAG (streaming JSON)
   - POST /chat/selected – selected-text RAG (enforces context boundary)
   - POST /ingest – idempotent Markdown → chunks → embeddings → Qdrant
   - GET  /health – uptime + Qdrant connectivity

5. RAG Pipeline (per query)
   a. Receive {question, optional selectedText, sessionId}
   b. Embed question (selectedText if present) → 1536 vector
   c. Qdrant search: top-k=5, score_threshold ≥ 0.75, filter by lang/difficulty if given
   d. If selectedText → rerank against selectedText (cross-encoder miniLM, keep ≥ 0.85 overlap)
   e. Build prompt: “Answer solely from below chunks; cite chapter+url; if insufficient say ‘Not in selection.’”
   f. Stream answer via OpenAI Agents SDK (gpt-4o-mini)
   g. Log messages + retrievals to Neon (async pool)

6. Safety & Guardrails
   - Prompt injection filter (rudimentary regex + LLM guard)
   - No hardware-control commands allowed (blocklist: “move”, “goto”, “torque”)
   - Rate-limit: 30 req/min per IP, 10 req/min per session
   - Return 400 if selectedText length &gt; 4 000 chars

7. Performance Budget
   - Embedding latency ≤ 250 ms (OpenAI)
   - Qdrant search ≤ 300 ms (regional cluster)
   - LLM first token ≤ 500 ms
   - Total p99 ≤ 1 s (excludes network)

8. Testing Strategy
   - Unit: pytest ≥ 80 % coverage (embed, retrieve, prompt builder)
   - Integration: 20 pre-canned Q&A pairs → assert cosine ≥ 0.90 vs expected chunk
   - E2E: playwright tests in CI (selected-text boundary, “not in selection” reply, health)

9. Incremental Ingest (Docusaurus hook)
   - On markdown file change → github action calls /ingest with file path
   - Idempotent: chunk hash → skip if exists, update if text changed, delete orphans

10. Deployment & CI/CD
    - Container: `ghcr.io/&lt;org&gt;/rag-bot:latest` ( distroless python, ~120 MB )
    - Platform: Fly.io free tier (2x shared CPU, 512 MB) or Render
    - Secrets: `.env.example` lists `OPENAI_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, `NEON_DB_URL`
    - GitHub Actions:
      - on PR: pytest + lint + build image
      - on merge: deploy container, run smoke-test against /health

11. Milestones (2-week sprint)
    W1-D1  – scaffold repo, Dockerfile, FastAPI skeleton
    W1-D3  – Neon schema migrations, basic CRUD
    W1-D5  – Qdrant connect, ingest script MVP
    W1-D7  – /chat endpoint + streaming + logging
    W2-D2  – /chat/selected + cross-encoder filter
    W2-D4  – Docusaurus embed script + UI button
    W2-D6  – 20 E2E tests pass, CI green, demo video
    W2-D7  – buffer day & documentation freeze

12. Risks & Mitigations
    - Qdrant free-tier limit → add LRU cache (redis-lite) and early-warning metric
    - OpenAI rate-limit → fallback to Claude Code Router (config switch)
    - Large markdown files → chunker streams, 5 MB limit per file

13. Post-MVP (optional)
    - Urdu translation filter (lang='ur')
    - User preference agent (uses Neon profile)
    - Analytics dashboard (Metabase on Neon)

Acceptance Gate: PR checklist references this plan; all tasks linked to issues; no merge without green CI + review.
```

## Response Text

Executed the planning process for the RAG chatbot backend, creating all required planning artifacts:

1. Created `specs/001-rag-chatbot/plan.md` with complete implementation plan including technical context, constitution check, and project structure
2. Created `specs/001-rag-chatbot/research.md` with technology decisions and rationale
3. Created `specs/001-rag-chatbot/data-model.md` with complete database and Qdrant schema
4. Created `specs/001-rag-chatbot/quickstart.md` with setup and deployment instructions
5. Created `specs/001-rag-chatbot/contracts/api-spec.md` with complete API specification
6. Updated `CLAUDE.md` to include new technologies and recent changes

The plan follows all constitution principles and provides a complete roadmap for implementing the RAG chatbot backend with FastAPI, OpenAI, Qdrant, and Neon Postgres.

## Outcome

✅ Planning completed successfully:
  • All required planning artifacts created in specs/001-rag-chatbot/
  • Implementation plan with technical context and architecture
  • Data models for both Postgres and Qdrant
  • API contracts and quickstart guide
  • Agent context updated with new technologies
  • Constitution compliance verified