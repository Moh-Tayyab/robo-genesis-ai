# Research: RAG Chatbot Backend

**Feature**: 001-rag-chatbot
**Date**: 2025-12-06

## Executive Summary

This research document addresses the key unknowns identified in the implementation plan for the RAG Chatbot Backend. The research covers OpenAI Agents SDK integration, uv package manager setup, Qdrant hybrid search, HyDE and MMR implementation, and Neon Postgres connection pooling.

## 1. OpenAI Agents SDK Research

### Decision: OpenAI Assistant API with Custom Tools
- **Rationale**: OpenAI Agents SDK (official name: OpenAI Assistant API) provides the proper framework for creating agents with custom tools and delegation patterns. This aligns with the requirement to use "OpenAI Agents SDK (official latest way — NO langchain, NO llama-index, only agents + tools + handoff)"
- **Implementation**: Use `openai.Assistant` and `openai.Thread` with custom functions as tools
- **Agent Delegation**: Implement tool-based handoff between BookAgent and SelectedTextAgent using function calling

### Key Findings:
- OpenAI Assistant API supports custom tools via function definitions
- Can create multiple assistants (BookAgent and SelectedTextAgent) with different tool sets
- Tool delegation works through function calling mechanism
- Need to use `openai.beta.threads` for proper thread management and message passing
- The Assistant API is the official OpenAI Agents SDK that meets requirements

### Implementation Pattern:
```python
# BookAgent with retrieve_context and finalize_answer tools
book_agent = client.beta.assistants.create(
    name="BookAgent",
    tools=[
        {"type": "function", "function": retrieve_context_schema},
        {"type": "function", "function": finalize_answer_schema}
    ],
    model="gpt-4o-mini"
)

# SelectedTextAgent for exclusive selected text processing
selected_agent = client.beta.assistants.create(
    name="SelectedTextAgent",
    tools=[...],  # Specialized tools for selected text processing
    model="gpt-4o-mini"
)
```

### Alternatives considered:
- LangChain Agents: Explicitly prohibited by requirements
- LlamaIndex: Explicitly prohibited by requirements
- Direct OpenAI API calls: Less structured, no built-in tool management

## 2. uv Package Manager Research

### Decision: Use uv for project initialization and dependency management
- **Rationale**: uv is the fastest Python package installer and resolver, written in Rust. It aligns with the requirement to use uv as package manager
- **Implementation**: Initialize project with `uv init` and manage dependencies with `uv add`

### Key Findings:
- `uv init` creates a new Python project with pyproject.toml
- `uv add package_name` adds dependencies to pyproject.toml and updates uv.lock
- `uv sync` installs dependencies from uv.lock
- `uv run` executes commands in the project environment
- Compatible with standard Python tooling and virtual environments

### Setup Commands:
```bash
uv init
uv add "openai-agents" fastapi uvicorn "psycopg2-binary" "qdrant-client" python-dotenv pydantic "pydantic-settings" "openai" "tiktoken"
```

### Alternatives considered:
- pip + requirements.txt: Standard but slower than uv
- Poetry: More complex configuration than needed
- Conda: Overkill for this project scope

## 3. Qdrant Cloud Hybrid Search Research

### Decision: Use Qdrant's sparse + dense vector search for hybrid search
- **Rationale**: Qdrant Cloud supports hybrid search combining dense vector similarity with sparse keyword matching. This aligns with the requirement to use Qdrant Cloud Free Tier with hybrid search enabled
- **Implementation**: Use both dense vectors (from text-embedding-3-small) and sparse vectors for better retrieval

### Key Findings:
- Qdrant supports hybrid search with `query` parameter accepting both dense and sparse vectors
- Can store metadata (chapter, section, book_version) as payload
- Supports filtering, scoring, and custom ranking
- Free tier supports up to 5GB storage and 1M vectors
- Hybrid search enables both semantic and keyword matching

### Implementation Pattern:
```python
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Hybrid search combining dense and sparse vectors
search_results = client.search(
    collection_name="book_chunks",
    query_vector=models.NamedSparseVector(
        name="sparse",
        vector=models.SparseVector(
            indices=[...],
            values=[...]
        )
    ),
    query_filter=models.Filter(
        must=[
            models.FieldCondition(
                key="chapter",
                match=models.MatchValue(value="chapter_name")
            )
        ]
    ),
    limit=8,  # Top-8 chunks as required
    with_payload=True
)
```

### Alternatives considered:
- Pinecone: Commercial alternative but less flexible than Qdrant
- Weaviate: Good alternative but Qdrant chosen per requirements
- Elasticsearch: More focused on keyword search than vector search

## 4. HyDE and MMR Implementation Research

### Decision: Implement Hypothetical Document Embeddings (HyDE) and Maximum Marginal Relevance (MMR) for diverse, accurate retrieval
- **Rationale**: HyDE improves retrieval by generating hypothetical answers, MMR ensures diversity in retrieved chunks. This meets the requirement for "laser accurate, no hallucination, uses top-8 chunks + HyDE + MMR"
- **Implementation**: Generate hypothetical document for query, embed it, and use for initial retrieval; then apply MMR to select top-8 diverse chunks

### Key Findings:
- HyDE: Generate hypothetical answer to query, embed it, and use for retrieval instead of original query
- MMR: Select documents that are both relevant to query and diverse from already selected documents
- Formula: MMR = λ * similarity_to_query - (1-λ) * max_similarity_to_selected
- Can be implemented using cosine similarity and custom ranking function

### Implementation Pattern:
```python
def retrieve_with_hyde_and_mmr(query: str, top_k: int = 8):
    # Step 1: Generate hypothetical document
    hypothetical_doc = generate_hypothetical_answer(query)
    hypothetical_embedding = embed_text(hypothetical_doc)

    # Step 2: Retrieve initial candidates using hypothetical embedding
    candidates = qdrant_client.search(
        collection_name="book_chunks",
        query_vector=hypothetical_embedding,
        limit=top_k * 2,  # Get more candidates for MMR selection
        with_payload=True
    )

    # Step 3: Apply MMR to select diverse, relevant chunks
    selected = mmr_selection(
        candidates=candidates,
        query_embedding=embed_text(query),
        top_k=top_k,
        diversity=0.7  # Balance between relevance and diversity
    )

    return selected
```

### Alternatives considered:
- Simple vector search: Less accurate than HyDE + MMR approach
- BM25 alone: Good for keyword matching but misses semantic similarity
- Pure semantic search: May miss diverse, relevant content

## 5. Neon Postgres Connection Pooling Research

### Decision: Use asyncpg with connection pooling for Neon Postgres
- **Rationale**: asyncpg provides efficient async connection pooling that works well with Neon's serverless architecture. This aligns with the requirement for "Full async support, connection pooling for Neon"
- **Implementation**: Use asyncpg.create_pool() with proper configuration for Neon

### Key Findings:
- Neon supports connection pooling but has limits (typically 20-200 connections depending on plan)
- asyncpg provides robust async connection pooling for FastAPI applications
- Need to handle connection timeouts and retries properly
- Use connection pool with min/max size and proper timeout settings

### Implementation Pattern:
```python
import asyncpg
from contextlib import asynccontextmanager

class DatabasePool:
    def __init__(self, dsn: str):
        self.dsn = dsn
        self.pool = None

    async def init_pool(self):
        self.pool = await asyncpg.create_pool(
            self.dsn,
            min_size=5,      # Minimum connections in pool
            max_size=20,     # Maximum connections in pool
            command_timeout=60,  # Command timeout
            server_settings={
                "application_name": "rag-chatbot",
                "statement_timeout": "30s"
            }
        )

    @asynccontextmanager
    async def acquire(self):
        async with self.pool.acquire() as conn:
            yield conn
```

### Alternatives considered:
- SQLAlchemy with async engines: More complex ORM approach than needed
- Direct psycopg2: Synchronous, not ideal for FastAPI async architecture
- Motor (for MongoDB): Doesn't match requirement for Neon Postgres

## 6. Selected Text Mode Implementation Research

### Decision: Implement strict context isolation with separate processing pipeline
- **Rationale**: Selected text mode requires complete isolation from book content to prevent leakage. This meets the requirement that "answer comes EXCLUSIVELY from selected text only. If question can't be answered from selection → reply 'I can only answer based on the selected text, and the answer is not present there.'"
- **Implementation**: Separate agent/processing pipeline that only has access to selected text

### Key Findings:
- Need to create a separate processing flow that doesn't access vector store
- Selected text agent should only use the provided text as context
- If question can't be answered from selection, return specific fallback message
- Implement proper validation to ensure text isolation

### Implementation Pattern:
```python
async def process_selected_text_mode(selected_text: str, question: str):
    # Create a context-limited agent that only has access to selected_text
    context = f"Context: {selected_text}\n\nQuestion: {question}"

    response = await client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {
                "role": "system",
                "content": "Answer ONLY based on the provided context. If the answer is not in the context, respond with: 'I can only answer based on the selected text, and the answer is not present there.'"
            },
            {"role": "user", "content": context}
        ],
        max_tokens=500
    )

    return response.choices[0].message.content
```

## Research Conclusion

All unknowns from the technical context have been addressed with specific implementation approaches. The research confirms that the planned architecture is feasible with the required technologies and meets all specified constraints.

Key technical decisions:
1. OpenAI Assistant API for agent implementation (official OpenAI Agents SDK)
2. uv for package management
3. Qdrant hybrid search for vector storage
4. HyDE + MMR for accurate, diverse retrieval
5. asyncpg for Neon connection pooling
6. Strict isolation for selected text mode