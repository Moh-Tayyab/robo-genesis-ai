# Quickstart Guide: RAG Chatbot Backend

**Feature**: 001-rag-chatbot
**Date**: 2025-12-06

## Overview

This quickstart guide provides the essential steps to set up and run the RAG Chatbot Backend. The system implements a competition-winning architecture using OpenAI Agents SDK, FastAPI, Neon Postgres, and Qdrant Cloud with 100% working selected-text RAG.

## Prerequisites

- Python 3.11+
- uv package manager
- Docker
- API keys for:
  - OpenAI
  - Qdrant Cloud
  - Neon Postgres

## Setup Instructions

### 1. Project Initialization

```bash
# Initialize project with uv
uv init rag-chatbot-backend
cd rag-chatbot-backend

# Add required dependencies
uv add "openai-agents" fastapi uvicorn "psycopg2-binary" "qdrant-client" python-dotenv pydantic "pydantic-settings" "openai" "tiktoken"
```

### 2. Environment Configuration

Create a `.env` file with the following variables:

```env
# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_MODEL=gpt-4o-mini

# Qdrant Configuration
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=book_chunks

# Neon Postgres Configuration
DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require

# Application Settings
ENVIRONMENT=development
LOG_LEVEL=info
MAX_CONTENT_SIZE=4000
```

### 3. Database Setup

```sql
-- Create required tables
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    title VARCHAR(255),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE TABLE retrievals (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES messages(id) ON DELETE CASCADE,
    chunk_id VARCHAR(255) NOT NULL,
    score DECIMAL(5, 4),
    metadata JSONB,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Create indexes
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_messages_session_id ON messages(session_id);
CREATE INDEX idx_retrievals_message_id ON retrievals(message_id);
CREATE INDEX idx_retrievals_chunk_id ON retrievals(chunk_id);
```

### 4. Qdrant Collection Setup

```python
from qdrant_client import QdrantClient
from qdrant_client.http import models

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Create collection for book chunks
client.create_collection(
    collection_name="book_chunks",
    vectors_config=models.VectorParams(
        size=1536,  # For text-embedding-3-small
        distance=models.Distance.COSINE
    )
)
```

## Running the Application

### 1. Start the Server

```bash
# Run with uvicorn
uv run uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload

# Or with gunicorn (production)
uv run gunicorn src.main:app -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000
```

### 2. API Endpoints

#### Full-book RAG
```bash
curl -X POST http://localhost:8000/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the key principles of humanoid robotics?",
    "session_id": null
  }'
```

#### Selected-text RAG
```bash
curl -X POST http://localhost:8000/v1/chat/selected \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What does this text say about balance control?",
    "selected_text": "Humanoid robots require sophisticated balance control mechanisms to maintain stability during locomotion...",
    "session_id": null
  }'
```

#### Content Ingestion
```bash
curl -X POST http://localhost:8000/v1/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "content": "Chapter content here...",
    "chapter": "Balance Control",
    "section": "Introduction",
    "book_version": "1.0.0"
  }'
```

#### Health Check
```bash
curl http://localhost:8000/v1/health
```

## Key Architecture Components

### 1. BookAgent Implementation
- Uses OpenAI Assistant API with custom tools
- Implements `retrieve_context` and `finalize_answer` tools
- Handles delegation to SelectedTextAgent when needed

### 2. SelectedTextAgent Implementation
- Isolated agent for selected-text mode
- No access to vector store or book content
- Returns specific fallback when answer not found

### 3. Async Architecture
- Full async support with FastAPI
- Connection pooling with asyncpg for Neon Postgres
- Async operations throughout the stack

### 4. RAG Pipeline
- HyDE (Hypothetical Document Embeddings) for improved retrieval
- MMR (Maximum Marginal Relevance) for diverse chunk selection
- Top-8 chunk retrieval for accuracy

## Testing

### Unit Tests
```bash
# Run unit tests
uv run pytest tests/unit/ -v

# Run with coverage
uv run pytest tests/unit/ --cov=src --cov-report=html
```

### Integration Tests
```bash
# Run integration tests
uv run pytest tests/integration/ -v
```

## Production Deployment

### Docker Configuration
```dockerfile
FROM ghcr.io/astral-sh/uv:python3.11-bookworm-slim

WORKDIR /app

# Copy project files
COPY pyproject.toml uv.lock ./
COPY src ./src

# Install dependencies
RUN uv sync --no-dev

# Expose port
EXPOSE 8000

# Run application
CMD ["uv", "run", "uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

### Environment Variables for Production
```env
# Security
SECRET_KEY=your_secret_key_here
DEBUG=false

# Database
DATABASE_URL=your_production_database_url

# Rate limiting
RATE_LIMIT_PER_MINUTE=30

# Performance
WORKERS=4
TIMEOUT=300
```

## Troubleshooting

### Common Issues

1. **Connection Pool Exhaustion**: Increase pool size in database configuration
2. **Rate Limiting**: Check OpenAI and Qdrant usage quotas
3. **Embedding Latency**: Optimize chunk sizes and implement caching
4. **Memory Issues**: Monitor asyncpg pool and implement proper connection cleanup

### Performance Monitoring
- Monitor response times for API endpoints
- Track database connection pool metrics
- Watch OpenAI and Qdrant API usage
- Log slow queries and operations

## Next Steps

1. Implement the complete backend following the architecture documents
2. Set up proper CI/CD pipelines
3. Add comprehensive monitoring and alerting
4. Implement proper error handling and fallbacks
5. Add caching layers for improved performance