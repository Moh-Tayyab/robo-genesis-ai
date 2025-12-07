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