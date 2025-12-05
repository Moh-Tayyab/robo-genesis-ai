# Quickstart: RAG Chatbot Backend

## Prerequisites
- Python 3.11+
- Docker (optional, for containerization)
- OpenAI API key
- Qdrant Cloud account
- Neon Postgres account

## Setup

### 1. Clone Repository
```bash
git clone <repository-url>
cd Physical-AI-Humanoid-Robotics-Textbook
```

### 2. Create Virtual Environment
```bash
cd app
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

### 4. Environment Configuration
Copy `.env.example` to `.env` and fill in your credentials:
```bash
cp .env.example .env
# Edit .env with your API keys and connection strings
```

### 5. Install Dependencies File
Create `requirements.txt`:
```txt
fastapi==0.104.1
uvicorn==0.24.0
openai==1.3.8
qdrant-client==1.7.0
python-dotenv==1.0.0
pydantic==2.5.0
pydantic-settings==2.1.0
sqlalchemy==2.0.23
asyncpg==0.29.0
langchain==0.0.339
langchain-openai==0.0.5
sentence-transformers==2.2.2
pytest==7.4.3
httpx==0.25.2
python-multipart==0.0.6
python-jose[cryptography]==3.3.0
passlib[bcrypt]==1.7.4
python-rapidjson==1.12
```

## Running Locally

### 1. Start the Application
```bash
cd app
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### 2. API Documentation
Visit `http://localhost:8000/docs` for interactive API documentation.

## Ingesting Content

### 1. Prepare Markdown Files
Ensure your markdown files are in the correct format with proper metadata.

### 2. Run Ingestion
```bash
cd scripts
python ingest.py path/to/markdown/files
```

## Testing

### 1. Run Unit Tests
```bash
cd app
python -m pytest tests/unit/
```

### 2. Run Integration Tests
```bash
cd app
python -m pytest tests/integration/
```

### 3. Run Full Test Suite
```bash
cd app
python -m pytest
```

## API Endpoints

### Chat (Full Book)
```
POST /chat
Content-Type: application/json

{
  "question": "Your question here",
  "sessionId": "optional session ID"
}
```

### Chat (Selected Text)
```
POST /chat/selected
Content-Type: application/json

{
  "question": "Your question here",
  "selectedText": "The text you selected",
  "sessionId": "optional session ID"
}
```

### Health Check
```
GET /health
```

## Docker Deployment

### 1. Build Image
```bash
docker build -t rag-chatbot .
```

### 2. Run Container
```bash
docker run -p 8000:8000 --env-file .env rag-chatbot
```

## Configuration

The application uses environment variables for configuration:

- `OPENAI_API_KEY`: Your OpenAI API key
- `QDRANT_URL`: Qdrant Cloud URL
- `QDRANT_API_KEY`: Qdrant API key
- `NEON_DB_URL`: Neon Postgres connection string
- `QDRANT_TOP_K`: Number of chunks to retrieve (default: 5)
- `QDRANT_SCORE_THRESHOLD`: Minimum similarity score (default: 0.75)
- `RERANK_THRESHOLD`: Cross-encoder rerank threshold (default: 0.85)