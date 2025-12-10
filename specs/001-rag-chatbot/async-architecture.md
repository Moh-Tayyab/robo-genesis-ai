
# Async Architecture with Connection Pooling: RAG Chatbot Backend

**Feature**: 001-rag-chatbot
**Date**: 2025-12-06

## Overview

This document outlines the asynchronous architecture design for the RAG Chatbot Backend with proper connection pooling for Neon Postgres. The architecture ensures full async support, efficient resource utilization, and scalability to handle concurrent users while maintaining ≤ 1 second response time.

## Architecture Components

### 1. Application Layer (FastAPI)
- **Framework**: FastAPI with async support
- **Concurrency**: Built-in async/await support
- **Performance**: Pydantic models for fast validation
- **Documentation**: Automatic OpenAPI/Swagger generation

### 2. Database Layer (Neon Postgres with asyncpg)
- **Driver**: asyncpg for async PostgreSQL operations
- **Pooling**: Connection pooling for efficient resource management
- **Transactions**: Async transaction support
- **Querying**: Async query execution

### 3. Vector Store Layer (Qdrant)
- **Client**: Qdrant async client
- **Operations**: Async search and storage operations
- **Batching**: Async batch operations
- **Connections**: Async HTTP/WebSocket connections

### 4. AI Service Layer (OpenAI)
- **Client**: OpenAI async client
- **Operations**: Async API calls
- **Streaming**: Async streaming responses
- **Rate Limits**: Async rate limiting

## Async Architecture Design

### 1. FastAPI Application Structure

```python
from contextlib import asynccontextmanager
from fastapi import FastAPI, Depends, HTTPException
import asyncpg
from typing import AsyncGenerator
import logging

# Global database pool
db_pool = None

@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    global db_pool

    # Startup
    logging.info("Initializing database connection pool...")
    db_pool = await asyncpg.create_pool(
        dsn=settings.DATABASE_URL,
        min_size=5,
        max_size=20,
        max_queries=50000,  # Close connection after this many queries
        max_inactive_connection_lifetime=300,  # 5 minutes
        command_timeout=60,
        server_settings={
            "application_name": "rag-chatbot",
            "statement_timeout": "30s"
        }
    )

    try:
        yield
    finally:
        # Shutdown
        if db_pool:
            await db_pool.close()
            logging.info("Database connection pool closed")

app = FastAPI(
    title="RAG Chatbot API",
    version="1.0.0",
    lifespan=lifespan
)

# Dependency to get database connection
async def get_db_pool():
    return db_pool
```

### 2. Database Connection Pooling

#### Pool Configuration:
```python
from asyncpg import Pool
import os

class DatabasePoolManager:
    def __init__(self):
        self.pool: Pool = None
        self.dsn = os.getenv("NEON_DATABASE_URL")

    async def init_pool(self):
        self.pool = await asyncpg.create_pool(
            dsn=self.dsn,
            min_size=5,                    # Minimum connections in pool
            max_size=20,                   # Maximum connections in pool
            max_queries=50000,             # Max queries per connection
            max_inactive_connection_lifetime=300,  # 5 minutes idle timeout
            command_timeout=60,            # 60 second command timeout
            init=DatabasePoolManager._init_connection,  # Initialize new connections
            server_settings={
                "application_name": "rag-chatbot",
                "statement_timeout": "30s",
                "idle_in_transaction_session_timeout": "60s"
            }
        )
        return self.pool

    @staticmethod
    async def _init_connection(conn):
        """Initialize a new connection with session settings"""
        await conn.set_type_codec(
            'json',
            encoder=json.dumps,
            decoder=json.loads,
            schema='pg_catalog'
        )

    @asynccontextmanager
    async def acquire(self):
        """Context manager to acquire a connection from the pool"""
        async with self.pool.acquire() as conn:
            yield conn

    async def close(self):
        """Close the connection pool"""
        if self.pool:
            await self.pool.close()
```

#### Repository Pattern with Async:
```python
from typing import List, Optional, Dict, Any
from datetime import datetime
import asyncpg
from contextlib import asynccontextmanager

class UserRepository:
    def __init__(self, pool: asyncpg.Pool):
        self.pool = pool

    async def create_user(self) -> str:
        """Create a new anonymous user"""
        async with self.pool.acquire() as conn:
            result = await conn.fetchrow(
                """
                INSERT INTO users (created_at, updated_at)
                VALUES (NOW(), NOW())
                RETURNING id
                """,
            )
            return str(result['id'])

    async def get_user(self, user_id: str) -> Optional[Dict[str, Any]]:
        """Get user by ID"""
        async with self.pool.acquire() as conn:
            result = await conn.fetchrow(
                "SELECT * FROM users WHERE id = $1",
                user_id
            )
            return dict(result) if result else None

class SessionRepository:
    def __init__(self, pool: asyncpg.Pool):
        self.pool = pool

    async def create_session(self, user_id: str, title: Optional[str] = None) -> str:
        """Create a new chat session"""
        async with self.pool.acquire() as conn:
            result = await conn.fetchrow(
                """
                INSERT INTO sessions (user_id, title, created_at, updated_at)
                VALUES ($1, $2, NOW(), NOW())
                RETURNING id
                """,
                user_id, title
            )
            return str(result['id'])

    async def get_session(self, session_id: str) -> Optional[Dict[str, Any]]:
        """Get session by ID"""
        async with self.pool.acquire() as conn:
            result = await conn.fetchrow(
                "SELECT * FROM sessions WHERE id = $1",
                session_id
            )
            return dict(result) if result else None

    async def update_session_title(self, session_id: str, title: str):
        """Update session title"""
        async with self.pool.acquire() as conn:
            await conn.execute(
                """
                UPDATE sessions
                SET title = $1, updated_at = NOW()
                WHERE id = $2
                """,
                title, session_id
            )
```

### 3. Async Service Layer

```python
from typing import List, Dict, Any, Optional
from datetime import datetime
import uuid
from openai import AsyncOpenAI
from qdrant_client import AsyncQdrantClient
from contextlib import asynccontextmanager

class ChatService:
    def __init__(self,
                 db_pool: asyncpg.Pool,
                 qdrant_client: AsyncQdrantClient,
                 openai_client: AsyncOpenAI):
        self.db_pool = db_pool
        self.qdrant_client = qdrant_client
        self.openai_client = openai_client
        self.user_repo = UserRepository(db_pool)
        self.session_repo = SessionRepository(db_pool)
        self.message_repo = MessageRepository(db_pool)
        self.retrieval_repo = RetrievalRepository(db_pool)

    async def process_chat_request(self,
                                 question: str,
                                 session_id: Optional[str] = None,
                                 use_selected: bool = False,
                                 selected_text: Optional[str] = None) -> Dict[str, Any]:
        """Process a chat request asynchronously"""

        # Create or get session
        if not session_id:
            user_id = await self.user_repo.create_user()
            session_id = await self.session_repo.create_session(user_id)
        else:
            session = await self.session_repo.get_session(session_id)
            if not session:
                raise HTTPException(status_code=404, detail="Session not found")
            user_id = session['user_id']

        # Add user message to database
        user_msg_id = await self.message_repo.create_message(
            session_id, "user", question
        )

        # Process based on mode
        if use_selected:
            response = await self._process_selected_text_mode(
                selected_text, question
            )
        else:
            response = await self._process_full_book_mode(question)

        # Add assistant message to database
        assistant_msg_id = await self.message_repo.create_message(
            session_id, "assistant", response['answer']
        )

        # If in full-book mode, store retrievals
        if not use_selected and 'sources' in response:
            for source in response.get('sources', []):
                await self.retrieval_repo.create_retrieval(
                    message_id=assistant_msg_id,
                    chunk_id=source['chunk_id'],
                    score=source.get('score'),
                    metadata=source
                )

        return {
            "answer": response['answer'],
            "sources": response.get('sources'),
            "session_id": session_id
        }

    async def _process_full_book_mode(self, question: str) -> Dict[str, Any]:
        """Process question using full book RAG"""
        # 1. Retrieve context using Qdrant
        retrieved_chunks = await self._retrieve_context(question)

        # 2. Generate answer using OpenAI
        answer = await self._generate_answer(question, retrieved_chunks)

        # 3. Format sources
        sources = self._format_sources(retrieved_chunks)

        return {
            "answer": answer,
            "sources": sources
        }

    async def _process_selected_text_mode(self,
                                        selected_text: str,
                                        question: str) -> Dict[str, Any]:
        """Process question using selected text only"""
        if not selected_text or len(selected_text) > 4000:
            raise ValueError("Selected text must be between 1-4000 characters")

        # Use isolated processing with only selected text
        context = f"Context: {selected_text}\n\nQuestion: {question}"

        response = await self.openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "system",
                    "content": "Answer ONLY based on the provided context. If the answer is not in the context, respond with: 'I can only answer based on the selected text, and the answer is not present there.'"
                },
                {"role": "user", "content": context}
            ],
            max_tokens=500,
            temperature=0.1
        )

        return {
            "answer": response.choices[0].message.content
        }

    async def _retrieve_context(self, query: str) -> List[Dict[str, Any]]:
        """Retrieve relevant context from Qdrant asynchronously"""
        # Generate hypothetical document for HyDE
        hypothetical_doc = await self._generate_hypothetical_document(query)
        hypothetical_embedding = await self._embed_text(hypothetical_doc)

        # Search in Qdrant
        search_results = await self.qdrant_client.search(
            collection_name="book_chunks",
            query_vector=hypothetical_embedding,
            limit=16,  # Get more for MMR
            with_payload=True,
            with_vectors=False
        )

        # Apply MMR to select top-8 diverse chunks
        selected_chunks = await self._apply_mmr_selection(
            candidates=search_results,
            query=query,
            top_k=8
        )

        return selected_chunks

    async def _generate_answer(self, question: str, context: List[Dict[str, Any]]) -> str:
        """Generate answer using OpenAI asynchronously"""
        context_text = "\n\n".join([chunk['content'] for chunk in context])

        response = await self.openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "system",
                    "content": "You are an AI assistant for a Physical AI & Humanoid Robotics textbook. Provide accurate answers based on the provided context."
                },
                {
                    "role": "user",
                    "content": f"Context: {context_text}\n\nQuestion: {question}"
                }
            ],
            max_tokens=500
        )

        return response.choices[0].message.content
```

### 4. Async API Endpoints

```python
from fastapi import APIRouter, Depends, HTTPException, BackgroundTasks
from pydantic import UUID4
import uuid
from typing import Optional

router = APIRouter()

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    db_pool: asyncpg.Pool = Depends(get_db_pool)
):
    """Full-book RAG chat endpoint"""
    try:
        service = ChatService(
            db_pool=db_pool,
            qdrant_client=qdrant_client,
            openai_client=openai_client
        )

        result = await service.process_chat_request(
            question=request.question,
            session_id=str(request.session_id) if request.session_id else None,
            use_selected=request.use_selected,
            selected_text=request.selected_text
        )

        return ChatResponse(**result)
    except Exception as e:
        logging.error(f"Chat endpoint error: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")

@router.post("/chat/selected", response_model=ChatResponse)
async def chat_selected_endpoint(
    request: ChatSelectedRequest,
    db_pool: asyncpg.Pool = Depends(get_db_pool)
):
    """Selected-text RAG chat endpoint"""
    if not request.selected_text or len(request.selected_text) > 4000:
        raise HTTPException(status_code=400, detail="Selected text must be 1-4000 characters")

    try:
        service = ChatService(
            db_pool=db_pool,
            qdrant_client=qdrant_client,
            openai_client=openai_client
        )

        result = await service.process_chat_request(
            question=request.question,
            session_id=str(request.session_id) if request.session_id else None,
            use_selected=True,
            selected_text=request.selected_text
        )

        return ChatResponse(**result)
    except Exception as e:
        logging.error(f"Selected chat endpoint error: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")

@router.post("/ingest")
async def ingest_endpoint(
    request: IngestRequest,
    background_tasks: BackgroundTasks,
    db_pool: asyncpg.Pool = Depends(get_db_pool)
):
    """Content ingestion endpoint"""
    try:
        ingestion_service = IngestionService(
            db_pool=db_pool,
            qdrant_client=qdrant_client,
            embedding_generator=embedding_generator
        )

        # Process ingestion in background to handle large content
        result = await ingestion_service.ingest_content(
            content=request.content,
            metadata={
                "chapter": request.chapter,
                "section": request.section,
                "book_version": request.book_version,
                "page_number": request.page_number,
                "source_file": request.source_file
            }
        )

        return result
    except Exception as e:
        logging.error(f"Ingestion endpoint error: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")

@router.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint"""
    try:
        # Check database connectivity
        async with db_pool.acquire() as conn:
            await conn.fetchval("SELECT 1")

        # Check Qdrant connectivity
        await qdrant_client.get_collection("book_chunks")

        # Check OpenAI connectivity
        await openai_client.models.list()

        return HealthResponse(
            status="healthy",
            timestamp=datetime.utcnow()
        )
    except Exception as e:
        logging.error(f"Health check failed: {str(e)}")
        raise HTTPException(status_code=503, detail="Service unhealthy")
```

## Connection Pooling Strategy

### 1. Pool Sizing
- **Min Size**: 5 connections (handle baseline load)
- **Max Size**: 20 connections (handle peak load)
- **Max Queries**: 50,000 per connection (prevent memory leaks)
- **Idle Timeout**: 300 seconds (balance between resource usage and connection reuse)

### 2. Connection Lifecycle
```
New Request → Acquire Connection from Pool → Process Request → Release Connection to Pool
     ↑                                         ↓
     ← Connection is reused for next request ←−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
```

### 3. Pool Monitoring
```python
import time
from prometheus_client import Counter, Histogram, Gauge

# Metrics for monitoring
connection_acquire_time = Histogram('db_connection_acquire_seconds', 'Time to acquire database connection')
active_connections = Gauge('db_active_connections', 'Number of active database connections')
idle_connections = Gauge('db_idle_connections', 'Number of idle database connections')
pool_requests_total = Counter('db_pool_requests_total', 'Total number of pool requests')
pool_requests_failed = Counter('db_pool_requests_failed', 'Number of failed pool requests')

class MonitoredPool:
    def __init__(self, pool):
        self.pool = pool

    async def acquire(self):
        start_time = time.time()
        try:
            conn = await self.pool.acquire()
            connection_acquire_time.observe(time.time() - start_time)
            pool_requests_total.inc()
            return conn
        except Exception as e:
            pool_requests_failed.inc()
            raise
        finally:
            active_connections.set(self.pool.get_size())
            idle_connections.set(self.pool.get_idle_size())
```

## Performance Optimizations

### 1. Async Context Managers
```python
from contextlib import asynccontextmanager

@asynccontextmanager
async def get_db_transaction(pool: asyncpg.Pool):
    """Context manager for database transactions"""
    async with pool.acquire() as conn:
        async with conn.transaction():
            yield conn
```

### 2. Connection Reuse
- Use connection pooling to avoid connection overhead
- Reuse connections for multiple requests
- Properly release connections back to pool

### 3. Batch Operations
```python
async def batch_store_messages(messages: List[Message]):
    """Efficiently store multiple messages"""
    async with db_pool.acquire() as conn:
        # Use COPY for bulk insertions when possible
        await conn.copy_records_to_table(
            'messages',
            records=[(msg.session_id, msg.role, msg.content, msg.created_at) for msg in messages],
            columns=['session_id', 'role', 'content', 'created_at']
        )
```

## Error Handling and Resilience

### 1. Connection Recovery
```python
from tenacity import retry, stop_after_attempt, wait_exponential

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=4, max=10)
)
async def robust_db_operation(operation, *args, **kwargs):
    """Execute database operation with retry logic"""
    try:
        return await operation(*args, **kwargs)
    except asyncpg.PostgresError as e:
        logging.error(f"Database error: {str(e)}")
        raise
    except Exception as e:
        logging.error(f"Unexpected error: {str(e)}")
        raise
```

### 2. Circuit Breaker Pattern
```python
import asyncio
from enum import Enum
from datetime import datetime, timedelta

class CircuitState(Enum):
    CLOSED = "closed"
    OPEN = "open"
    HALF_OPEN = "half_open"

class CircuitBreaker:
    def __init__(self, failure_threshold: int = 5, timeout: int = 60):
        self.failure_threshold = failure_threshold
        self.timeout = timeout
        self.failure_count = 0
        self.last_failure_time = None
        self.state = CircuitState.CLOSED

    async def call(self, func, *args, **kwargs):
        if self.state == CircuitState.OPEN:
            if datetime.now() - self.last_failure_time > timedelta(seconds=self.timeout):
                self.state = CircuitState.HALF_OPEN
            else:
                raise Exception("Circuit breaker is OPEN")

        try:
            result = await func(*args, **kwargs)
            self.on_success()
            return result
        except Exception as e:
            self.on_failure()
            raise e

    def on_success(self):
        self.failure_count = 0
        self.state = CircuitState.CLOSED

    def on_failure(self):
        self.failure_count += 1
        self.last_failure_time = datetime.now()
        if self.failure_count >= self.failure_threshold:
            self.state = CircuitState.OPEN
```

## Monitoring and Observability

### 1. Key Metrics
- **Connection Pool Metrics**: Active, idle, and waiting connections
- **Query Performance**: Query execution time, slow queries
- **Error Rates**: Database connection errors, query errors
- **Response Times**: API endpoint response times

### 2. Logging Strategy
- Log slow queries (>100ms)
- Log connection pool events
- Log error details for debugging
- Log performance metrics

### 3. Health Checks
- Database connectivity
- Connection pool status
- Query performance
- Overall system health

## Security Considerations

### 1. Connection Security
- Use SSL/TLS for database connections
- Validate SSL certificates
- Use connection string parameters securely

### 2. Credential Management
- Store database credentials securely
- Use environment variables or secure vault
- Rotate credentials regularly

### 3. Injection Prevention
- Use parameterized queries
- Validate all inputs
- Sanitize user inputs