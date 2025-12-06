# Data Model: RAG Chatbot Backend

**Feature**: 001-rag-chatbot
**Date**: 2025-12-06

## Overview

This document defines the data models for the RAG Chatbot Backend, including database schemas, Pydantic models, and vector store structures. The models support both full-book Q&A and selected-text Q&A modes with proper metadata handling and context isolation.

## Database Schema

### 1. User Table
**Purpose**: Store anonymous user information for tracking purposes

```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);
```

**Model**: `User`

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique identifier for anonymous user |
| created_at | TIMESTAMP WITH TIME ZONE | DEFAULT NOW() | Record creation timestamp |
| updated_at | TIMESTAMP WITH TIME ZONE | DEFAULT NOW() | Record update timestamp |

### 2. Session Table
**Purpose**: Store chat session metadata

```sql
CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    title VARCHAR(255),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);
```

**Model**: `Session`

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique session identifier |
| user_id | UUID | NOT NULL, FOREIGN KEY | Reference to user |
| title | VARCHAR(255) | | Session title (auto-generated from first question) |
| created_at | TIMESTAMP WITH TIME ZONE | DEFAULT NOW() | Session creation timestamp |
| updated_at | TIMESTAMP WITH TIME ZONE | DEFAULT NOW() | Session update timestamp |

### 3. Message Table
**Purpose**: Store individual messages in chat conversations

```sql
CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);
```

**Model**: `Message`

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique message identifier |
| session_id | UUID | NOT NULL, FOREIGN KEY | Reference to session |
| role | VARCHAR(20) | NOT NULL, CHECK role IN ('user', 'assistant') | Message role (user or assistant) |
| content | TEXT | NOT NULL | Message content |
| created_at | TIMESTAMP WITH TIME ZONE | DEFAULT NOW() | Message creation timestamp |

### 4. Retrieval Table
**Purpose**: Store information about which content chunks were retrieved for each message

```sql
CREATE TABLE retrievals (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES messages(id) ON DELETE CASCADE,
    chunk_id VARCHAR(255) NOT NULL,
    score DECIMAL(5, 4),
    metadata JSONB,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);
```

**Model**: `Retrieval`

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique retrieval identifier |
| message_id | UUID | NOT NULL, FOREIGN KEY | Reference to message |
| chunk_id | VARCHAR(255) | NOT NULL | ID of the retrieved chunk in vector store |
| score | DECIMAL(5, 4) | | Relevance score of the retrieval |
| metadata | JSONB | | Additional metadata (chapter, section, book_version) |
| created_at | TIMESTAMP WITH TIME ZONE | DEFAULT NOW() | Retrieval creation timestamp |

## Pydantic Models

### 1. User Models

```python
from pydantic import BaseModel, UUID4
from datetime import datetime
from typing import Optional
import uuid

class UserBase(BaseModel):
    pass

class UserCreate(UserBase):
    pass

class User(UserBase):
    id: UUID4
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

### 2. Session Models

```python
class SessionBase(BaseModel):
    title: Optional[str] = None

class SessionCreate(SessionBase):
    user_id: UUID4

class Session(SessionBase):
    id: UUID4
    user_id: UUID4
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
```

### 3. Message Models

```python
from enum import Enum

class MessageRole(str, Enum):
    user = "user"
    assistant = "assistant"

class MessageBase(BaseModel):
    session_id: UUID4
    role: MessageRole
    content: str

class MessageCreate(MessageBase):
    pass

class Message(MessageBase):
    id: UUID4
    created_at: datetime

    class Config:
        from_attributes = True
```

### 4. Retrieval Models

```python
from typing import Dict, Any

class RetrievalBase(BaseModel):
    message_id: UUID4
    chunk_id: str
    score: Optional[float] = None
    metadata: Optional[Dict[str, Any]] = None

class RetrievalCreate(RetrievalBase):
    pass

class Retrieval(RetrievalBase):
    id: UUID4
    created_at: datetime

    class Config:
        from_attributes = True
```

### 5. Chat Request/Response Models

```python
class ChatRequest(BaseModel):
    question: str
    session_id: Optional[UUID4] = None
    use_selected: bool = False
    selected_text: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str
    sources: Optional[list] = None
    session_id: UUID4

class HealthResponse(BaseModel):
    status: str
    timestamp: datetime
```

## Vector Store Schema (Qdrant)

### 1. Collection Structure
**Collection Name**: `book_chunks`

**Vector Configuration**:
- Dense vector: `content_vector` (dimension: 1536 - for text-embedding-3-small)
- Sparse vector: `sparse_vector` (for hybrid search)

**Payload Structure**:
```json
{
  "content": "string",
  "chapter": "string",
  "section": "string",
  "book_version": "string",
  "page_number": "integer",
  "source_file": "string",
  "chunk_index": "integer",
  "created_at": "timestamp"
}
```

**Payload Fields**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| content | string | Yes | The actual text content of the chunk |
| chapter | string | Yes | Chapter identifier or name |
| section | string | Yes | Section identifier or name |
| book_version | string | Yes | Version of the book this chunk belongs to |
| page_number | integer | No | Page number in the source document |
| source_file | string | No | Original source file name |
| chunk_index | integer | No | Sequential index of this chunk in the document |
| created_at | timestamp | Yes | Timestamp when chunk was created |

### 2. Collection Configuration
```python
from qdrant_client.http import models

collection_config = models.VectorParams(
    size=1536,  # For text-embedding-3-small
    distance=models.Distance.COSINE,
    hnsw_config=models.HnswConfigDiff(
        m=16,
        ef_construct=100,
        full_scan_threshold=10000
    ),
    quantization_config=models.ScalarQuantization(
        type=models.QuantizationType.INT8,
        quantile=0.99,
        always_ram=True
    )
)
```

## Relationships

### Database Relationships
```
User (1) → (N) Session
Session (1) → (N) Message
Message (1) → (N) Retrieval
```

### Vector Store Relationships
- Document chunks stored in Qdrant collection with metadata
- Retrieval table references chunk IDs in vector store
- Messages link to retrievals that informed the response

## Indexes

### Database Indexes
```sql
-- User indexes
CREATE INDEX idx_users_created_at ON users(created_at);

-- Session indexes
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_created_at ON sessions(created_at);

-- Message indexes
CREATE INDEX idx_messages_session_id ON messages(session_id);
CREATE INDEX idx_messages_created_at ON messages(created_at);

-- Retrieval indexes
CREATE INDEX idx_retrievals_message_id ON retrievals(message_id);
CREATE INDEX idx_retrievals_chunk_id ON retrievals(chunk_id);
```

### Vector Store Indexes
- Qdrant automatically creates HNSW index for dense vectors
- Sparse vector index for hybrid search
- Payload indexes for metadata filtering (chapter, section, book_version)

## Validation Rules

### 1. User Validation
- User ID must be a valid UUID
- Created/updated timestamps automatically managed

### 2. Session Validation
- Session must belong to a valid user
- Title optional, auto-generated if not provided

### 3. Message Validation
- Role must be either 'user' or 'assistant'
- Content must not be empty
- Message must belong to a valid session

### 4. Retrieval Validation
- Retrieval must belong to a valid message
- Chunk ID must exist in vector store
- Score between 0.0 and 1.0 if provided

### 5. Vector Store Validation
- Content chunks must have required metadata (chapter, section, book_version)
- Content length should be optimized (not too short or too long)
- Unique constraints on chunk identifiers

## Context Isolation Requirements

### Selected Text Mode Validation
- When `use_selected` is True, the system must NOT access vector store
- Only the provided `selected_text` should be used as context
- If question cannot be answered from selection, return specific message
- No cross-contamination between selected text and full book content