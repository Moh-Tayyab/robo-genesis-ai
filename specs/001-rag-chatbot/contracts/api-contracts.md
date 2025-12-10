# API Contracts: RAG Chatbot Backend

**Feature**: 001-rag-chatbot
**Date**: 2025-12-06

## Overview

This document defines the API contracts for the RAG Chatbot Backend. The API supports both full-book Q&A and selected-text Q&A modes with proper rate limiting and health monitoring.

## API Specification

### Base URL
```
https://api.rag-chatbot.example.com/v1
```

### Common Headers
- `Content-Type: application/json`
- `Authorization: Bearer {API_KEY}` (for authenticated endpoints)

### Rate Limiting
- 30 requests per minute per IP address
- 10 requests per minute per session

## Endpoints

### 1. Full-book RAG Chat Endpoint

#### POST `/chat`

**Description**: Process a question against the entire book content using RAG.

**Request**:
```json
{
  "question": "string",
  "session_id": "UUID (optional)",
  "use_selected": false,
  "selected_text": null
}
```

**Response**:
- **200 OK**: Successfully processed question
```json
{
  "answer": "string",
  "sources": [
    {
      "chapter": "string",
      "section": "string",
      "page": "integer",
      "score": "float"
    }
  ],
  "session_id": "UUID"
}
```

- **400 Bad Request**: Invalid request parameters
- **429 Too Many Requests**: Rate limit exceeded
- **500 Internal Server Error**: Processing error

**Example Request**:
```bash
curl -X POST https://api.rag-chatbot.example.com/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is the main principle of humanoid robotics?",
    "session_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef"
  }'
```

**Example Response**:
```json
{
  "answer": "The main principle of humanoid robotics is to create robots that mimic human form and behavior...",
  "sources": [
    {
      "chapter": "Introduction to Humanoid Robotics",
      "section": "Core Principles",
      "page": 15,
      "score": 0.87
    }
  ],
  "session_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef"
}
```

### 2. Selected-text RAG Chat Endpoint

#### POST `/chat/selected`

**Description**: Process a question exclusively against the provided selected text.

**Request**:
```json
{
  "question": "string",
  "selected_text": "string",
  "session_id": "UUID (optional)",
  "use_selected": true
}
```

**Response**:
- **200 OK**: Successfully processed question
```json
{
  "answer": "string",
  "sources": null,
  "session_id": "UUID"
}
```

- **400 Bad Request**: Invalid request parameters or selected_text too long
- **429 Too Many Requests**: Rate limit exceeded
- **500 Internal Server Error**: Processing error

**Example Request**:
```bash
curl -X POST https://api.rag-chatbot.example.com/v1/chat/selected \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What does this text say about bipedal locomotion?",
    "selected_text": "Bipedal locomotion in humanoid robots requires precise balance control and coordination of multiple joints...",
    "session_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef"
  }'
```

**Example Response**:
```json
{
  "answer": "The text explains that bipedal locomotion in humanoid robots requires precise balance control and coordination of multiple joints...",
  "sources": null,
  "session_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef"
}
```

**Fallback Response Example**:
```json
{
  "answer": "I can only answer based on the selected text, and the answer is not present there.",
  "sources": null,
  "session_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef"
}
```

### 3. Content Ingestion Endpoint

#### POST `/ingest`

**Description**: Ingest book content into the vector store with metadata.

**Request**:
```json
{
  "content": "string",
  "chapter": "string",
  "section": "string",
  "book_version": "string",
  "page_number": "integer (optional)",
  "source_file": "string (optional)"
}
```

**Response**:
- **200 OK**: Successfully ingested content
```json
{
  "status": "success",
  "chunks_created": "integer",
  "chunk_ids": ["string"]
}
```

- **400 Bad Request**: Invalid content or missing required metadata
- **409 Conflict**: Content already exists (idempotent operation)
- **500 Internal Server Error**: Ingestion error

**Example Request**:
```bash
curl -X POST https://api.rag-chatbot.example.com/v1/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "content": "Chapter 1: Introduction to Physical AI and Humanoid Robotics...",
    "chapter": "Introduction",
    "section": "Overview",
    "book_version": "1.0.0",
    "page_number": 1
  }'
```

### 4. Health Check Endpoint

#### GET `/health`

**Description**: Check the health status of the service.

**Response**:
- **200 OK**: Service is healthy
```json
{
  "status": "healthy",
  "timestamp": "ISO 8601 datetime string",
  "version": "string",
  "dependencies": {
    "postgres": "healthy",
    "qdrant": "healthy",
    "openai": "healthy"
  }
}
```

- **503 Service Unavailable**: Service is unhealthy

**Example Request**:
```bash
curl https://api.rag-chatbot.example.com/v1/health
```

**Example Response**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-06T10:30:00Z",
  "version": "1.0.0",
  "dependencies": {
    "postgres": "healthy",
    "qdrant": "healthy",
    "openai": "healthy"
  }
}
```

## Error Responses

All error responses follow the same structure:

```json
{
  "error": {
    "code": "string",
    "message": "string",
    "details": "object (optional)"
  }
}
```

### Common Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| `INVALID_REQUEST` | 400 | Request parameters are invalid |
| `RATE_LIMIT_EXCEEDED` | 429 | Rate limit has been exceeded |
| `VECTOR_STORE_ERROR` | 500 | Error accessing vector store |
| `DATABASE_ERROR` | 500 | Error accessing database |
| `OPENAI_ERROR` | 500 | Error with OpenAI API |
| `SELECTION_TOO_LONG` | 400 | Selected text exceeds 4000 character limit |
| `NO_ANSWER_FOUND` | 200 | For selected text mode when answer not in selection |

## Rate Limiting Headers

When rate limits are approached, the following headers are included:

- `X-RateLimit-Limit`: The maximum number of requests allowed
- `X-RateLimit-Remaining`: The remaining number of requests
- `X-RateLimit-Reset`: Unix timestamp for when the rate limit resets

## Security Requirements

1. All API requests must use HTTPS
2. Authentication via API key in Authorization header
3. Rate limiting per IP and per session
4. Input validation and sanitization
5. No sensitive information in error messages

## Performance Requirements

1. Response time ≤ 1 second for 95% of requests
2. Support for concurrent users
3. Proper timeout handling for external services (OpenAI, Qdrant, Neon)
4. Connection pooling for database operations

## Validation Rules

1. All UUIDs must be valid format
2. Text content must be properly encoded
3. Selected text length ≤ 4000 characters
4. Required fields must be present
5. Data types must match schema