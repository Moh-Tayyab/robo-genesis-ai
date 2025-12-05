# OpenAPI Specification: RAG Chatbot Backend

## /chat [POST]
### Description
Full-book RAG endpoint that answers questions based on the entire textbook content.

### Request
```json
{
  "question": "string",
  "sessionId": "string (optional)",
  "userId": "string (optional)"
}
```

### Response (200 OK)
```json
{
  "response": "string",
  "sources": [
    {
      "chapter": "string",
      "section": "string",
      "url": "string",
      "similarity": "number"
    }
  ],
  "sessionId": "string",
  "retrievalMetadata": {
    "chunksRetrieved": "number",
    "retrievalTimeMs": "number"
  }
}
```

## /chat/selected [POST]
### Description
Selected-text RAG endpoint that answers questions based only on the provided text selection.

### Request
```json
{
  "question": "string",
  "selectedText": "string (max 4000 chars)",
  "sessionId": "string (optional)",
  "userId": "string (optional)"
}
```

### Response (200 OK)
```json
{
  "response": "string",
  "sources": [
    {
      "chapter": "string",
      "section": "string",
      "url": "string",
      "similarity": "number"
    }
  ],
  "sessionId": "string",
  "retrievalMetadata": {
    "chunksRetrieved": "number",
    "retrievalTimeMs": "number"
  }
}
```

## /ingest [POST]
### Description
Idempotent ingestion endpoint that processes markdown files into vector embeddings.

### Request
```json
{
  "filePath": "string",
  "metadata": {
    "chapter": "string",
    "section": "string",
    "url": "string",
    "difficulty": "string (enum: beginner, intermediate, advanced)",
    "lang": "string (default: en)"
  }
}
```

### Response (200 OK)
```json
{
  "status": "completed",
  "chunksProcessed": "number",
  "processingTimeMs": "number"
}
```

## /health [GET]
### Description
Health check endpoint to verify service availability and external dependencies.

### Response (200 OK)
```json
{
  "status": "healthy",
  "timestamp": "ISO 8601 datetime",
  "dependencies": {
    "openai": "boolean",
    "qdrant": "boolean",
    "neon": "boolean"
  }
}
```

## Error Responses
All endpoints may return:
- 400 Bad Request: Invalid input parameters
- 401 Unauthorized: Missing or invalid authentication
- 429 Too Many Requests: Rate limit exceeded
- 500 Internal Server Error: Unexpected server error

### Error Format
```json
{
  "error": {
    "code": "string",
    "message": "string",
    "details": "object (optional)"
  }
}
```