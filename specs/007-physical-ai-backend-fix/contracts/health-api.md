# API Contract: Health Check Service

## Overview
Health check endpoint to verify backend service status, including embedding model availability after the ONNX download fixes.

## Endpoint
`GET /health`

## Request
### Path
`/health`

### Headers
```
Accept: application/json
```

### Parameters
None

## Response
### Success Response (200 OK)
```json
{
  "status": "healthy",
  "timestamp": "2025-12-07T12:04:00Z",
  "services": {
    "embedding_model": {
      "status": "available",
      "model_id": "Qdrant/bge-small-en-v1.5-onnx-q",
      "is_fallback": false,
      "download_attempts": 1
    },
    "qdrant": {
      "status": "connected"
    },
    "gemini_agent": {
      "status": "ready"
    }
  },
  "details": {
    "version": "1.0.0",
    "model_cache_path": "/path/to/model/cache",
    "last_model_download": "2025-12-07T12:03:00Z"
  }
}
```

### Service Unavailable Response (503)
```json
{
  "status": "unhealthy",
  "timestamp": "2025-12-07T12:04:00Z",
  "services": {
    "embedding_model": {
      "status": "unavailable",
      "error": "Model download failed after 3 attempts",
      "model_id": "Qdrant/bge-small-en-v1.5-onnx-q",
      "is_fallback": false,
      "download_attempts": 3
    },
    "qdrant": {
      "status": "disconnected",
      "error": "Connection refused"
    }
  },
  "details": {
    "version": "1.0.0",
    "model_cache_path": "/path/to/model/cache"
  }
}
```

## Error Responses
- `503 Service Unavailable`: Backend services are not ready, typically due to failed model downloads or other startup issues

## Purpose
This endpoint allows monitoring systems and users to verify that the backend service has started successfully and that the embedding model has been downloaded and is available. After the ONNX download fixes, this endpoint should show "healthy" status more consistently, with proper tracking of download attempts and fallback model usage if needed.