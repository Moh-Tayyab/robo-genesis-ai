# Vector Ingestion Pipeline: RAG Chatbot Backend

**Feature**: 001-rag-chatbot
**Date**: 2025-12-06

## Overview

This document outlines the design for the vector ingestion pipeline that will process book content and store it in Qdrant with proper metadata (chapter, section, book_version). The pipeline ensures idempotent operations and proper chunking for optimal RAG performance.

## Requirements

### Functional Requirements
1. **Idempotent Ingestion**: Multiple ingestions of the same content should not create duplicates
2. **Proper Chunking**: Split content into appropriately sized chunks for optimal retrieval
3. **Metadata Preservation**: Store chapter, section, and book_version metadata with each chunk
4. **Content Validation**: Validate content before ingestion
5. **Error Handling**: Handle ingestion errors gracefully with retry mechanisms

### Non-Functional Requirements
1. **Performance**: Efficient processing of large documents
2. **Scalability**: Handle multiple concurrent ingestion requests
3. **Reliability**: Ensure no data loss during ingestion
4. **Monitoring**: Track ingestion metrics and errors

## Pipeline Architecture

### 1. Ingestion Flow

```
Input Content → Validation → Chunking → Embedding → Qdrant Storage → Metadata Storage
```

### 2. Component Breakdown

#### A. Input Validation Layer
- Validate content format and structure
- Check for required metadata (chapter, section, book_version)
- Verify content is not empty or malformed

#### B. Content Chunking Layer
- Split content into optimal-sized chunks
- Preserve semantic boundaries
- Handle overlapping chunks if needed for context

#### C. Embedding Generation Layer
- Generate embeddings using text-embedding-3-small model
- Process in batches for efficiency
- Handle embedding errors and retries

#### D. Vector Storage Layer
- Store vectors in Qdrant collection
- Include metadata with each vector
- Handle duplicate detection and idempotency

#### E. Metadata Storage Layer
- Store ingestion metadata in Neon Postgres
- Track ingestion status and history
- Enable content lineage tracking

## Detailed Design

### 1. Content Validation

```python
def validate_content(content: str, metadata: dict) -> bool:
    # Check content is not empty
    if not content or len(content.strip()) == 0:
        raise ValueError("Content cannot be empty")

    # Check required metadata
    required_fields = ['chapter', 'section', 'book_version']
    for field in required_fields:
        if field not in metadata or not metadata[field]:
            raise ValueError(f"Missing required metadata field: {field}")

    # Check content length limits
    if len(content) > MAX_CONTENT_SIZE:
        raise ValueError(f"Content exceeds maximum size of {MAX_CONTENT_SIZE} characters")

    return True
```

### 2. Smart Chunking Strategy

#### Chunking Parameters:
- **Target Chunk Size**: 1000-2000 tokens (approximately 750-1500 words)
- **Overlap**: 200 tokens to maintain context across chunks
- **Semantic Boundaries**: Prefer splitting at paragraph, sentence, or section boundaries

```python
from typing import List, Dict, Any
import tiktoken

class ContentChunker:
    def __init__(self, target_size: int = 1500, overlap: int = 200):
        self.target_size = target_size
        self.overlap = overlap
        self.encoder = tiktoken.encoding_for_model("text-embedding-3-small")

    def chunk_content(self, content: str, metadata: Dict[str, Any]) -> List[Dict[str, Any]]:
        # Split content into sentences/paragraphs first
        paragraphs = self.split_by_paragraphs(content)

        chunks = []
        current_chunk = ""
        current_tokens = 0

        for paragraph in paragraphs:
            paragraph_tokens = len(self.encoder.encode(paragraph))

            if current_tokens + paragraph_tokens > self.target_size and current_chunk:
                # Finalize current chunk
                chunks.append({
                    "content": current_chunk.strip(),
                    "metadata": metadata.copy(),
                    "chunk_index": len(chunks)
                })

                # Start new chunk with overlap
                if self.overlap > 0:
                    overlap_content = self.get_overlap_content(current_chunk, self.overlap)
                    current_chunk = overlap_content + paragraph
                    current_tokens = len(self.encoder.encode(current_chunk))
                else:
                    current_chunk = paragraph
                    current_tokens = paragraph_tokens
            else:
                current_chunk += paragraph
                current_tokens += paragraph_tokens

        # Add final chunk
        if current_chunk.strip():
            chunks.append({
                "content": current_chunk.strip(),
                "metadata": metadata.copy(),
                "chunk_index": len(chunks)
            })

        return chunks

    def split_by_paragraphs(self, content: str) -> List[str]:
        # Split by double newlines first, then handle other separators
        paragraphs = content.split('\n\n')
        result = []

        for para in paragraphs:
            if len(self.encoder.encode(para)) > self.target_size * 2:
                # If paragraph is too large, split by sentences
                sentences = self.split_by_sentences(para)
                temp_para = ""

                for sentence in sentences:
                    if len(self.encoder.encode(temp_para + sentence)) > self.target_size:
                        if temp_para.strip():
                            result.append(temp_para.strip() + "\n\n")
                        temp_para = sentence
                    else:
                        temp_para += sentence

                if temp_para.strip():
                    result.append(temp_para.strip() + "\n\n")
            else:
                result.append(para + "\n\n")

        return [p for p in result if p.strip()]

    def split_by_sentences(self, text: str) -> List[str]:
        import re
        # Split by sentence endings
        sentences = re.split(r'(?<=[.!?])\s+', text)
        return [s + '.' if not s.endswith(('.', '!', '?')) else s for s in sentences if s.strip()]
```

### 3. Embedding Generation

```python
import asyncio
from openai import AsyncOpenAI
from typing import List

class EmbeddingGenerator:
    def __init__(self, batch_size: int = 100):
        self.client = AsyncOpenAI()
        self.batch_size = batch_size

    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        all_embeddings = []

        # Process in batches to respect API limits
        for i in range(0, len(texts), self.batch_size):
            batch = texts[i:i + self.batch_size]

            response = await self.client.embeddings.create(
                model="text-embedding-3-small",
                input=batch
            )

            batch_embeddings = [item.embedding for item in response.data]
            all_embeddings.extend(batch_embeddings)

        return all_embeddings
```

### 4. Qdrant Storage with Idempotency

```python
from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
import hashlib
from typing import List, Dict, Any

class QdrantStorage:
    def __init__(self, url: str, api_key: str):
        self.client = AsyncQdrantClient(url=url, api_key=api_key)
        self.collection_name = "book_chunks"

    async def store_chunks(self, chunks: List[Dict[str, Any]]) -> None:
        # Prepare points for batch insertion
        points = []

        for chunk in chunks:
            # Create unique ID based on content hash and metadata
            content_hash = hashlib.sha256(
                (chunk["content"] + str(chunk["metadata"])).encode()
            ).hexdigest()

            # Generate embedding
            embedding = await self.generate_single_embedding(chunk["content"])

            point = models.PointStruct(
                id=content_hash,
                vector=embedding,
                payload={
                    "content": chunk["content"],
                    "chapter": chunk["metadata"]["chapter"],
                    "section": chunk["metadata"]["section"],
                    "book_version": chunk["metadata"]["book_version"],
                    "page_number": chunk["metadata"].get("page_number"),
                    "source_file": chunk["metadata"].get("source_file"),
                    "chunk_index": chunk["chunk_index"],
                    "created_at": chunk["metadata"].get("created_at", "NOW()")  # Will be set in DB
                }
            )

            points.append(point)

        # Upsert points (this handles idempotency - existing points are updated)
        await self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

    async def generate_single_embedding(self, text: str) -> List[float]:
        response = await self.client.embeddings.create(
            model="text-embedding-3-small",
            input=text
        )
        return response.data[0].embedding

    async def check_content_exists(self, content: str, metadata: Dict[str, Any]) -> bool:
        content_hash = hashlib.sha256(
            (content + str(metadata)).encode()
        ).hexdigest()

        try:
            records = await self.client.retrieve(
                collection_name=self.collection_name,
                ids=[content_hash]
            )
            return len(records) > 0
        except:
            return False
```

### 5. Metadata Tracking in Neon Postgres

```python
import asyncpg
from typing import Dict, Any
import uuid
from datetime import datetime

class MetadataTracker:
    def __init__(self, db_pool):
        self.pool = db_pool

    async def track_ingestion(self,
                            source_file: str,
                            chunks_created: int,
                            metadata: Dict[str, Any]) -> str:
        ingestion_id = str(uuid.uuid4())

        async with self.pool.acquire() as conn:
            await conn.execute("""
                INSERT INTO ingestions (
                    id, source_file, chunks_created, metadata,
                    status, created_at, completed_at
                ) VALUES ($1, $2, $3, $4, 'completed', $5, $6)
            """,
            ingestion_id, source_file, chunks_created,
            json.dumps(metadata), datetime.utcnow(), datetime.utcnow())

        return ingestion_id

    async def mark_chunk_processed(self, chunk_id: str, ingestion_id: str):
        async with self.pool.acquire() as conn:
            await conn.execute("""
                INSERT INTO processed_chunks (
                    chunk_id, ingestion_id, processed_at
                ) VALUES ($1, $2, $3)
            """, chunk_id, ingestion_id, datetime.utcnow())
```

## Idempotency Implementation

### 1. Content Hashing Strategy
- Generate SHA-256 hash of content + metadata
- Use hash as unique identifier in Qdrant
- Check existence before ingestion

### 2. Duplicate Detection
```python
async def ingest_content_safely(content: str, metadata: Dict[str, Any]) -> Dict[str, Any]:
    # Step 1: Validate content
    validate_content(content, metadata)

    # Step 2: Chunk content
    chunker = ContentChunker()
    chunks = chunker.chunk_content(content, metadata)

    # Step 3: Check if content already exists (idempotency)
    existing_count = 0
    for chunk in chunks:
        content_hash = hashlib.sha256(
            (chunk["content"] + str(chunk["metadata"])).encode()
        ).hexdigest()

        if await qdrant_storage.check_content_exists(chunk["content"], chunk["metadata"]):
            existing_count += 1

    # If all chunks exist, return early
    if existing_count == len(chunks):
        return {
            "status": "already_exists",
            "chunks_processed": len(chunks),
            "chunks_created": 0
        }

    # Step 4: Store in Qdrant
    await qdrant_storage.store_chunks(chunks)

    # Step 5: Track in database
    ingestion_id = await metadata_tracker.track_ingestion(
        source_file=metadata.get("source_file", "unknown"),
        chunks_created=len(chunks),
        metadata=metadata
    )

    return {
        "status": "success",
        "chunks_processed": len(chunks),
        "chunks_created": len(chunks),
        "ingestion_id": ingestion_id
    }
```

## Error Handling and Retry Logic

### 1. Retry Strategy
```python
import asyncio
from tenacity import retry, stop_after_attempt, wait_exponential

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=4, max=10)
)
async def robust_ingest_chunk(chunk: Dict[str, Any]):
    try:
        # Generate embedding
        embedding = await embedding_generator.generate_single_embedding(chunk["content"])

        # Store in Qdrant
        await qdrant_storage.store_single_chunk(chunk, embedding)

        return True
    except Exception as e:
        print(f"Error ingesting chunk: {str(e)}")
        raise  # Re-raise to trigger retry
```

### 2. Partial Failure Handling
- Track successfully processed chunks
- Resume from failure point
- Maintain consistency between Qdrant and database

## Performance Optimizations

### 1. Batch Processing
- Process multiple chunks in single API calls
- Batch embedding generation
- Batch Qdrant upserts

### 2. Parallel Processing
- Parallel embedding generation for independent chunks
- Concurrent Qdrant operations within rate limits
- Asynchronous database operations

### 3. Caching
- Cache embeddings for frequently ingested content
- Cache chunking results during development
- Cache metadata validation results

## Monitoring and Observability

### 1. Key Metrics
- Ingestion success rate
- Average processing time per document
- Embedding generation time
- Qdrant storage performance

### 2. Logging
- Log each ingestion attempt with metadata
- Track chunk-level processing status
- Monitor for idempotency violations

### 3. Alerts
- Alert on ingestion failures
- Alert on performance degradation
- Alert on unusual duplicate detection patterns

## Security Considerations

### 1. Content Validation
- Sanitize content before processing
- Validate file types and formats
- Check for malicious content patterns

### 2. API Key Security
- Secure storage of OpenAI and Qdrant API keys
- Use environment variables or secure vault
- Implement key rotation

### 3. Access Control
- Restrict ingestion API access
- Implement authentication for ingestion endpoints
- Rate limit ingestion requests