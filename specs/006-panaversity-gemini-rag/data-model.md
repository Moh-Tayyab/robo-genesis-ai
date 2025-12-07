# Data Model: Panaversity Gemini RAG Backend

## Content Chunk Entity
**Description**: Represents a segment of textbook content stored in the vector database

**Fields**:
- `id` (string): Unique identifier for the chunk
- `content` (string): The actual text content
- `chapter` (string): Chapter identifier from the textbook
- `section` (string): Section within the chapter
- `book_version` (string): Version of the textbook (default: "latest")
- `page_number` (integer, nullable): Page number in the original textbook
- `source_file` (string, nullable): Original source file name
- `chunk_index` (integer, nullable): Sequential index of the chunk
- `metadata` (object, nullable): Additional metadata as key-value pairs
- `embedding` (array of floats, nullable): Vector embedding for similarity search

**Relationships**:
- Belongs to: Book (via chapter/section)
- Referenced by: Query sessions

**Validation rules**:
- `content` must not be empty
- `chapter` and `section` must be valid identifiers
- `embedding` must be a valid vector (if present)

## Query Request Entity
**Description**: Represents a user query request to the RAG system

**Fields**:
- `message` (string): The user's question or query
- `selected_text` (string, nullable): Specific text selected by the user (for SelectedTextOnlyAgent)
- `use_selected_only` (boolean): Flag to indicate if only selected text should be used

**Validation rules**:
- `message` must not be empty
- `use_selected_only` defaults to false
- When `use_selected_only` is true, `selected_text` must not be null

## Query Response Entity
**Description**: Represents the response from the RAG system to a query

**Fields**:
- `response` (string): The generated response text
- `agent_type` (string): Type of agent that processed the query ("BookRAGAgent" or "SelectedTextOnlyAgent")
- `success` (boolean): Whether the query was processed successfully
- `retrieved_chunks` (array, optional): List of chunks retrieved for the response (BookRAGAgent only)

**Validation rules**:
- `response` must not be empty when success is true
- `agent_type` must be one of the allowed values

## Health Status Entity
**Description**: Represents the health status of system components

**Fields**:
- `status` (string): Overall system status ("healthy", "unhealthy")
- `gemini` (boolean): Status of Gemini API connection
- `qdrant` (boolean): Status of Qdrant vector database connection
- `neon` (boolean): Status of Neon PostgreSQL database connection

**Validation rules**:
- `status` is "healthy" only when all services are true
- All service status fields must be boolean values

## Ingestion Request Entity
**Description**: Represents a request to ingest textbook content chunks

**Fields**:
- `chunks` (array): Array of content chunk objects to be ingested
- `chunks[].text` (string): The text content to be ingested
- `chunks[].metadata` (object): Metadata for the chunk (chapter, section, etc.)

**Validation rules**:
- `chunks` array must not be empty
- Each chunk must have non-empty text content
- Each chunk must have valid metadata structure