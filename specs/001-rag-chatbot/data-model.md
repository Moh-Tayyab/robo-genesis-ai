# Data Model: RAG Chatbot Backend

## Entities

### User
- **userId**: UUID (Primary Key)
- **createdAt**: DateTime
- **email**: String (Optional)
- **preferences**: JSON (Optional)

### Session
- **sessionId**: UUID (Primary Key)
- **userId**: UUID (Foreign Key to User)
- **startedAt**: DateTime
- **endedAt**: DateTime (Optional)
- **metadata**: JSON (Optional)

### Message
- **msgId**: UUID (Primary Key)
- **sessionId**: UUID (Foreign Key to Session)
- **role**: Enum('user', 'assistant')
- **text**: Text
- **createdAt**: DateTime
- **metadata**: JSON (Optional)

### Retrieval
- **retrievalId**: UUID (Primary Key)
- **msgId**: UUID (Foreign Key to Message)
- **chunkId**: String
- **chapter**: String
- **section**: String
- **url**: String
- **latencyMs**: Integer
- **createdAt**: DateTime

## Qdrant Collection Schema: "book-chunks"

### Vector Configuration
- **Vectors**: 1536-dim (text-embedding-3-small)
- **Distance**: Cosine

### Payload Fields
- **chunkId**: String
- **chapter**: String
- **section**: String
- **url**: String
- **difficulty**: String (enum: beginner, intermediate, advanced)
- **lang**: String (default: en)
- **textHash**: String
- **text**: String (the actual content chunk)

## Validation Rules

### User
- userId must be valid UUID
- createdAt must be in past

### Session
- sessionId must be valid UUID
- userId must reference existing user
- startedAt must be before endedAt (if present)

### Message
- msgId must be valid UUID
- sessionId must reference existing session
- role must be 'user' or 'assistant'
- text must not be empty
- createdAt must be in past

### Retrieval
- retrievalId must be valid UUID
- msgId must reference existing message
- latencyMs must be positive
- createdAt must be in past

## State Transitions

### Session
- **Active** → **Ended**: When session is closed (endedAt is set)
- **Created** → **Active**: When first message is added

## Relationships

- **User** (1) → (0..n) **Session**
- **Session** (1) → (0..n) **Message**
- **Message** (1) → (0..n) **Retrieval**