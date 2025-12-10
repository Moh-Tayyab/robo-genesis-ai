# Feature Specification: RAG Chatbot Backend

**Feature Branch**: `001-rag-chatbot`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Physical AI & Humanoid Robotics – Integrated RAG Chatbot Backend (implements Module-level chatbot that answers ONLY from book content + selected-text Q&A)"

## Clarifications

### Session 2025-12-06

- Q: Which OpenAI model should be used for the RAG chatbot? → A: Use gpt-4o-mini as specified in original requirements
- Q: What should be the rate limiting strategy? → A: Use 30 req/min per IP, 10 req/min per session as specified in original requirements
- Q: Which OpenAI embedding model should be used? → A: Use text-embedding-3-small as specified in original requirements
- Q: What should be the Qdrant search parameters? → A: Use top-k=5, score_threshold ≥ 0.75 as specified in original requirements

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Full-book Q&A (Priority: P1)

Student asks questions about the textbook content and receives accurate answers based on the entire book.

**Why this priority**: This is the core functionality that allows students to get answers from the textbook content.

**Independent Test**: Can be fully tested by asking questions and verifying the responses come from the book content with proper citations.

**Acceptance Scenarios**:

1. **Given** textbook content is indexed in vector database, **When** student asks a question, **Then** chatbot returns accurate answer citing relevant chapters and sections
2. **Given** textbook content is indexed in vector database, **When** student asks a question not covered in the book, **Then** chatbot responds with "Not in selection" or similar

---

### User Story 2 - Selected-text Q&A (Priority: P2)

Student selects specific text in the textbook and asks questions about only that text, receiving answers constrained to the selected context.

**Why this priority**: This provides a focused Q&A experience that respects context boundaries.

**Independent Test**: Can be tested by selecting text and asking questions that should only be answered from that specific text.

**Acceptance Scenarios**:

1. **Given** student has selected text in textbook, **When** student asks a question about the selection, **Then** chatbot returns answer based only on the selected text
2. **Given** student has selected text in textbook, **When** student asks a question not covered in the selection, **Then** chatbot responds with "Not in selection"

---

### User Story 3 - Anonymous Usage Tracking (Priority: P3)

System tracks usage patterns anonymously for improving the chatbot experience.

**Why this priority**: Understanding usage helps improve the system over time while maintaining user privacy.

**Independent Test**: Can be tested by using the chatbot and verifying that usage data is collected without personal identification.

**Acceptance Scenarios**:

1. **Given** student uses the chatbot, **When** conversation occurs, **Then** anonymized usage data is stored for analytics

---

### Edge Cases

- What happens when selected text is longer than 4000 characters?
- How does system handle Qdrant vector search failures?
- What happens when OpenAI API is unavailable?
- How does system handle concurrent users hitting rate limits?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST embed RAG chatbot in Docusaurus textbook interface
- **FR-002**: System MUST use OpenAI Agents SDK with gpt-4o-mini model for chat functionality
- **FR-003**: System MUST store anonymized {userId, chatHistory, retrievalMetadata} in Neon Postgres
- **FR-004**: System MUST use Qdrant Cloud Free Tier with text-embedding-3-small embeddings for vector storage
- **FR-005**: System MUST implement ≤ 1 s retrieval latency for responses
- **FR-006**: System MUST achieve ≥ 90% accuracy on 20 book-Q&A pairs during testing
- **FR-007**: System MUST provide selected-text Q&A with "not in selection" fallback
- **FR-008**: System MUST NOT perform runtime web search (only uses book content)
- **FR-009**: System MUST provide POST /chat endpoint for full-book RAG
- **FR-010**: System MUST provide POST /chat/selected endpoint for selected-text RAG
- **FR-011**: System MUST provide POST /ingest endpoint for idempotent content ingestion
- **FR-012**: System MUST provide GET /health endpoint for monitoring
- **FR-013**: System MUST implement rate limiting of 30 req/min per IP and 10 req/min per session
- **FR-014**: System MUST use top-k=5 and score_threshold ≥ 0.75 for Qdrant vector search

### Key Entities *(include if feature involves data)*

- **User**: Anonymous user identified by UUID for tracking purposes
- **Session**: Chat session with metadata about the conversation
- **Message**: Individual user or assistant message in a conversation
- **Retrieval**: Record of which content chunks were retrieved for a message

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Chatbot responds with ≤ 1 second retrieval latency in 95% of requests
- **SC-002**: Chatbot achieves ≥ 90% accuracy on 20 pre-defined book-Q&A pairs
- **SC-003**: Selected-text Q&A correctly restricts answers to selected context with "not in selection" fallback when appropriate
- **SC-004**: System handles no runtime web search and only uses book content as intended