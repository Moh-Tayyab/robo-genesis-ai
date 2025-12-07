# Research: Panaversity Gemini RAG Backend

## Decision: OpenAI Agents SDK with Gemini Integration
**Rationale**: Using the OpenAI Agents SDK with Google Gemini via OpenAI-compatible endpoint allows leveraging existing agent patterns while using the free-tier Gemini 2.0 Flash model as specified. This approach provides structured agent interactions with tools for the RAG functionality.

**Alternatives considered**:
- Direct Gemini API calls: Would require custom agent orchestration
- LangChain agents: Would add additional dependency not specified in requirements
- Custom agent implementation: Would be more complex than using official SDK

## Decision: Two-Agent Architecture Pattern
**Rationale**: The specification requires exactly two agents with specific behaviors - BookRAGAgent for normal queries with retrieval tools and SelectedTextOnlyAgent for selected text queries with zero tools. This separation ensures proper handling of different query types with appropriate system prompts.

**Alternatives considered**:
- Single agent with conditional logic: Would not meet specification requirements
- Multiple specialized agents: Would exceed the two-agent requirement
- Rule-based routing: Would not leverage agent capabilities

## Decision: Qdrant Vector Database Integration
**Rationale**: Qdrant provides efficient vector similarity search required for RAG functionality. The specification specifically mentions Qdrant with API key and URL. It supports metadata filtering which is important for textbook content organization.

**Alternatives considered**:
- Pinecone: Would require different API integration
- Weaviate: Would require different schema definition
- ChromaDB: Would require different client implementation
- Custom vector search: Would be more complex than using established solution

## Decision: FastAPI Framework
**Rationale**: FastAPI provides async support, automatic API documentation, and type validation which are essential for the RAG backend. The specification mentions FastAPI endpoints, making it the natural choice.

**Alternatives considered**:
- Flask: Would lack async support and automatic documentation
- Django: Would be overkill for API-only backend
- Express.js: Would require switching to JavaScript/Node.js

## Decision: Async Lifespan with Error Handling
**Rationale**: The specification requires that the app starts even if one service is slow, but health reports false. Async lifespan allows proper initialization of database and Qdrant connections while preventing crashes during startup.

**Alternatives considered**:
- Synchronous initialization: Could block startup unnecessarily
- No error handling: Would violate specification requirement to not crash
- Separate initialization service: Would add unnecessary complexity