# BookAgent Architecture: RAG Chatbot Backend

**Feature**: 001-rag-chatbot
**Date**: 2025-12-06

## Overview

This document defines the architecture for the BookAgent, which serves as the main orchestrator for the RAG Chatbot Backend. The BookAgent implements the required functionality using OpenAI Assistant API with custom tools and proper delegation to a specialized SelectedTextAgent when needed.

## Architecture Components

### 1. BookAgent Structure

The BookAgent is implemented using OpenAI's Assistant API and consists of:

- **Name**: "BookAgent"
- **Model**: gpt-4o-mini (as specified in requirements)
- **Tools**:
  - `retrieve_context` - for vector store retrieval
  - `finalize_answer` - for generating final responses
- **Instructions**: System prompt that defines the agent's behavior

### 2. Tool Definitions

#### Tool 1: retrieve_context

**Purpose**: Retrieve relevant context from the vector store based on the query.

**Function Definition**:
```json
{
  "name": "retrieve_context",
  "description": "Retrieve relevant context from the book content based on the query",
  "parameters": {
    "type": "object",
    "properties": {
      "query": {
        "type": "string",
        "description": "The query to search for in the vector store"
      },
      "use_selected": {
        "type": "boolean",
        "description": "Whether to use selected text mode (if true, agent should delegate to SelectedTextAgent)"
      },
      "selected_text": {
        "type": "string",
        "description": "The selected text to use for context (when use_selected is true)"
      }
    },
    "required": ["query", "use_selected"]
  }
}
```

**Implementation**:
- If `use_selected` is true, the tool should trigger delegation to SelectedTextAgent
- If `use_selected` is false, perform vector search in Qdrant with HyDE + MMR
- Return top-8 relevant chunks with metadata

#### Tool 2: finalize_answer

**Purpose**: Generate the final answer based on retrieved context.

**Function Definition**:
```json
{
  "name": "finalize_answer",
  "description": "Generate the final answer based on the provided context",
  "parameters": {
    "type": "object",
    "properties": {
      "context": {
        "type": "string",
        "description": "The context to use for generating the answer"
      },
      "question": {
        "type": "string",
        "description": "The original question"
      },
      "is_selected_mode": {
        "type": "boolean",
        "description": "Whether this is in selected text mode"
      }
    },
    "required": ["context", "question", "is_selected_mode"]
  }
}
}
```

**Implementation**:
- Format the response based on the context and question
- If in selected mode and answer not found, return the specific fallback message
- Include source citations when available

### 3. SelectedTextAgent Structure

The SelectedTextAgent is a specialized agent for handling selected text mode:

- **Name**: "SelectedTextAgent"
- **Model**: gpt-4o-mini (as specified in requirements)
- **Tools**: Custom tools for processing selected text only
- **Instructions**: System prompt that restricts the agent to only use provided text

## Agent Flow

### 1. Full-book Q&A Flow

```
User Request → BookAgent → retrieve_context(query) → finalize_answer(context, question) → Response
```

1. BookAgent receives the question
2. BookAgent calls `retrieve_context` with the query (use_selected=false)
3. `retrieve_context` performs vector search in Qdrant using HyDE + MMR to get top-8 chunks
4. BookAgent calls `finalize_answer` with retrieved context
5. `finalize_answer` generates response with source citations

### 2. Selected-text Q&A Flow

```
User Request → BookAgent → [Delegation Trigger] → SelectedTextAgent → Response
```

1. BookAgent receives the question with selected text
2. BookAgent detects `use_selected=true` and delegates to SelectedTextAgent
3. SelectedTextAgent processes question exclusively with provided selected text
4. SelectedTextAgent returns response (or fallback message if not found)

## Implementation Details

### 1. Agent Creation

```python
from openai import OpenAI

client = OpenAI()

# Create BookAgent with tools
book_agent = client.beta.assistants.create(
    name="BookAgent",
    instructions="You are an AI assistant for a Physical AI & Humanoid Robotics textbook. Use the provided tools to retrieve context and generate answers.",
    model="gpt-4o-mini",
    tools=[
        {
            "type": "function",
            "function": {
                "name": "retrieve_context",
                "description": "Retrieve relevant context from the book content based on the query",
                "parameters": retrieve_context_schema
            }
        },
        {
            "type": "function",
            "function": {
                "name": "finalize_answer",
                "description": "Generate the final answer based on the provided context",
                "parameters": finalize_answer_schema
            }
        }
    ]
)

# Create SelectedTextAgent
selected_agent = client.beta.assistants.create(
    name="SelectedTextAgent",
    instructions="Answer ONLY based on the provided selected text. If the answer is not in the selected text, respond with: 'I can only answer based on the selected text, and the answer is not present there.'",
    model="gpt-4o-mini",
    tools=[
        # Specialized tools for selected text processing
    ]
)
```

### 2. Tool Function Implementation

```python
import json
from typing import Dict, Any
from qdrant_client import QdrantClient
from openai import OpenAI

class AgentTools:
    def __init__(self):
        self.qdrant_client = QdrantClient(url="YOUR_QDRANT_URL", api_key="YOUR_API_KEY")
        self.openai_client = OpenAI()

    def retrieve_context(self, query: str, use_selected: bool, selected_text: str = None) -> Dict[str, Any]:
        if use_selected:
            # Trigger delegation to SelectedTextAgent
            return {
                "delegation_required": True,
                "selected_text": selected_text,
                "query": query
            }
        else:
            # Perform vector search with HyDE + MMR
            # 1. Generate hypothetical document
            hypothetical_doc = self.generate_hypothetical_document(query)
            hypothetical_embedding = self.embed_text(hypothetical_doc)

            # 2. Retrieve candidates using hypothetical embedding
            candidates = self.qdrant_client.search(
                collection_name="book_chunks",
                query_vector=hypothetical_embedding,
                limit=16,  # Get more candidates for MMR
                with_payload=True
            )

            # 3. Apply MMR to select top-8 diverse chunks
            selected_chunks = self.apply_mmr_selection(
                candidates=candidates,
                query_embedding=self.embed_text(query),
                top_k=8
            )

            return {
                "chunks": selected_chunks,
                "query": query
            }

    def finalize_answer(self, context: str, question: str, is_selected_mode: bool) -> str:
        # Generate response using the context and question
        response = self.openai_client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "system",
                    "content": "You are an AI assistant for a Physical AI & Humanoid Robotics textbook. Provide accurate answers based on the provided context."
                },
                {
                    "role": "user",
                    "content": f"Context: {context}\n\nQuestion: {question}"
                }
            ],
            max_tokens=500
        )

        return response.choices[0].message.content
```

### 3. Thread Management

```python
class AgentManager:
    def __init__(self):
        self.client = OpenAI()
        self.tools = AgentTools()

    async def process_request(self, question: str, use_selected: bool = False, selected_text: str = None):
        # Create a thread for this conversation
        thread = self.client.beta.threads.create()

        # Add the user's message to the thread
        self.client.beta.threads.messages.create(
            thread_id=thread.id,
            role="user",
            content=question
        )

        if use_selected:
            # Process with SelectedTextAgent
            response = await self.process_with_selected_agent(selected_text, question)
        else:
            # Process with BookAgent
            run = self.client.beta.threads.runs.create(
                thread_id=thread.id,
                assistant_id=book_agent.id,
                # Include additional context for tool calls
            )

            # Wait for the run to complete
            response = await self.wait_for_run_completion(thread.id, run.id)

        return response
```

## Delegation Pattern

### 1. Detection of Selected Text Mode
- BookAgent detects when `use_selected` is True in the request
- BookAgent recognizes that it should not access the vector store
- BookAgent triggers delegation to SelectedTextAgent

### 2. Handoff Process
- BookAgent passes the selected text and question to SelectedTextAgent
- SelectedTextAgent operates independently with only the provided context
- SelectedTextAgent returns response directly to the caller

### 3. Context Isolation
- SelectedTextAgent has no access to vector store or book content
- SelectedTextAgent is restricted to only use the provided selected text
- No cross-contamination between selected text and full book modes

## Error Handling

### 1. Tool Call Errors
- Handle OpenAI API errors gracefully
- Handle Qdrant connection errors
- Implement retry logic for transient failures

### 2. Delegation Errors
- Handle failures during agent handoff
- Fallback to safe response if delegation fails
- Log errors for debugging and monitoring

### 3. Validation Errors
- Validate input parameters before processing
- Check selected text length limits
- Ensure required fields are present

## Performance Considerations

### 1. Tool Response Time
- Optimize vector search queries for speed
- Implement caching for frequently accessed content
- Use async operations where possible

### 2. Agent Response Time
- Monitor and optimize for ≤ 1 second response time
- Implement timeout handling for external API calls
- Use efficient MMR implementation

## Security Considerations

### 1. API Key Management
- Secure storage of OpenAI, Qdrant, and Neon API keys
- Use environment variables or secure vault
- Rotate keys regularly

### 2. Input Validation
- Validate all user inputs
- Sanitize text inputs to prevent injection
- Check content length limits

### 3. Agent Instructions
- Ensure agent instructions prevent harmful responses
- Maintain context isolation between modes
- Implement proper content filtering