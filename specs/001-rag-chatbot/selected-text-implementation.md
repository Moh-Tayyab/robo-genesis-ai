# Selected-Text RAG Implementation: Zero Leakage Architecture

**Feature**: 001-rag-chatbot
**Date**: 2025-12-06

## Overview

This document outlines the implementation plan for the selected-text RAG mode with zero leakage. The implementation ensures that when `use_selected` is True, the system exclusively uses the provided selected text as context, with no access to the broader book content or vector store.

## Requirements

### Functional Requirements
1. **Exclusive Context**: When `use_selected=True`, only the provided `selected_text` should be used as context
2. **No Leakage**: No access to vector store or book content during selected-text mode
3. **Fallback Response**: If answer cannot be found in selected text, return: "I can only answer based on the selected text, and the answer is not present there."
4. **Context Isolation**: Complete separation between selected-text and full-book modes
5. **Response Quality**: Maintain high-quality responses within the selected context

### Non-Functional Requirements
1. **Performance**: ≤ 1 second response time
2. **Security**: No unauthorized access to broader content
3. **Validation**: Selected text length ≤ 4000 characters
4. **Error Handling**: Graceful handling of edge cases

## Architecture Design

### 1. Isolated Processing Pipeline

```
Input: question + selected_text + use_selected=true
    ↓
Validation Layer (length, content checks)
    ↓
Context-Isolated Agent (SelectedTextAgent)
    ↓
Response Generation (with fallback logic)
    ↓
Output: answer OR fallback message
```

### 2. SelectedTextAgent Implementation

#### Agent Configuration
- **Name**: "SelectedTextAgent"
- **Model**: gpt-4o-mini
- **System Prompt**: Strict instructions to only use provided context
- **Tools**: None (or minimal tools that don't access external content)

#### System Instructions
```
You are an AI assistant that can ONLY answer questions based on the provided selected text.
You must NOT use any external knowledge or access to broader content.
If the answer to the question is not present in the selected text,
respond with exactly: "I can only answer based on the selected text, and the answer is not present there."
```

### 3. Context Isolation Mechanisms

#### A. Code-Level Isolation
- Separate processing functions for selected-text vs full-book modes
- No shared vector store access in selected-text path
- Dedicated SelectedTextAgent with no vector store tools

#### B. Data-Level Isolation
- Selected text passed as direct parameter, not retrieved from store
- No database queries during selected-text processing
- No vector store queries during selected-text processing

#### C. Agent-Level Isolation
- Dedicated SelectedTextAgent with restricted capabilities
- BookAgent delegates to SelectedTextAgent when `use_selected=True`
- No cross-agent communication that could leak context

## Implementation Strategy

### 1. Request Routing

```python
async def process_chat_request(chat_request: ChatRequest):
    if chat_request.use_selected:
        return await process_selected_text_mode(
            selected_text=chat_request.selected_text,
            question=chat_request.question
        )
    else:
        return await process_full_book_mode(
            question=chat_request.question
        )
```

### 2. Selected Text Processing Function

```python
async def process_selected_text_mode(selected_text: str, question: str):
    # Validation
    if not selected_text or len(selected_text) > 4000:
        raise ValueError("Selected text must be between 1-4000 characters")

    # Context preparation
    context = f"Context: {selected_text}\n\nQuestion: {question}"

    # Use isolated agent
    response = await openai_client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {
                "role": "system",
                "content": "Answer ONLY based on the provided context. If the answer is not in the context, respond with: 'I can only answer based on the selected text, and the answer is not present there.'"
            },
            {
                "role": "user",
                "content": context
            }
        ],
        max_tokens=500,
        temperature=0.1  # Lower temperature for more consistent responses
    )

    answer = response.choices[0].message.content

    return ChatResponse(
        answer=answer,
        sources=None,
        session_id=generate_or_use_existing_session_id()
    )
```

### 3. Fallback Response Implementation

The system implements a two-layer approach:
1. **Primary**: Let the AI model determine if answer exists in context
2. **Secondary**: If response seems to go beyond context, validate and potentially replace

```python
def validate_selected_text_response(response: str, selected_text: str) -> str:
    # Check if response contains clear indicators of not finding the answer
    fallback_phrase = "I can only answer based on the selected text, and the answer is not present there."

    # If the response already contains the fallback message, return as is
    if fallback_phrase in response:
        return response

    # Additional validation could be implemented here if needed
    # For now, we trust the AI's adherence to system instructions
    return response
```

## Zero Leakage Guarantees

### 1. Code Separation
- Separate code paths for selected-text and full-book modes
- No shared functions that access vector store in selected-text path
- Clear boundaries between different processing modes

### 2. Runtime Isolation
- SelectedTextAgent has no tools that access vector store
- No database connections during selected-text processing
- No external API calls beyond OpenAI during selected-text processing

### 3. Configuration Isolation
- Different system prompts for different modes
- SelectedTextAgent configured without vector store access
- Environment variables and settings separated by mode

## Validation and Testing

### 1. Unit Tests
- Test that selected-text mode doesn't call vector store functions
- Test fallback response generation
- Test context length validation
- Test session management in isolation

### 2. Integration Tests
- End-to-end test of selected-text flow
- Verify no database queries during selected-text processing
- Verify no vector store queries during selected-text processing
- Test edge cases and error conditions

### 3. Security Tests
- Attempt to force access to broader content during selected-text mode
- Test injection attacks through selected text
- Verify isolation boundaries are maintained

## Error Handling

### 1. Input Validation Errors
- Selected text too long (>4000 chars)
- Selected text too short (empty)
- Invalid characters or encoding

### 2. Processing Errors
- OpenAI API errors during selected-text processing
- Network timeouts
- Invalid responses from model

### 3. Fallback Handling
- Ensure fallback message is returned when appropriate
- Handle cases where model doesn't follow instructions
- Graceful degradation if isolation mechanisms fail

## Performance Considerations

### 1. Response Time
- Selected-text mode should be faster than full-book mode
- No vector store queries = reduced latency
- Optimize OpenAI API calls for context length

### 2. Resource Usage
- Lower resource usage in selected-text mode
- No vector store connections
- No database queries for retrieval

### 3. Caching
- Cache selected-text processing results if applicable
- Consider caching for repeated queries on same text

## Security Considerations

### 1. Input Sanitization
- Sanitize selected text input to prevent injection
- Validate content before processing
- Check for malicious content patterns

### 2. Context Boundary Enforcement
- Ensure selected-text boundary is never crossed
- Validate that no external content is accessed
- Monitor for potential leakage patterns

### 3. Access Control
- Ensure selected-text mode doesn't bypass any security measures
- Maintain session tracking and rate limiting
- Preserve user anonymity requirements

## Monitoring and Observability

### 1. Logging
- Log when selected-text mode is used
- Track fallback response frequency
- Monitor for potential leakage attempts

### 2. Metrics
- Response time for selected-text mode
- Success vs fallback response rates
- Error rates and types

### 3. Alerts
- Alert on potential context leakage
- Alert on high fallback response rates
- Alert on performance degradation

## Deployment Considerations

### 1. Configuration
- Environment-specific configuration for isolation
- Feature flags for selected-text mode
- Security settings for context isolation

### 2. Rollout Strategy
- Gradual rollout of selected-text functionality
- A/B testing with and without isolation
- Monitoring during rollout phase

### 3. Rollback Plan
- Quick rollback if leakage is detected
- Fallback to full-book mode if needed
- Data integrity during rollback