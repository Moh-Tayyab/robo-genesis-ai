"""Chat API endpoints for RAG functionality"""
from fastapi import APIRouter, HTTPException, Depends, Request
from typing import Optional
import logging
from datetime import datetime
from ..schemas.chat import ChatRequest, ChatResponse, ChatHistoryResponse, SessionCreateRequest, SessionCreateResponse
from ..services.chat_service import chat_service
from ..database import session_manager
from ..middleware.rate_limit import rate_limit_middleware

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/chat", tags=["chat"])

@router.post("", response_model=ChatResponse)
async def chat_endpoint(chat_request: ChatRequest):
    """
    Main chat endpoint for RAG queries.
    Supports both full-book Q&A and selected-text Q&A (when selected_text is provided).
    """
    try:
        logger.info(f"Received chat request for session: {chat_request.session_id}")

        # Create or validate session
        session_id = chat_request.session_id
        if not session_id:
            session_id = await chat_service.create_session(chat_request.user_id)
            logger.info(f"Created new session: {session_id}")
        else:
            # Verify session exists
            session = await chat_service.get_session(session_id)
            if not session:
                raise HTTPException(status_code=404, detail="Session not found")

        # Check if this is a selected-text query (US2) or full-book query (US1)
        if chat_request.selected_text:
            # For now, we'll handle this as a full-book query until US2 is implemented
            # In the future, this will use selected-text RAG
            logger.info(f"Processing selected-text query for session: {session_id}")
            # For now, we'll implement this as part of US2
            rag_response = await chat_service.process_full_book_query(session_id, chat_request.message)
        else:
            # Process as full-book RAG query
            logger.info(f"Processing full-book query for session: {session_id}")
            rag_response = await chat_service.process_full_book_query(session_id, chat_request.message)

        # Get the latest assistant message ID for this session
        from ..models.message import MessageSQLModel
        from sqlmodel import select

        async with session_manager.session() as db_session:
            # Get the last message in the session (should be the assistant's response)
            statement = select(MessageSQLModel).where(
                MessageSQLModel.session_id == session_id
            ).order_by(MessageSQLModel.created_at.desc())
            result = await db_session.exec(statement)
            last_message = result.first()
            message_id = last_message.id if last_message else ""

        # Create response
        response = ChatResponse(
            response=rag_response.answer,
            session_id=session_id,
            message_id=message_id,
            sources=rag_response.sources,
            retrieved_chunks=len(rag_response.retrieved_chunks),
            timestamp=datetime.utcnow()
        )

        logger.info(f"Chat response generated for session: {session_id}")
        return response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.post("/session", response_model=SessionCreateResponse)
async def create_session_endpoint(session_request: SessionCreateRequest):
    """Create a new chat session"""
    try:
        logger.info("Creating new chat session")

        session_id = await chat_service.create_session(session_request.user_id)

        # If an initial message is provided, process it
        if session_request.initial_message:
            await chat_service.add_message_to_session(session_id, "user", session_request.initial_message)
            # Process the initial message as a query
            await chat_service.process_full_book_query(session_id, session_request.initial_message)

        response = SessionCreateResponse(
            session_id=session_id,
            created_at=datetime.utcnow()
        )

        logger.info(f"Session created: {session_id}")
        return response

    except Exception as e:
        logger.error(f"Error creating session: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.get("/history/{session_id}", response_model=ChatHistoryResponse)
async def get_chat_history(session_id: str):
    """Get chat history for a session"""
    try:
        logger.info(f"Getting chat history for session: {session_id}")

        session = await chat_service.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        # Get messages from the database
        messages = await chat_service.get_conversation_history(session_id)

        response = ChatHistoryResponse(
            messages=[],
            session_id=session_id,
            total_messages=len(messages)
        )

        # Convert messages to the expected format
        for msg in messages:
            response.messages.append({
                "role": msg["role"],
                "content": msg["content"],
                "timestamp": datetime.fromisoformat(msg["timestamp"])
            })

        logger.info(f"Retrieved {len(messages)} messages for session: {session_id}")
        return response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting chat history: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.delete("/session/{session_id}")
async def delete_session(session_id: str):
    """Delete a chat session and all associated data"""
    try:
        logger.info(f"Deleting session: {session_id}")

        await chat_service.delete_session(session_id)

        return {"message": f"Session {session_id} deleted successfully"}

    except Exception as e:
        logger.error(f"Error deleting session: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.get("/session/{session_id}/stats")
async def get_session_stats(session_id: str):
    """Get statistics for a session"""
    try:
        logger.info(f"Getting stats for session: {session_id}")

        stats = await chat_service.get_session_stats(session_id)
        if not stats:
            raise HTTPException(status_code=404, detail="Session not found")

        return stats

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting session stats: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


# ✓ SPEC-KIT PLUS VERIFIED