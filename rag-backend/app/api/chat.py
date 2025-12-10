from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional
from app.database import get_db
from app.schemas.chat import ChatRequest, ChatResponse, SelectedTextChatRequest
from app.services.chat_service import chat_service
from app.core.config import settings
from app.services.agent_service import agent_rag_service
import logging

logger = logging.getLogger(__name__)

router = APIRouter()

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(
    request: ChatRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Full-book RAG endpoint that answers questions based on the entire textbook content using OpenAI Agents.
    """
    try:
        # Process the query using the agent-based RAG service
        result = await chat_service.process_full_book_query(
            question=request.question,
            session_id=request.sessionId,
            user_id=request.userId
        )

        # Convert the result to the expected response format
        response = ChatResponse(
            response=result["response"],
            sources=result["sources"],
            sessionId=result["sessionId"],
            retrievalMetadata=result["retrievalMetadata"]
        )

        return response
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")

@router.post("/chat/selected", response_model=ChatResponse)
async def chat_selected_endpoint(
    request: SelectedTextChatRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Selected-text RAG endpoint that answers questions based only on the provided text selection using OpenAI Agents.
    """
    try:
        # Validate selected text length
        if len(request.selectedText) > settings.max_selected_text_length:
            raise HTTPException(
                status_code=400,
                detail=f"Selected text too long: {len(request.selectedText)} characters. Maximum allowed: {settings.max_selected_text_length}"
            )

        # Process the query using the agent-based RAG service
        result = await chat_service.process_selected_text_query(
            question=request.question,
            selected_text=request.selectedText,
            session_id=request.sessionId,
            user_id=request.userId
        )

        # Convert the result to the expected response format
        response = ChatResponse(
            response=result["response"],
            sources=result["sources"],
            sessionId=result["sessionId"],
            retrievalMetadata=result["retrievalMetadata"]
        )

        return response
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error in chat selected endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")