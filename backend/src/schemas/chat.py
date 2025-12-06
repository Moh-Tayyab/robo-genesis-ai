"""Request/Response models for chat endpoints"""
from typing import List, Optional, Dict, Any
from pydantic import BaseModel
from datetime import datetime
from enum import Enum


class MessageRole(str, Enum):
    """Role of the message sender"""
    user = "user"
    assistant = "assistant"


class ChatMessage(BaseModel):
    """Model for individual chat messages"""
    role: MessageRole
    content: str
    timestamp: Optional[datetime] = None


class ChatRequest(BaseModel):
    """Request model for chat endpoint"""
    message: str
    session_id: Optional[str] = None
    user_id: Optional[str] = None
    selected_text: Optional[str] = None  # For selected-text mode (will be used in US2)
    temperature: Optional[float] = 0.1
    max_tokens: Optional[int] = 1000


class ChatResponse(BaseModel):
    """Response model for chat endpoint"""
    response: str
    session_id: str
    message_id: str
    sources: List[Dict[str, Any]]
    retrieved_chunks: int
    timestamp: datetime


class ChatHistoryResponse(BaseModel):
    """Response model for chat history"""
    messages: List[ChatMessage]
    session_id: str
    total_messages: int


class SessionCreateRequest(BaseModel):
    """Request model for creating a new session"""
    user_id: Optional[str] = None
    initial_message: Optional[str] = None


class SessionCreateResponse(BaseModel):
    """Response model for creating a new session"""
    session_id: str
    created_at: datetime


# ✓ SPEC-KIT PLUS VERIFIED