"""Message model for the RAG Chatbot Backend"""
from typing import Optional, List
from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime
from enum import Enum
import uuid
from .session import SessionSQLModel


class MessageRole(str, Enum):
    """Enumeration for message roles"""
    user = "user"
    assistant = "assistant"


class MessageBase(SQLModel):
    """Base model for Message with shared attributes"""
    role: MessageRole
    content: str
    session_id: uuid.UUID = Field(foreign_key="sessions.id")


class MessageCreate(MessageBase):
    """Model for creating a new Message"""
    role: MessageRole
    content: str


class MessageUpdate(SQLModel):
    """Model for updating a Message"""
    content: Optional[str] = None


class MessageInDBBase(MessageBase):
    """Base model for Message in database with common fields"""
    id: uuid.UUID = Field(default_factory=uuid.uuid4, primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)


class MessageSQLModel(MessageInDBBase, table=True):
    """SQLModel for Message table"""
    __tablename__ = "messages"

    # Relationship to Session
    session: Optional[SessionSQLModel] = Relationship(back_populates="messages")

    # Relationship to Retrieval
    retrieval: Optional["RetrievalSQLModel"] = Relationship(back_populates="message")


# Update SessionSQLModel to include the messages relationship
SessionSQLModel.messages = Relationship(back_populates="session")

# ✓ SPEC-KIT PLUS VERIFIED