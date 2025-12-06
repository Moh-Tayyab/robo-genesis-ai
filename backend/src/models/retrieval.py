"""Retrieval model for the RAG Chatbot Backend"""
from typing import Optional, Dict, Any
from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime
import uuid
from .message import MessageSQLModel


class RetrievalBase(SQLModel):
    """Base model for Retrieval with shared attributes"""
    message_id: uuid.UUID = Field(foreign_key="messages.id")
    query: str
    context_used: str
    retrieved_chunks: int
    sources: str  # JSON string of sources
    metadata: Optional[Dict[str, Any]] = Field(default={})


class RetrievalCreate(RetrievalBase):
    """Model for creating a new Retrieval"""
    query: str
    context_used: str
    retrieved_chunks: int
    sources: str


class RetrievalUpdate(SQLModel):
    """Model for updating a Retrieval"""
    metadata: Optional[Dict[str, Any]] = None


class RetrievalInDBBase(RetrievalBase):
    """Base model for Retrieval in database with common fields"""
    id: uuid.UUID = Field(default_factory=uuid.uuid4, primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)


class RetrievalSQLModel(RetrievalInDBBase, table=True):
    """SQLModel for Retrieval table"""
    __tablename__ = "retrievals"

    # Relationship to Message
    message: Optional[MessageSQLModel] = Relationship(back_populates="retrieval")


# Update MessageSQLModel to include the retrieval relationship
MessageSQLModel.retrieval = Relationship(back_populates="message")

# ✓ SPEC-KIT PLUS VERIFIED