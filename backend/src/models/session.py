"""Session model for the RAG Chatbot Backend"""
from typing import Optional, List
from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime
import uuid
from .user import UserSQLModel


class SessionBase(SQLModel):
    """Base model for Session with shared attributes"""
    title: str = Field(max_length=200)
    user_id: Optional[uuid.UUID] = Field(default=None, foreign_key="users.id")


class SessionCreate(SessionBase):
    """Model for creating a new Session"""
    title: Optional[str] = None  # Can be generated from first query
    user_id: Optional[uuid.UUID] = None


class SessionUpdate(SessionBase):
    """Model for updating a Session"""
    title: Optional[str] = None


class SessionInDBBase(SessionBase):
    """Base model for Session in database with common fields"""
    id: uuid.UUID = Field(default_factory=uuid.uuid4, primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)


class SessionSQLModel(SessionInDBBase, table=True):
    """SQLModel for Session table"""
    __tablename__ = "sessions"

    # Relationship to User
    user: Optional[UserSQLModel] = Relationship(back_populates="user_sessions")

    # Relationship to Messages
    messages: List["MessageSQLModel"] = Relationship(back_populates="session")


# ✓ SPEC-KIT PLUS VERIFIED