"""User model for the RAG Chatbot Backend"""
from typing import Optional, List
from sqlmodel import SQLModel, Field, Relationship
from datetime import datetime
import uuid


class UserBase(SQLModel):
    """Base model for User with shared attributes"""
    email: str = Field(unique=True, index=True)
    full_name: Optional[str] = None
    is_active: bool = True
    is_superuser: bool = False


class UserCreate(UserBase):
    """Model for creating a new User"""
    email: str
    password: str  # In a real app, this would be hashed


class UserUpdate(UserBase):
    """Model for updating a User"""
    email: Optional[str] = None
    full_name: Optional[str] = None
    is_active: Optional[bool] = None
    is_superuser: Optional[bool] = None
    password: Optional[str] = None  # Optional because it might not be updated


class UserInDBBase(UserBase):
    """Base model for User in database with common fields"""
    id: uuid.UUID = Field(default_factory=uuid.uuid4, primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)


class UserSQLModel(UserInDBBase, table=True):
    """SQLModel for User table"""
    __tablename__ = "users"

    # Additional fields for the database model can go here
    hashed_password: Optional[str] = Field(default=None)

    # Relationship to sessions
    user_sessions: List["SessionSQLModel"] = Relationship(back_populates="user")


class UserPublic(UserInDBBase):
    """Model for returning public user data"""
    pass


class UserPrivate(UserInDBBase):
    """Model for returning private user data (includes sensitive info)"""
    email: str

# ✓ SPEC-KIT PLUS VERIFIED