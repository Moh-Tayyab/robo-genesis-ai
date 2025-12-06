from sqlalchemy import Column, DateTime, String, UUID, Text, Integer, Boolean
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID
from sqlalchemy.ext.declarative import declarative_base
from app.database import Base
import uuid
from datetime import datetime


class User(Base):
    __tablename__ = "users"

    user_id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    email = Column(String, nullable=True)

    # Background information collected during sign-up
    programming_experience = Column(String, nullable=True)  # "beginner", "intermediate", "advanced", "expert"
    os_preference = Column(String, nullable=True)  # "windows", "macos", "linux"
    gpu_available = Column(Boolean, nullable=True)  # Whether user has GPU
    preferred_language = Column(String, nullable=True)  # "python", "typescript", "cpp", etc.
    learning_goals = Column(Text, nullable=True)  # Text field for learning goals
    hardware_background = Column(String, nullable=True)  # "none", "maker", "engineer", "researcher"
    software_background = Column(String, nullable=True)  # "none", "developer", "engineer", "researcher"
    profile_completed = Column(Boolean, default=False)  # Whether user completed background questions

    # Additional preferences
    preferences = Column(String, nullable=True)  # JSON string for additional preferences

    def __repr__(self):
        return f"<User(user_id={self.user_id}, email={self.email}, programming_experience={self.programming_experience})>"