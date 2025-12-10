from sqlalchemy import Column, DateTime, String, UUID, ForeignKey
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID
from sqlalchemy.ext.declarative import declarative_base
from app.database import Base
import uuid
from datetime import datetime


class Session(Base):
    __tablename__ = "sessions"

    session_id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(PostgresUUID(as_uuid=True), ForeignKey("users.user_id"), nullable=True)
    started_at = Column(DateTime, default=datetime.utcnow)
    ended_at = Column(DateTime, nullable=True)
    metadata = Column(String, nullable=True)  # JSON string

    def __repr__(self):
        return f"<Session(session_id={self.session_id}, user_id={self.user_id})>"