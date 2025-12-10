from sqlalchemy import Column, DateTime, String, UUID, ForeignKey, Enum
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID
from sqlalchemy.ext.declarative import declarative_base
from app.database import Base
import uuid
from datetime import datetime
from enum import Enum as PyEnum


class MessageRole(str, PyEnum):
    user = "user"
    assistant = "assistant"


class Message(Base):
    __tablename__ = "messages"

    msg_id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(PostgresUUID(as_uuid=True), ForeignKey("sessions.session_id"))
    role = Column(Enum(MessageRole))
    text = Column(String)
    created_at = Column(DateTime, default=datetime.utcnow)
    metadata = Column(String, nullable=True)  # JSON string

    def __repr__(self):
        return f"<Message(msg_id={self.msg_id}, role={self.role}, session_id={self.session_id})>"