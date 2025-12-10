from sqlalchemy import Column, DateTime, String, UUID, Integer, ForeignKey
from sqlalchemy.dialects.postgresql import UUID as PostgresUUID
from sqlalchemy.ext.declarative import declarative_base
from app.database import Base
import uuid
from datetime import datetime


class Retrieval(Base):
    __tablename__ = "retrievals"

    retrieval_id = Column(PostgresUUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    msg_id = Column(PostgresUUID(as_uuid=True), ForeignKey("messages.msg_id"))
    chunk_id = Column(String)
    chapter = Column(String)
    section = Column(String)
    url = Column(String)
    latency_ms = Column(Integer)
    created_at = Column(DateTime, default=datetime.utcnow)

    def __repr__(self):
        return f"<Retrieval(retrieval_id={self.retrieval_id}, msg_id={self.msg_id}, chunk_id={self.chunk_id})>"