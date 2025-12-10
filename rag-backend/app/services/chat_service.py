from typing import Dict, Any, Optional, List
from sqlalchemy.ext.asyncio import AsyncSession
from app.models.message import Message, MessageRole
from app.models.session import Session
from app.models.user import User
from app.models.retrieval import Retrieval
from app.database import AsyncSessionLocal
from app.services.rag_service import rag_service
from app.services.prompt_service import prompt_service
from app.core.config import settings
import uuid
import logging
from datetime import datetime

logger = logging.getLogger(__name__)

class ChatService:
    def __init__(self):
        self.rag_service = rag_service
        self.prompt_service = prompt_service

    async def create_session(self, user_id: Optional[str] = None) -> str:
        """
        Create a new chat session
        """
        async with AsyncSessionLocal() as db:
            try:
                session = Session(
                    user_id=uuid.UUID(user_id) if user_id else None
                )
                db.add(session)
                await db.commit()
                await db.refresh(session)
                return str(session.session_id)
            except Exception as e:
                await db.rollback()
                logger.error(f"Error creating session: {e}")
                raise

    async def add_message_to_session(self, session_id: str, role: str, text: str) -> str:
        """
        Add a message to a session
        """
        async with AsyncSessionLocal() as db:
            try:
                message = Message(
                    session_id=uuid.UUID(session_id),
                    role=MessageRole(role),
                    text=text
                )
                db.add(message)
                await db.commit()
                await db.refresh(message)
                return str(message.msg_id)
            except Exception as e:
                await db.rollback()
                logger.error(f"Error adding message to session: {e}")
                raise

    async def get_session_history(self, session_id: str) -> List[Dict[str, Any]]:
        """
        Get the message history for a session
        """
        async with AsyncSessionLocal() as db:
            try:
                result = await db.execute(
                    Message.__table__.select().where(
                        Message.session_id == uuid.UUID(session_id)
                    ).order_by(Message.created_at)
                )
                messages = result.fetchall()
                return [
                    {
                        "msg_id": str(msg.msg_id),
                        "role": msg.role,
                        "text": msg.text,
                        "created_at": msg.created_at.isoformat()
                    }
                    for msg in messages
                ]
            except Exception as e:
                logger.error(f"Error getting session history: {e}")
                raise

    async def process_full_book_query(self, question: str, session_id: Optional[str] = None, user_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Process a full-book RAG query
        """
        try:
            # Create session if not provided
            if not session_id:
                session_id = await self.create_session(user_id)

            # Add user message to session
            await self.add_message_to_session(session_id, "user", question)

            # Process with RAG service
            rag_result = await self.rag_service.query_full_book(question, user_id)

            # Add assistant response to session
            await self.add_message_to_session(session_id, "assistant", rag_result["response"])

            # Return result with session ID
            rag_result["sessionId"] = session_id
            return rag_result
        except Exception as e:
            logger.error(f"Error processing full book query: {e}")
            raise

    async def process_selected_text_query(self, question: str, selected_text: str, session_id: Optional[str] = None, user_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Process a selected-text RAG query
        """
        try:
            # Validate selected text length
            if len(selected_text) > settings.max_selected_text_length:
                raise ValueError(f"Selected text too long: {len(selected_text)} characters. Maximum allowed: {settings.max_selected_text_length}")

            # Create session if not provided
            if not session_id:
                session_id = await self.create_session(user_id)

            # Add user message to session
            await self.add_message_to_session(session_id, "user", f"Question: {question}\nSelected Text: {selected_text}")

            # Process with RAG service
            rag_result = await self.rag_service.query_selected_text(question, selected_text, user_id)

            # Add assistant response to session
            await self.add_message_to_session(session_id, "assistant", rag_result["response"])

            # Return result with session ID
            rag_result["sessionId"] = session_id
            return rag_result
        except Exception as e:
            logger.error(f"Error processing selected text query: {e}")
            raise

chat_service = ChatService()