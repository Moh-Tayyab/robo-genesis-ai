"""Chat Service for managing conversations and integrating with RAG"""
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
import logging
from datetime import datetime
from sqlmodel import Session, select
from ..models.user import UserSQLModel
from ..models.session import SessionSQLModel
from ..models.message import MessageSQLModel, MessageRole
from ..models.retrieval import RetrievalSQLModel
from ..database import session_manager
from ..services.rag_service import rag_service, RAGResponse
from ..services.prompt_service import prompt_service
from ..services.vector_search_service import vector_search_service, SearchResult
from ..llm import openai_manager
from ..config import config_manager

logger = logging.getLogger(__name__)

class ChatMessage(BaseModel):
    """Model for chat messages"""
    role: MessageRole
    content: str
    timestamp: datetime


class ChatSession(BaseModel):
    """Model for chat sessions"""
    session_id: str
    user_id: Optional[str] = None
    created_at: datetime
    updated_at: datetime
    messages: List[ChatMessage]


class ChatService:
    """Service for managing conversations and integrating with RAG"""

    def __init__(self):
        self.rag_service = rag_service
        self.prompt_service = prompt_service
        self.vector_search_service = vector_search_service
        self.openai_manager = openai_manager
        self.config = config_manager.settings

    async def create_session(self, user_id: Optional[str] = None) -> str:
        """Create a new chat session"""
        try:
            session = SessionSQLModel(
                user_id=user_id,
                title="New Chat"  # Will be updated after first message
            )

            async with session_manager.session() as db_session:
                db_session.add(session)
                await db_session.commit()
                await db_session.refresh(session)

            logger.info(f"Created new session: {session.id}")
            return session.id

        except Exception as e:
            logger.error(f"Error creating session: {e}")
            raise

    async def get_session(self, session_id: str) -> Optional[ChatSession]:
        """Get a chat session with its messages"""
        try:
            async with session_manager.session() as db_session:
                # Get the session
                session = await db_session.get(SessionSQLModel, session_id)
                if not session:
                    return None

                # Get messages for the session
                statement = select(MessageSQLModel).where(
                    MessageSQLModel.session_id == session_id
                ).order_by(MessageSQLModel.created_at)
                result = await db_session.exec(statement)
                messages_db = result.all()

                # Convert to ChatMessage objects
                messages = [
                    ChatMessage(
                        role=message.role,
                        content=message.content,
                        timestamp=message.created_at
                    )
                    for message in messages_db
                ]

                # Create ChatSession object
                chat_session = ChatSession(
                    session_id=session.id,
                    user_id=session.user_id,
                    created_at=session.created_at,
                    updated_at=session.updated_at,
                    messages=messages
                )

                return chat_session

        except Exception as e:
            logger.error(f"Error getting session: {e}")
            raise

    async def add_message_to_session(self, session_id: str, role: MessageRole, content: str) -> str:
        """Add a message to a chat session"""
        try:
            message = MessageSQLModel(
                session_id=session_id,
                role=role,
                content=content
            )

            async with session_manager.session() as db_session:
                db_session.add(message)
                await db_session.commit()
                await db_session.refresh(message)

            logger.debug(f"Added message to session {session_id}: {role.value}")
            return message.id

        except Exception as e:
            logger.error(f"Error adding message to session: {e}")
            raise

    async def update_session_title(self, session_id: str, title: str):
        """Update the title of a session based on the first user message"""
        try:
            async with session_manager.session() as db_session:
                session = await db_session.get(SessionSQLModel, session_id)
                if session:
                    session.title = title
                    await db_session.commit()
                    logger.debug(f"Updated session title for {session_id}: {title}")

        except Exception as e:
            logger.error(f"Error updating session title: {e}")
            raise

    async def generate_session_title(self, user_message: str) -> str:
        """Generate a title for a chat session based on the user's first message"""
        try:
            # Create a prompt to generate a concise title
            system_prompt = (
                "Generate a concise, descriptive title (max 50 characters) for a chat session "
                "based on the user's question. The title should capture the main topic of the conversation. "
                "Use title case and avoid ending with punctuation."
            )

            user_prompt = f"Generate a title for this question: {user_message}"

            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ]

            title = await self.openai_manager.generate_response(
                messages=messages,
                temperature=0.3,
                max_tokens=20
            )

            # Ensure title is not too long
            if len(title) > 50:
                title = title[:50].strip()

            # Remove any trailing punctuation
            title = title.rstrip('.!?,:;')

            logger.debug(f"Generated session title: {title}")
            return title

        except Exception as e:
            logger.error(f"Error generating session title: {e}")
            # Fallback to first 50 characters of the message
            return user_message[:50].strip()

    async def process_full_book_query(self, session_id: str, query: str) -> RAGResponse:
        """Process a full-book RAG query and save to database"""
        try:
            # Add user message to session
            await self.add_message_to_session(session_id, MessageRole.user, query)

            # Process the RAG query
            rag_response = await self.rag_service.query_full_book(query)

            # Add assistant message to session
            await self.add_message_to_session(session_id, MessageRole.assistant, rag_response.answer)

            # Update session title if this is the first query
            session = await self.get_session(session_id)
            if session and len(session.messages) <= 2:  # Just user and assistant messages
                title = await self.generate_session_title(query)
                await self.update_session_title(session_id, title)

            # Save retrieval metadata to database
            await self._save_retrieval_metadata(session_id, query, rag_response)

            logger.info(f"Processed full-book query for session {session_id}")
            return rag_response

        except Exception as e:
            logger.error(f"Error processing full-book query: {e}")
            raise

    async def _save_retrieval_metadata(self, session_id: str, query: str, rag_response: RAGResponse):
        """Save retrieval metadata to the database"""
        try:
            # Get the latest assistant message ID for this session
            async with session_manager.session() as db_session:
                # Get the last message in the session (should be the assistant's response)
                statement = select(MessageSQLModel).where(
                    MessageSQLModel.session_id == session_id
                ).order_by(MessageSQLModel.created_at.desc())
                result = await db_session.exec(statement)
                last_message = result.first()

                if last_message and last_message.role == MessageRole.assistant:
                    # Create retrieval record
                    retrieval = RetrievalSQLModel(
                        message_id=last_message.id,
                        query=query,
                        context_used=rag_response.context_used,
                        retrieved_chunks=len(rag_response.retrieved_chunks),
                        sources=str(rag_response.sources),  # Store as JSON string
                        metadata={
                            "query_type": "full_book",
                            "processing_time": datetime.utcnow().isoformat()
                        }
                    )

                    db_session.add(retrieval)
                    await db_session.commit()

                    logger.debug(f"Saved retrieval metadata for message {last_message.id}")

        except Exception as e:
            logger.error(f"Error saving retrieval metadata: {e}")
            # This is non-critical, so we don't raise the exception

    async def get_conversation_history(self, session_id: str, limit: int = 50) -> List[Dict[str, Any]]:
        """Get conversation history for a session"""
        try:
            async with session_manager.session() as db_session:
                statement = select(MessageSQLModel).where(
                    MessageSQLModel.session_id == session_id
                ).order_by(MessageSQLModel.created_at.desc()).limit(limit)
                result = await db_session.exec(statement)
                messages_db = result.all()

                # Convert to dictionary format
                messages = [
                    {
                        "id": message.id,
                        "role": message.role.value,
                        "content": message.content,
                        "timestamp": message.created_at.isoformat()
                    }
                    for message in reversed(messages_db)  # Reverse to get chronological order
                ]

                logger.debug(f"Retrieved {len(messages)} messages for session {session_id}")
                return messages

        except Exception as e:
            logger.error(f"Error getting conversation history: {e}")
            raise

    async def get_session_stats(self, session_id: str) -> Dict[str, Any]:
        """Get statistics for a session"""
        try:
            async with session_manager.session() as db_session:
                # Get session
                session = await db_session.get(SessionSQLModel, session_id)
                if not session:
                    return {}

                # Count messages
                message_count_statement = select(MessageSQLModel).where(
                    MessageSQLModel.session_id == session_id
                )
                message_result = await db_session.exec(message_count_statement)
                messages = message_result.all()
                user_messages = sum(1 for msg in messages if msg.role == MessageRole.user)
                assistant_messages = sum(1 for msg in messages if msg.role == MessageRole.assistant)

                # Count retrievals
                retrieval_count_statement = select(RetrievalSQLModel).join(MessageSQLModel).where(
                    MessageSQLModel.session_id == session_id
                )
                retrieval_result = await db_session.exec(retrieval_count_statement)
                retrievals = retrieval_result.all()

                stats = {
                    "session_id": session.id,
                    "user_id": session.user_id,
                    "title": session.title,
                    "created_at": session.created_at.isoformat(),
                    "updated_at": session.updated_at.isoformat(),
                    "total_messages": len(messages),
                    "user_messages": user_messages,
                    "assistant_messages": assistant_messages,
                    "total_retrievals": len(retrievals),
                    "first_message_at": messages[0].created_at.isoformat() if messages else None,
                    "last_message_at": messages[-1].created_at.isoformat() if messages else None
                }

                return stats

        except Exception as e:
            logger.error(f"Error getting session stats: {e}")
            raise

    async def delete_session(self, session_id: str):
        """Delete a chat session and all associated data"""
        try:
            async with session_manager.session() as db_session:
                # Delete retrievals first (due to foreign key constraints)
                retrieval_statement = select(RetrievalSQLModel).join(MessageSQLModel).where(
                    MessageSQLModel.session_id == session_id
                )
                retrieval_result = await db_session.exec(retrieval_statement)
                retrievals = retrieval_result.all()

                for retrieval in retrievals:
                    await db_session.delete(retrieval)

                # Delete messages
                message_statement = select(MessageSQLModel).where(
                    MessageSQLModel.session_id == session_id
                )
                message_result = await db_session.exec(message_statement)
                messages = message_result.all()

                for message in messages:
                    await db_session.delete(message)

                # Delete session
                session = await db_session.get(SessionSQLModel, session_id)
                if session:
                    await db_session.delete(session)
                    await db_session.commit()

                logger.info(f"Deleted session {session_id} and all associated data")

        except Exception as e:
            logger.error(f"Error deleting session: {e}")
            raise


# Global instance
chat_service = ChatService()

# ✓ SPEC-KIT PLUS VERIFIED