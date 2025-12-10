from typing import List, Dict, Any, Optional
from app.llm import llm_client
from app.vector_db import vector_db_client
from app.core.config import settings
from app.services.agent_service import agent_rag_service
import logging

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        self.llm = llm_client
        self.vector_db = vector_db_client
        self.agent_service = agent_rag_service

    async def query_full_book(self, question: str, user_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Query the full book for an answer to the question using OpenAI Agent
        """
        try:
            # Generate response using the agent-based approach
            response_text = await self.llm.generate_response(
                prompt=question,
                context_chunks=None  # Agent will handle RAG internally
            )

            # Since the agent handles RAG internally, we need to extract sources from the response
            # For now, we'll return a simplified response - in a real implementation,
            # we would parse the agent's tool call results to get sources
            sources = []  # Will be populated based on agent's tool calls

            return {
                "response": response_text,
                "sources": sources,
                "retrievalMetadata": {
                    "chunksRetrieved": 0,  # This will be updated based on actual retrieval
                    "retrievalTimeMs": 0
                }
            }
        except Exception as e:
            logger.error(f"Error in query_full_book: {e}")
            raise

    async def query_selected_text(self, question: str, selected_text: str, user_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Query based on selected text only using OpenAI Agent with strict context isolation
        """
        try:
            # Validate selected text length
            if len(selected_text) > settings.max_selected_text_length:
                raise ValueError(f"Selected text too long: {len(selected_text)} characters. Maximum allowed: {settings.max_selected_text_length}")

            # Generate response using the agent-based approach with selected text
            response_text = await self.llm.generate_selected_text_response(
                question=question,
                selected_text=selected_text
            )

            # Format response with selected text metadata
            sources = [
                {
                    "chapter": "Selected Text",
                    "section": "User Selection",
                    "url": "#selected-text",
                    "similarity": 1.0
                }
            ]

            return {
                "response": response_text,
                "sources": sources,
                "retrievalMetadata": {
                    "chunksRetrieved": 1,
                    "retrievalTimeMs": 0  # This would be calculated in a real implementation
                }
            }
        except Exception as e:
            logger.error(f"Error in query_selected_text: {e}")
            raise


rag_service = RAGService()