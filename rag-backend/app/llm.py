from openai import OpenAI
from app.core.config import settings
from typing import List, Dict, Any, Optional
import asyncio
import logging
from .services.agent_service import agent_rag_service

logger = logging.getLogger(__name__)

class LLMClient:
    def __init__(self):
        # For backward compatibility, we'll use the agent service
        self.model = settings.openai_model
        self.embedding_model = settings.embedding_model
        self.agent_service = agent_rag_service

    async def generate_response(self, prompt: str, context_chunks: List[Dict[str, Any]] = None) -> str:
        """
        Generate a response using the OpenAI Agent for full-book RAG
        """
        try:
            # Create a new thread for this conversation
            thread_id = self.agent_service.create_thread()

            # Add the user message to the thread
            self.agent_service.add_message_to_thread(thread_id, "user", prompt)

            # Run the assistant with instructions to use retrieve_knowledge tool
            if context_chunks:
                # If context chunks are provided, we can use them directly
                context_text = "\n\n".join([chunk["text"] for chunk in context_chunks])
                instructions = f"Answer questions based solely on the provided context with high precision. Context:\n{context_text}"
            else:
                instructions = "Use the retrieve_knowledge tool to search for relevant information from the textbook based on the user's question."

            response = self.agent_service.run_assistant(thread_id, instructions)
            return response
        except Exception as e:
            logger.error(f"Error generating response with agent: {e}")
            raise

    async def generate_selected_text_response(self, question: str, selected_text: str) -> str:
        """
        Generate a response using the OpenAI Agent for selected-text mode with strict context isolation
        """
        try:
            # Create a new thread for this conversation
            thread_id = self.agent_service.create_thread()

            # Add the user message to the thread
            user_message = f"Question: {question}\nSelected Text: {selected_text}"
            self.agent_service.add_message_to_thread(thread_id, "user", user_message)

            # Run the assistant with instructions to use selected text context
            instructions = f"Answer the question using only the selected text: {selected_text}"

            response = self.agent_service.run_assistant(thread_id, instructions)
            return response
        except Exception as e:
            logger.error(f"Error generating selected text response with agent: {e}")
            raise

    async def create_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Create embeddings for the given texts using the OpenAI API
        """
        try:
            client = OpenAI(api_key=settings.openai_api_key)
            response = client.embeddings.create(
                model=self.embedding_model,
                input=texts
            )
            return [item.embedding for item in response.data]
        except Exception as e:
            logger.error(f"Error creating embeddings: {e}")
            raise

    async def embed_text(self, text: str) -> List[float]:
        """
        Create embedding for a single text
        """
        embeddings = await self.create_embeddings([text])
        return embeddings[0] if embeddings else []

# Global instance
llm_client = LLMClient()