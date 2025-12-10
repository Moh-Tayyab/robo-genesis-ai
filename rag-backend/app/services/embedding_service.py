from typing import List
from app.llm import llm_client
import logging

logger = logging.getLogger(__name__)

class EmbeddingService:
    def __init__(self):
        self.llm = llm_client

    async def create_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Create embeddings for a list of texts
        """
        try:
            return await self.llm.create_embeddings(texts)
        except Exception as e:
            logger.error(f"Error creating embeddings: {e}")
            raise

    async def embed_text(self, text: str) -> List[float]:
        """
        Create embedding for a single text
        """
        try:
            return await self.llm.embed_text(text)
        except Exception as e:
            logger.error(f"Error embedding text: {e}")
            raise

embedding_service = EmbeddingService()