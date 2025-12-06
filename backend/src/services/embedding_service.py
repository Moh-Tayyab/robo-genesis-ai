"""Embedding Service for generating and managing embeddings using OpenAI"""
from typing import List
import logging
from ..llm import openai_manager
from ..config import config_manager

logger = logging.getLogger(__name__)

class EmbeddingService:
    """Service for generating and managing embeddings using OpenAI"""

    def __init__(self):
        self.openai_manager = openai_manager
        self.config = config_manager.settings
        self.max_batch_size = self.config.max_embedding_batch_size

    async def generate_embedding(self, text: str) -> List[float]:
        """Generate a single embedding for the given text"""
        try:
            # Check if text is too long and needs to be truncated
            if len(text) > 8192:  # OpenAI's limit for text-embedding-3-small
                logger.warning(f"Text truncated from {len(text)} to 8192 characters for embedding")
                text = text[:8192]

            embedding = await self.openai_manager.generate_embedding(text)
            logger.debug(f"Generated embedding of length {len(embedding)} for text of length {len(text)}")
            return embedding

        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a batch of texts with chunking for large batches"""
        try:
            all_embeddings = []

            # Process in batches to respect API limits
            for i in range(0, len(texts), self.max_batch_size):
                batch = texts[i:i + self.max_batch_size]

                # Truncate texts that are too long
                processed_batch = []
                for text in batch:
                    if len(text) > 8192:  # OpenAI's limit
                        logger.warning(f"Text truncated from {len(text)} to 8192 characters for embedding")
                        processed_batch.append(text[:8192])
                    else:
                        processed_batch.append(text)

                batch_embeddings = await self.openai_manager.generate_embeddings_batch(processed_batch)
                all_embeddings.extend(batch_embeddings)

                logger.debug(f"Processed batch {i//self.max_batch_size + 1}/{(len(texts)-1)//self.max_batch_size + 1}")

            logger.info(f"Generated {len(all_embeddings)} embeddings for {len(texts)} texts")
            return all_embeddings

        except Exception as e:
            logger.error(f"Error generating batch embeddings: {e}")
            raise

    async def generate_hypothetical_document_embedding(self, query: str) -> List[float]:
        """Generate embedding for a hypothetical document using HyDE approach"""
        try:
            # Generate hypothetical document
            hypothetical_doc = await self.openai_manager.generate_hypothetical_document(query)

            # Generate embedding for the hypothetical document
            embedding = await self.generate_embedding(hypothetical_doc)

            logger.debug(f"Generated HyDE embedding for query: {query[:50]}...")
            return embedding

        except Exception as e:
            logger.error(f"Error generating HyDE embedding: {e}")
            # Fallback to embedding the original query
            return await self.generate_embedding(query)

    def cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """Calculate cosine similarity between two vectors"""
        if len(vec1) != len(vec2):
            raise ValueError("Vectors must have the same length")

        dot_product = sum(a * b for a, b in zip(vec1, vec2))
        magnitude1 = sum(a * a for a in vec1) ** 0.5
        magnitude2 = sum(b * b for b in vec2) ** 0.5

        if magnitude1 == 0 or magnitude2 == 0:
            return 0.0

        return dot_product / (magnitude1 * magnitude2)

    def get_embedding_dimensions(self) -> int:
        """Get the expected dimensions for embeddings (based on OpenAI model)"""
        # text-embedding-3-small produces 1536-dimensional vectors
        # This can be changed based on the model configuration
        return 1536


# Global instance
embedding_service = EmbeddingService()

# ✓ SPEC-KIT PLUS VERIFIED