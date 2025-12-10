import os
from typing import List, Dict, Any, Optional, Union
from openai import AsyncOpenAI
from pydantic import BaseModel
import logging
import asyncio
import tiktoken

logger = logging.getLogger(__name__)

# Configuration for OpenAI
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
OPENAI_MODEL = os.getenv("OPENAI_MODEL", "gpt-4o-mini")
EMBEDDING_MODEL = os.getenv("EMBEDDING_MODEL", "text-embedding-3-small")

class OpenAIClientManager:
    """Manages OpenAI client for both chat and embedding operations"""

    def __init__(self):
        if not OPENAI_API_KEY:
            raise ValueError("OPENAI_API_KEY environment variable is required")

        self.client = AsyncOpenAI(api_key=OPENAI_API_KEY)
        self.model = OPENAI_MODEL
        self.embedding_model = EMBEDDING_MODEL
        self._tokenizer = None  # Lazy load tokenizer

    @property
    def tokenizer(self):
        """Lazy load the tokenizer"""
        if self._tokenizer is None:
            self._tokenizer = tiktoken.encoding_for_model(self.model)
        return self._tokenizer

    async def generate_response(
        self,
        messages: List[Dict[str, str]],
        temperature: float = 0.1,
        max_tokens: Optional[int] = 1000,
        top_p: float = 1.0,
        stop: Optional[List[str]] = None
    ) -> str:
        """Generate response from OpenAI chat model"""
        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=temperature,
                max_tokens=max_tokens,
                top_p=top_p,
                stop=stop
            )

            return response.choices[0].message.content

        except Exception as e:
            logger.error(f"Error generating response from OpenAI: {e}")
            raise

    async def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for text using OpenAI embedding model"""
        try:
            # Handle text that is too long by truncating
            if len(text) > 8192:  # OpenAI's limit for text-embedding-3-small
                text = text[:8192]
                logger.warning("Text truncated for embedding generation due to length limit")

            response = await self.client.embeddings.create(
                model=self.embedding_model,
                input=text
            )

            return response.data[0].embedding

        except Exception as e:
            logger.error(f"Error generating embedding from OpenAI: {e}")
            raise

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a batch of texts"""
        try:
            # Handle texts that are too long by truncating
            processed_texts = []
            for text in texts:
                if len(text) > 8192:  # OpenAI's limit for text-embedding-3-small
                    processed_texts.append(text[:8192])
                    logger.warning("Text truncated for embedding generation due to length limit")
                else:
                    processed_texts.append(text)

            response = await self.client.embeddings.create(
                model=self.embedding_model,
                input=processed_texts
            )

            return [item.embedding for item in response.data]

        except Exception as e:
            logger.error(f"Error generating batch embeddings from OpenAI: {e}")
            raise

    def count_tokens(self, text: str) -> int:
        """Count tokens in text using the appropriate tokenizer"""
        try:
            return len(self.tokenizer.encode(text))
        except Exception as e:
            logger.error(f"Error counting tokens: {e}")
            # Fallback: estimate using average token length
            return len(text) // 4  # Rough estimate

    async def generate_hypothetical_document(self, query: str) -> str:
        """Generate a hypothetical document for HyDE (Hypothetical Document Embeddings)"""
        try:
            messages = [
                {
                    "role": "system",
                    "content": "You are an expert at creating hypothetical answers to questions. Generate a detailed, accurate answer to the following question as if it were taken directly from a textbook. Your response should be in the same style as an academic textbook."
                },
                {
                    "role": "user",
                    "content": f"Generate a detailed answer to this question: {query}"
                }
            ]

            response = await self.generate_response(
                messages=messages,
                temperature=0.3,
                max_tokens=500
            )

            return response

        except Exception as e:
            logger.error(f"Error generating hypothetical document: {e}")
            # Return the original query as fallback
            return query

    async def check_content_relevance(self, content: str, query: str) -> bool:
        """Check if content is relevant to the query"""
        try:
            messages = [
                {
                    "role": "system",
                    "content": "You are a content relevance checker. Determine if the provided content is relevant to the given query. Respond with only 'YES' if relevant, or 'NO' if not relevant."
                },
                {
                    "role": "user",
                    "content": f"Content: {content}\\n\\nQuery: {query}\\n\\nIs the content relevant to the query?"
                }
            ]

            response = await self.generate_response(
                messages=messages,
                temperature=0.1,
                max_tokens=10
            )

            return response.strip().upper().startswith('YES')

        except Exception as e:
            logger.error(f"Error checking content relevance: {e}")
            # Default to true to be safe
            return True

    async def close(self):
        """Close the OpenAI client"""
        if hasattr(self.client, '_session'):
            await self.client.close()


# Global instance
openai_manager = OpenAIClientManager()

# ✓ SPEC-KIT PLUS VERIFIED