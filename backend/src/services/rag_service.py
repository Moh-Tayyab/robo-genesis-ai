"""RAG Service for full-book queries with HyDE and MMR implementation"""
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from ..llm import openai_manager
from ..vector_store.qdrant_client import qdrant_manager
from ..config import config_manager
from sqlmodel import Session
import logging
import uuid
from datetime import datetime

logger = logging.getLogger(__name__)

class RetrievalChunk(BaseModel):
    """Model for retrieved chunks with metadata"""
    id: str
    content: str
    chapter: str
    section: str
    score: float
    page_number: Optional[int] = None
    source_file: Optional[str] = None
    chunk_index: Optional[int] = None
    metadata: Optional[Dict[str, Any]] = None


class RAGResponse(BaseModel):
    """Model for RAG service response"""
    answer: str
    retrieved_chunks: List[RetrievalChunk]
    query: str
    context_used: str
    sources: List[Dict[str, Any]]


class RAGService:
    """RAG Service for full-book queries with HyDE and MMR implementation"""

    def __init__(self):
        self.openai_manager = openai_manager
        self.qdrant_manager = qdrant_manager
        self.config = config_manager.settings
        self.top_k = self.config.retrieval_top_k
        self.score_threshold = self.config.retrieval_score_threshold

    async def generate_hypothetical_answer(self, query: str) -> str:
        """Generate hypothetical document for HyDE approach"""
        return await self.openai_manager.generate_hypothetical_document(query)

    async def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for text using OpenAI"""
        return await self.openai_manager.generate_embedding(text)

    async def retrieve_chunks(self, query: str, top_k: Optional[int] = None) -> List[RetrievalChunk]:
        """Retrieve relevant chunks using vector search with HyDE"""
        try:
            # Use HyDE: Generate hypothetical answer first
            hypothetical_answer = await self.generate_hypothetical_answer(query)

            # Generate embedding for the hypothetical answer (HyDE approach)
            query_embedding = await self.generate_embedding(hypothetical_answer)

            # Search in Qdrant with the hypothetical embedding
            search_results = await self.qdrant_manager.search_chunks(
                query_vector=query_embedding,
                top_k=top_k or self.top_k,
                score_threshold=self.score_threshold
            )

            # Convert search results to RetrievalChunk objects
            chunks = []
            for result in search_results:
                chunk = RetrievalChunk(
                    id=result["id"],
                    content=result["content"],
                    chapter=result["chapter"],
                    section=result["section"],
                    score=result["score"],
                    page_number=result["page_number"],
                    source_file=result["source_file"],
                    chunk_index=result["chunk_index"],
                    metadata=result["metadata"]
                )
                chunks.append(chunk)

            logger.info(f"Retrieved {len(chunks)} chunks for query: {query[:100]}...")
            return chunks

        except Exception as e:
            logger.error(f"Error retrieving chunks: {e}")
            raise

    async def apply_mmr(self, query: str, chunks: List[RetrievalChunk], top_k: int = 8, diversity: float = 0.7) -> List[RetrievalChunk]:
        """Apply Maximum Marginal Relevance to diversify results"""
        if len(chunks) <= top_k:
            return chunks

        # Calculate embeddings for all chunks
        chunk_texts = [chunk.content for chunk in chunks]
        chunk_embeddings = await self.openai_manager.generate_embeddings_batch(chunk_texts)

        # Calculate query embedding
        query_embedding = await self.generate_embedding(query)

        # Calculate MMR scores
        selected_indices = []
        remaining_indices = list(range(len(chunks)))

        # Calculate similarity between query and chunks
        query_similarities = [
            self._cosine_similarity(query_embedding, chunk_emb)
            for chunk_emb in chunk_embeddings
        ]

        # Calculate similarities between chunks (for diversity)
        chunk_similarities = []
        for i, chunk_emb1 in enumerate(chunk_embeddings):
            row = []
            for j, chunk_emb2 in enumerate(chunk_embeddings):
                if i == j:
                    row.append(1.0)  # Same chunk similarity is 1
                else:
                    row.append(self._cosine_similarity(chunk_emb1, chunk_emb2))
            chunk_similarities.append(row)

        # Apply MMR algorithm
        for _ in range(min(top_k, len(chunks))):
            if not remaining_indices:
                break

            # Calculate MMR scores for remaining chunks
            mmr_scores = []
            for idx in remaining_indices:
                # Similarity to query (relevance)
                relevance = query_similarities[idx]

                # Maximum similarity to already selected chunks (diversity penalty)
                if selected_indices:
                    max_similarity = max(
                        chunk_similarities[idx][selected_idx]
                        for selected_idx in selected_indices
                    )
                else:
                    max_similarity = 0

                # MMR score: λ * relevance - (1-λ) * max_similarity
                mmr_score = diversity * relevance - (1 - diversity) * max_similarity
                mmr_scores.append((mmr_score, idx))

            # Select chunk with highest MMR score
            best_score, best_idx = max(mmr_scores, key=lambda x: x[0])
            selected_indices.append(best_idx)
            remaining_indices.remove(best_idx)

        # Return selected chunks in order
        return [chunks[i] for i in selected_indices]

    def _cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """Calculate cosine similarity between two vectors"""
        dot_product = sum(a * b for a, b in zip(vec1, vec2))
        magnitude1 = sum(a * a for a in vec1) ** 0.5
        magnitude2 = sum(b * b for b in vec2) ** 0.5

        if magnitude1 == 0 or magnitude2 == 0:
            return 0.0

        return dot_product / (magnitude1 * magnitude2)

    async def build_context(self, chunks: List[RetrievalChunk]) -> str:
        """Build context string from retrieved chunks"""
        context_parts = []
        for i, chunk in enumerate(chunks):
            context_parts.append(
                f"Source {i+1} (Chapter: {chunk.chapter}, Section: {chunk.section}):\n"
                f"{chunk.content}\n"
            )

        return "\n".join(context_parts)

    async def generate_answer(self, query: str, context: str) -> str:
        """Generate answer using OpenAI based on context"""
        try:
            # Build the prompt with context
            system_prompt = (
                "You are an expert assistant for a Physical AI and Humanoid Robotics textbook. "
                "Answer the user's question based on the provided context from the textbook. "
                "If the context doesn't contain enough information to answer the question, "
                "say 'I cannot answer this question based on the provided textbook content.' "
                "Always cite specific sources from the context when providing answers. "
                "Be accurate, concise, and helpful."
            )

            user_message = f"Context:\n{context}\n\nQuestion: {query}"

            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_message}
            ]

            answer = await self.openai_manager.generate_response(
                messages=messages,
                temperature=0.1,
                max_tokens=1000
            )

            return answer

        except Exception as e:
            logger.error(f"Error generating answer: {e}")
            raise

    async def query_full_book(self, query: str) -> RAGResponse:
        """Main method to query the full book with RAG"""
        try:
            logger.info(f"Processing full-book query: {query[:100]}...")

            # Step 1: Retrieve relevant chunks using HyDE
            initial_chunks = await self.retrieve_chunks(query)

            # Step 2: Apply MMR to diversify results (get top 8 most relevant and diverse)
            mmr_chunks = await self.apply_mmr(query, initial_chunks, top_k=8)

            # Step 3: Build context from selected chunks
            context = await self.build_context(mmr_chunks)

            # Step 4: Generate answer based on context
            answer = await self.generate_answer(query, context)

            # Step 5: Create response object
            response = RAGResponse(
                answer=answer,
                retrieved_chunks=mmr_chunks,
                query=query,
                context_used=context,
                sources=[
                    {
                        "id": chunk.id,
                        "chapter": chunk.chapter,
                        "section": chunk.section,
                        "page_number": chunk.page_number,
                        "source_file": chunk.source_file
                    }
                    for chunk in mmr_chunks
                ]
            )

            logger.info(f"Full-book query completed successfully, retrieved {len(mmr_chunks)} chunks")
            return response

        except Exception as e:
            logger.error(f"Error in full-book query: {e}")
            raise

    async def validate_content_relevance(self, content: str, query: str) -> bool:
        """Validate if content is relevant to the query using LLM"""
        return await self.openai_manager.check_content_relevance(content, query)


# Global instance
rag_service = RAGService()

# ✓ SPEC-KIT PLUS VERIFIED