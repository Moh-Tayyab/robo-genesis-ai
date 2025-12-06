from typing import List, Dict, Any, Optional
from app.llm import llm_client
from app.vector_db import vector_db_client
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        self.llm = llm_client
        self.vector_db = vector_db_client

    async def query_full_book(self, question: str, user_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Query the full book for an answer to the question
        """
        try:
            # Create embedding for the question
            question_embedding = await self.llm.embed_text(question)

            # Search in vector database
            search_results = self.vector_db.search(
                query_vector=question_embedding,
                top_k=settings.qdrant_top_k,
                score_threshold=settings.qdrant_score_threshold
            )

            # Generate response using LLM
            response_text = await self.llm.generate_response(
                prompt=question,
                context_chunks=search_results
            )

            # Format response
            sources = [
                {
                    "chapter": chunk["chapter"],
                    "section": chunk["section"],
                    "url": chunk["url"],
                    "similarity": chunk["similarity"]
                }
                for chunk in search_results
            ]

            return {
                "response": response_text,
                "sources": sources,
                "retrievalMetadata": {
                    "chunksRetrieved": len(search_results),
                    "retrievalTimeMs": 0  # This would be calculated in a real implementation
                }
            }
        except Exception as e:
            logger.error(f"Error in query_full_book: {e}")
            raise

    async def query_selected_text(self, question: str, selected_text: str, user_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Query based on selected text only - with proper context isolation
        """
        try:
            # Validate selected text length
            if len(selected_text) > settings.max_selected_text_length:
                raise ValueError(f"Selected text too long: {len(selected_text)} characters. Maximum allowed: {settings.max_selected_text_length}")

            # Create embedding for the question
            question_embedding = await self.llm.embed_text(question)

            # Search in vector database for the most relevant chunks to the question
            search_results = self.vector_db.search(
                query_vector=question_embedding,
                top_k=settings.qdrant_top_k,
                score_threshold=settings.qdrant_score_threshold
            )

            # Implement context isolation: only return results that are highly relevant to the selected text
            # Use a more sophisticated approach to ensure context isolation
            filtered_results = []

            # Convert selected text to lowercase for comparison
            selected_text_lower = selected_text.lower()

            for result in search_results:
                result_text_lower = result["text"].lower()

                # Check if the result text contains significant overlap with the selected text
                # This can be done using keyword matching, semantic similarity, or both
                if self._is_relevant_to_selection(result_text_lower, selected_text_lower):
                    # Enhance the result with the selected text context for the LLM
                    enhanced_result = result.copy()
                    enhanced_result["text"] = f"Based on selected text: {selected_text}\n\nRelevant content: {result['text']}"
                    filtered_results.append(enhanced_result)

            # If no relevant results found in selected text, return "Not in selection"
            if not filtered_results:
                response_text = "The answer to your question is not contained in the selected text."
            else:
                # Generate response using LLM with the enhanced context
                # The LLM will be constrained to only use information from the selected text context
                response_text = await self.llm.generate_response(
                    prompt=f"Based only on the following selected text, answer this question: {question}",
                    context_chunks=filtered_results
                )

            # Format response
            sources = [
                {
                    "chapter": chunk["chapter"],
                    "section": chunk["section"],
                    "url": chunk["url"],
                    "similarity": chunk["similarity"]
                }
                for chunk in filtered_results
            ]

            return {
                "response": response_text,
                "sources": sources,
                "retrievalMetadata": {
                    "chunksRetrieved": len(filtered_results),
                    "retrievalTimeMs": 0  # This would be calculated in a real implementation
                }
            }
        except Exception as e:
            logger.error(f"Error in query_selected_text: {e}")
            raise

    def _is_relevant_to_selection(self, result_text: str, selected_text: str) -> bool:
        """
        Determine if a result is relevant to the selected text using keyword overlap and semantic analysis
        """
        # Simple keyword overlap approach
        result_words = set(result_text.split())
        selected_words = set(selected_text.split())

        # Calculate overlap ratio
        if len(selected_words) == 0:
            return False

        overlap = len(result_words.intersection(selected_words))
        overlap_ratio = overlap / len(selected_words)

        # If there's significant overlap or if the result text contains key phrases from selected text
        if overlap_ratio > 0.1:  # At least 10% overlap
            return True

        # Check for semantic similarity using a simple containment check
        # In a real implementation, we might use embedding similarity here
        if len(selected_text) > 20:  # Only for reasonably long selections
            # Check if significant portions of selected text appear in result
            for phrase_len in [5, 10, 15]:  # Check for phrases of different lengths
                if phrase_len <= len(selected_text):
                    # Get a sample phrase from the selected text
                    sample_phrase = ' '.join(selected_text.split()[:phrase_len])
                    if sample_phrase in result_text:
                        return True

        return False

rag_service = RAGService()