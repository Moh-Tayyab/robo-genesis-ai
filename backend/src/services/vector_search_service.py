"""Vector Search Service for managing vector database operations"""
from typing import List, Dict, Any, Optional
import logging
from pydantic import BaseModel
from ..vector_store.qdrant_client import qdrant_manager
from ..services.embedding_service import embedding_service
from ..config import config_manager

logger = logging.getLogger(__name__)

class SearchFilter(BaseModel):
    """Model for search filters"""
    chapter_filter: Optional[str] = None
    section_filter: Optional[str] = None
    book_version_filter: Optional[str] = None


class SearchResult(BaseModel):
    """Model for search results"""
    id: str
    content: str
    chapter: str
    section: str
    score: float
    page_number: Optional[int] = None
    source_file: Optional[str] = None
    chunk_index: Optional[int] = None
    metadata: Optional[Dict[str, Any]] = None


class VectorSearchService:
    """Service for performing vector searches in Qdrant with various search strategies"""

    def __init__(self):
        self.qdrant_manager = qdrant_manager
        self.embedding_service = embedding_service
        self.config = config_manager.settings

    async def search_by_text(
        self,
        query: str,
        top_k: Optional[int] = None,
        score_threshold: Optional[float] = None,
        filters: Optional[SearchFilter] = None,
        use_sparse: bool = False
    ) -> List[SearchResult]:
        """Search for similar chunks using dense vector search with optional filters"""
        try:
            # Generate embedding for the query
            query_vector = await self.embedding_service.generate_embedding(query)

            # Set defaults from config if not provided
            top_k = top_k or self.config.retrieval_top_k
            score_threshold = score_threshold or self.config.retrieval_score_threshold

            # Prepare filter parameters
            filter_kwargs = {}
            if filters:
                filter_kwargs = {
                    "chapter_filter": filters.chapter_filter,
                    "section_filter": filters.section_filter,
                    "book_version_filter": filters.book_version_filter,
                    "use_sparse": use_sparse
                }

            # Perform search in Qdrant
            results = await self.qdrant_manager.search_chunks(
                query_vector=query_vector,
                top_k=top_k,
                score_threshold=score_threshold,
                **filter_kwargs
            )

            # Convert results to SearchResult objects
            search_results = []
            for result in results:
                search_result = SearchResult(
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
                search_results.append(search_result)

            logger.info(f"Vector search completed: {len(search_results)} results for query '{query[:50]}...'")
            return search_results

        except Exception as e:
            logger.error(f"Error in vector search: {e}")
            raise

    async def search_by_hyde(
        self,
        query: str,
        top_k: Optional[int] = None,
        score_threshold: Optional[float] = None,
        filters: Optional[SearchFilter] = None
    ) -> List[SearchResult]:
        """Search using Hypothetical Document Embeddings approach"""
        try:
            # Generate embedding using HyDE approach
            query_vector = await self.embedding_service.generate_hypothetical_document_embedding(query)

            # Set defaults from config if not provided
            top_k = top_k or self.config.retrieval_top_k
            score_threshold = score_threshold or self.config.retrieval_score_threshold

            # Prepare filter parameters
            filter_kwargs = {}
            if filters:
                filter_kwargs = {
                    "chapter_filter": filters.chapter_filter,
                    "section_filter": filters.section_filter,
                    "book_version_filter": filters.book_version_filter
                }

            # Perform search in Qdrant with HyDE vector
            results = await self.qdrant_manager.search_chunks(
                query_vector=query_vector,
                top_k=top_k,
                score_threshold=score_threshold,
                **filter_kwargs
            )

            # Convert results to SearchResult objects
            search_results = []
            for result in results:
                search_result = SearchResult(
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
                search_results.append(search_result)

            logger.info(f"HyDE search completed: {len(search_results)} results for query '{query[:50]}...'")
            return search_results

        except Exception as e:
            logger.error(f"Error in HyDE search: {e}")
            raise

    async def hybrid_search(
        self,
        query: str,
        top_k: Optional[int] = None,
        score_threshold: Optional[float] = None,
        filters: Optional[SearchFilter] = None
    ) -> List[SearchResult]:
        """Perform hybrid search using both dense and sparse vectors"""
        try:
            # Generate embedding for the query
            query_vector = await self.embedding_service.generate_embedding(query)

            # Set defaults from config if not provided
            top_k = top_k or self.config.retrieval_top_k
            score_threshold = score_threshold or self.config.retrieval_score_threshold

            # Prepare filter parameters
            filter_kwargs = {}
            if filters:
                filter_kwargs = {
                    "chapter_filter": filters.chapter_filter,
                    "section_filter": filters.section_filter,
                    "book_version_filter": filters.book_version_filter
                }

            # Perform hybrid search in Qdrant
            results = await self.qdrant_manager.hybrid_search_chunks(
                query_vector=query_vector,
                query_text=query,
                top_k=top_k,
                score_threshold=score_threshold,
                **filter_kwargs
            )

            # Convert results to SearchResult objects
            search_results = []
            for result in results:
                search_result = SearchResult(
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
                search_results.append(search_result)

            logger.info(f"Hybrid search completed: {len(search_results)} results for query '{query[:50]}...'")
            return search_results

        except Exception as e:
            logger.error(f"Error in hybrid search: {e}")
            raise

    async def search_with_mmr(
        self,
        query: str,
        top_k: int = 5,
        mmr_lambda: float = 0.7,
        filters: Optional[SearchFilter] = None
    ) -> List[SearchResult]:
        """Search and apply Maximum Marginal Relevance to diversify results"""
        try:
            # First, get more results than needed for MMR processing
            initial_results = await self.search_by_hyde(
                query=query,
                top_k=top_k * 2,  # Get more results for MMR selection
                filters=filters
            )

            if len(initial_results) <= top_k:
                return initial_results

            # Calculate embeddings for all results
            result_texts = [result.content for result in initial_results]
            result_embeddings = await self.embedding_service.generate_embeddings_batch(result_texts)

            # Calculate query embedding
            query_embedding = await self.embedding_service.generate_embedding(query)

            # Apply MMR algorithm
            selected_indices = []
            remaining_indices = list(range(len(initial_results)))

            # Calculate similarity between query and results
            query_similarities = [
                self.embedding_service.cosine_similarity(query_embedding, result_emb)
                for result_emb in result_embeddings
            ]

            # Calculate similarities between results (for diversity)
            result_similarities = []
            for i, result_emb1 in enumerate(result_embeddings):
                row = []
                for j, result_emb2 in enumerate(result_embeddings):
                    if i == j:
                        row.append(1.0)  # Same result similarity is 1
                    else:
                        row.append(self.embedding_service.cosine_similarity(result_emb1, result_emb2))
                result_similarities.append(row)

            # Apply MMR algorithm
            for _ in range(min(top_k, len(initial_results))):
                if not remaining_indices:
                    break

                # Calculate MMR scores for remaining results
                mmr_scores = []
                for idx in remaining_indices:
                    # Similarity to query (relevance)
                    relevance = query_similarities[idx]

                    # Maximum similarity to already selected results (diversity penalty)
                    if selected_indices:
                        max_similarity = max(
                            result_similarities[idx][selected_idx]
                            for selected_idx in selected_indices
                        )
                    else:
                        max_similarity = 0

                    # MMR score: λ * relevance - (1-λ) * max_similarity
                    mmr_score = mmr_lambda * relevance - (1 - mmr_lambda) * max_similarity
                    mmr_scores.append((mmr_score, idx))

                # Select result with highest MMR score
                best_score, best_idx = max(mmr_scores, key=lambda x: x[0])
                selected_indices.append(best_idx)
                remaining_indices.remove(best_idx)

            # Return selected results in MMR order
            mmr_results = [initial_results[i] for i in selected_indices]

            logger.info(f"MMR search completed: {len(mmr_results)} diverse results for query '{query[:50]}...'")
            return mmr_results

        except Exception as e:
            logger.error(f"Error in MMR search: {e}")
            raise

    async def get_chunk_by_id(self, chunk_id: str) -> Optional[SearchResult]:
        """Retrieve a specific chunk by ID"""
        try:
            result = await self.qdrant_manager.get_chunk_by_id(chunk_id)

            if result:
                search_result = SearchResult(
                    id=result["id"],
                    content=result["content"],
                    chapter=result["chapter"],
                    section=result["section"],
                    score=1.0,  # Not a search result, so score is 1.0
                    page_number=result["page_number"],
                    source_file=result["source_file"],
                    chunk_index=result["chunk_index"],
                    metadata=result["metadata"]
                )
                return search_result

            return None

        except Exception as e:
            logger.error(f"Error retrieving chunk by ID: {e}")
            raise


# Global instance
vector_search_service = VectorSearchService()

# ✓ SPEC-KIT PLUS VERIFIED