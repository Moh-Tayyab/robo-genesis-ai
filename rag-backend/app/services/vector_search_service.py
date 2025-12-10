from typing import List, Dict, Any, Optional
from app.vector_db import vector_db_client
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)

class VectorSearchService:
    def __init__(self):
        self.vector_db = vector_db_client

    def search(
        self,
        query_vector: List[float],
        top_k: Optional[int] = None,
        score_threshold: Optional[float] = None,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar chunks in the vector database
        """
        try:
            # Use default values if not provided
            if top_k is None:
                top_k = settings.qdrant_top_k
            if score_threshold is None:
                score_threshold = settings.qdrant_score_threshold

            return self.vector_db.search(
                query_vector=query_vector,
                top_k=top_k,
                score_threshold=score_threshold,
                filters=filters
            )
        except Exception as e:
            logger.error(f"Error in vector search: {e}")
            raise

    def insert_chunks(self, chunks: List[Dict[str, Any]]) -> bool:
        """
        Insert chunks into the vector database
        """
        try:
            return self.vector_db.insert_chunks(chunks)
        except Exception as e:
            logger.error(f"Error inserting chunks: {e}")
            return False

    def delete_chunks(self, chunk_ids: List[str]) -> bool:
        """
        Delete chunks from the vector database
        """
        try:
            return self.vector_db.delete_chunks(chunk_ids)
        except Exception as e:
            logger.error(f"Error deleting chunks: {e}")
            return False

vector_search_service = VectorSearchService()