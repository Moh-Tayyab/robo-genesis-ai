from qdrant_client import QdrantClient
from qdrant_client.http import models
from app.core.config import settings
from typing import List, Dict, Any, Optional
import logging

logger = logging.getLogger(__name__)

class VectorDBClient:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=True
        )
        self.collection_name = "book-chunks"
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """
        Ensure the Qdrant collection exists with the correct schema
        """
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection {self.collection_name} already exists")
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=1536,  # text-embedding-3-small dimension
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created collection {self.collection_name}")

    def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        score_threshold: float = 0.75,
        filters: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar chunks in the vector database
        """
        # Prepare filters if provided
        qdrant_filters = None
        if filters:
            filter_conditions = []
            for key, value in filters.items():
                if isinstance(value, list):
                    filter_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchAny(any=value)
                        )
                    )
                else:
                    filter_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )

            if filter_conditions:
                qdrant_filters = models.Filter(
                    must=filter_conditions
                )

        # Perform search
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_vector,
            limit=top_k,
            score_threshold=score_threshold,
            query_filter=qdrant_filters
        )

        # Format results
        formatted_results = []
        for result in results:
            formatted_results.append({
                "chunk_id": result.payload.get("chunk_id"),
                "chapter": result.payload.get("chapter"),
                "section": result.payload.get("section"),
                "url": result.payload.get("url"),
                "difficulty": result.payload.get("difficulty"),
                "lang": result.payload.get("lang"),
                "text": result.payload.get("text"),
                "similarity": result.score
            })

        return formatted_results

    def insert_chunks(self, chunks: List[Dict[str, Any]]) -> bool:
        """
        Insert chunks into the vector database
        """
        points = []
        for chunk in chunks:
            point = models.PointStruct(
                id=chunk["chunk_id"],
                vector=chunk["vector"],
                payload={
                    "chunk_id": chunk["chunk_id"],
                    "chapter": chunk["chapter"],
                    "section": chunk["section"],
                    "url": chunk["url"],
                    "difficulty": chunk["difficulty"],
                    "lang": chunk["lang"],
                    "text": chunk["text"],
                    "text_hash": chunk["text_hash"]
                }
            )
            points.append(point)

        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            return True
        except Exception as e:
            logger.error(f"Error inserting chunks: {e}")
            return False

    def delete_chunks(self, chunk_ids: List[str]) -> bool:
        """
        Delete chunks from the vector database
        """
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=chunk_ids
                )
            )
            return True
        except Exception as e:
            logger.error(f"Error deleting chunks: {e}")
            return False

# Global instance
vector_db_client = VectorDBClient()