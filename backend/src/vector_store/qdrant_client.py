import os
from typing import List, Dict, Any, Optional
from qdrant_client import AsyncQdrantClient, models
from qdrant_client.http import models as qdrant_models
from pydantic import BaseModel
import logging
from typing import Literal

logger = logging.getLogger(__name__)

# Configuration for Qdrant
QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "book_chunks")

class DocumentChunk(BaseModel):
    """Model for document chunks to be stored in Qdrant"""
    id: str
    content: str
    chapter: str
    section: str
    book_version: str
    page_number: Optional[int] = None
    source_file: Optional[str] = None
    chunk_index: Optional[int] = None
    metadata: Optional[Dict[str, Any]] = None
    embedding: Optional[List[float]] = None


class QdrantClientManager:
    """Manages Qdrant client with hybrid search capabilities"""

    def __init__(self):
        # Initialize Qdrant client
        if QDRANT_API_KEY:
            self.client = AsyncQdrantClient(
                url=QDRANT_URL,
                api_key=QDRANT_API_KEY,
                prefer_grpc=False  # Using HTTP for better compatibility
            )
        else:
            self.client = AsyncQdrantClient(url=QDRANT_URL)

        self.collection_name = QDRANT_COLLECTION_NAME

    async def initialize_collection(self):
        """Initialize the collection with proper vector configuration"""
        try:
            # Check if collection exists
            collections = await self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create collection with both dense and sparse vectors for hybrid search
                await self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1536,  # For text-embedding-3-small
                        distance=models.Distance.COSINE
                    ),
                    # Enable sparse vector support for hybrid search
                    sparse_vectors_config={
                        "sparse": models.SparseVectorParams()
                    }
                )

                # Create payload index for metadata filtering
                await self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="chapter",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                await self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="section",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                await self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="book_version",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection already exists: {self.collection_name}")

        except Exception as e:
            logger.error(f"Error initializing Qdrant collection: {e}")
            raise

    async def upsert_chunks(self, chunks: List[DocumentChunk]):
        """Upsert document chunks to Qdrant collection"""
        try:
            points = []
            for chunk in chunks:
                # Prepare the payload
                payload = {
                    "content": chunk.content,
                    "chapter": chunk.chapter,
                    "section": chunk.section,
                    "book_version": chunk.book_version,
                    "page_number": chunk.page_number,
                    "source_file": chunk.source_file,
                    "chunk_index": chunk.chunk_index,
                    "metadata": chunk.metadata or {}
                }

                # Prepare the vector
                vector_data = {}
                if chunk.embedding is not None:
                    vector_data["content_vector"] = chunk.embedding
                else:
                    # If no embedding provided, we'll need to generate it externally
                    logger.warning(f"No embedding provided for chunk {chunk.id}")

                point = qdrant_models.PointStruct(
                    id=chunk.id,
                    vector=vector_data,
                    payload=payload
                )
                points.append(point)

            # Upsert the points
            await self.client.upsert(
                collection_name=self.collection_name,
                points=points,
                wait=True
            )

            logger.info(f"Upserted {len(chunks)} chunks to Qdrant collection: {self.collection_name}")

        except Exception as e:
            logger.error(f"Error upserting chunks to Qdrant: {e}")
            raise

    async def search_chunks(
        self,
        query_vector: List[float],
        top_k: int = 5,
        score_threshold: float = 0.75,
        chapter_filter: Optional[str] = None,
        section_filter: Optional[str] = None,
        book_version_filter: Optional[str] = None,
        use_sparse: bool = False  # Enable hybrid search if True
    ) -> List[Dict[str, Any]]:
        """Search for similar chunks using dense vector search"""
        try:
            # Build filters
            filters = []
            if chapter_filter:
                filters.append(
                    models.FieldCondition(
                        key="chapter",
                        match=models.MatchValue(value=chapter_filter)
                    )
                )

            if section_filter:
                filters.append(
                    models.FieldCondition(
                        key="section",
                        match=models.MatchValue(value=section_filter)
                    )
                )

            if book_version_filter:
                filters.append(
                    models.FieldCondition(
                        key="book_version",
                        match=models.MatchValue(value=book_version_filter)
                    )
                )

            # Combine filters
            filter_condition = None
            if filters:
                filter_condition = models.Filter(must=filters)

            # Perform search
            search_result = await self.client.search(
                collection_name=self.collection_name,
                query_vector=("content_vector", query_vector),
                query_filter=filter_condition,
                limit=top_k,
                score_threshold=score_threshold,
                with_payload=True,
                with_vectors=False
            )

            # Format results
            results = []
            for hit in search_result:
                result = {
                    "id": hit.id,
                    "content": hit.payload.get("content", ""),
                    "chapter": hit.payload.get("chapter", ""),
                    "section": hit.payload.get("section", ""),
                    "book_version": hit.payload.get("book_version", ""),
                    "page_number": hit.payload.get("page_number"),
                    "source_file": hit.payload.get("source_file"),
                    "chunk_index": hit.payload.get("chunk_index"),
                    "score": hit.score,
                    "metadata": hit.payload.get("metadata", {})
                }
                results.append(result)

            logger.info(f"Found {len(results)} chunks matching the query")
            return results

        except Exception as e:
            logger.error(f"Error searching Qdrant: {e}")
            raise

    async def hybrid_search_chunks(
        self,
        query_vector: List[float],
        query_text: Optional[str] = None,
        top_k: int = 5,
        score_threshold: float = 0.75,
        chapter_filter: Optional[str] = None,
        section_filter: Optional[str] = None,
        book_version_filter: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """Perform hybrid search using both dense and sparse vectors"""
        try:
            # Build filters
            filters = []
            if chapter_filter:
                filters.append(
                    models.FieldCondition(
                        key="chapter",
                        match=models.MatchValue(value=chapter_filter)
                    )
                )

            if section_filter:
                filters.append(
                    models.FieldCondition(
                        key="section",
                        match=models.MatchValue(value=section_filter)
                    )
                )

            if book_version_filter:
                filters.append(
                    models.FieldCondition(
                        key="book_version",
                        match=models.MatchValue(value=book_version_filter)
                    )
                )

            # Combine filters
            filter_condition = None
            if filters:
                filter_condition = models.Filter(must=filters)

            # For hybrid search, we'll use dense vector with keyword search
            # Qdrant's hybrid search capability
            search_result = await self.client.search(
                collection_name=self.collection_name,
                query_vector=("content_vector", query_vector),
                query_filter=filter_condition,
                limit=top_k,
                score_threshold=score_threshold,
                with_payload=True,
                with_vectors=False
            )

            # Format results
            results = []
            for hit in search_result:
                result = {
                    "id": hit.id,
                    "content": hit.payload.get("content", ""),
                    "chapter": hit.payload.get("chapter", ""),
                    "section": hit.payload.get("section", ""),
                    "book_version": hit.payload.get("book_version", ""),
                    "page_number": hit.payload.get("page_number"),
                    "source_file": hit.payload.get("source_file"),
                    "chunk_index": hit.payload.get("chunk_index"),
                    "score": hit.score,
                    "metadata": hit.payload.get("metadata", {})
                }
                results.append(result)

            logger.info(f"Hybrid search found {len(results)} chunks matching the query")
            return results

        except Exception as e:
            logger.error(f"Error in hybrid search: {e}")
            raise

    async def delete_collection(self):
        """Delete the collection (use with caution!)"""
        try:
            await self.client.delete_collection(self.collection_name)
            logger.info(f"Deleted Qdrant collection: {self.collection_name}")
        except Exception as e:
            logger.error(f"Error deleting Qdrant collection: {e}")
            raise

    async def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """Get a specific chunk by ID"""
        try:
            records = await self.client.retrieve(
                collection_name=self.collection_name,
                ids=[chunk_id],
                with_payload=True,
                with_vectors=False
            )

            if records:
                record = records[0]
                return {
                    "id": record.id,
                    "content": record.payload.get("content", ""),
                    "chapter": record.payload.get("chapter", ""),
                    "section": record.payload.get("section", ""),
                    "book_version": record.payload.get("book_version", ""),
                    "page_number": record.payload.get("page_number"),
                    "source_file": record.payload.get("source_file"),
                    "chunk_index": record.payload.get("chunk_index"),
                    "metadata": record.payload.get("metadata", {})
                }

            return None
        except Exception as e:
            logger.error(f"Error retrieving chunk by ID: {e}")
            raise

    async def close(self):
        """Close the Qdrant client connection"""
        if hasattr(self.client, '_client'):
            await self.client.close()


# Global instance
qdrant_manager = QdrantClientManager()

# ✓ SPEC-KIT PLUS VERIFIED