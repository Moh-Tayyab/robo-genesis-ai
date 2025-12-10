import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from app.services.rag_service import RAGService
import asyncio

@pytest.fixture
def rag_service():
    """Create a RAGService instance for testing"""
    service = RAGService()
    # Mock the dependencies
    service.llm = AsyncMock()
    service.vector_db = MagicMock()
    return service

@pytest.mark.asyncio
async def test_query_full_book(rag_service):
    """Test the query_full_book method"""
    # Mock data
    question = "What is humanoid robotics?"
    embedding = [0.1, 0.2, 0.3]
    search_results = [
        {
            "chunk_id": "chunk1",
            "chapter": "Chapter 1",
            "section": "Section 1.1",
            "url": "/docs/chapter1",
            "difficulty": "beginner",
            "lang": "en",
            "text": "Humanoid robotics is a field of robotics that focuses on creating robots with human-like characteristics.",
            "similarity": 0.95
        }
    ]
    llm_response = "Humanoid robotics is a field of robotics that focuses on creating robots with human-like characteristics."

    # Setup mocks
    rag_service.llm.embed_text.return_value = embedding
    rag_service.vector_db.search.return_value = search_results
    rag_service.llm.generate_response.return_value = llm_response

    # Call the method
    result = await rag_service.query_full_book(question)

    # Assertions
    assert result["response"] == llm_response
    assert len(result["sources"]) == 1
    assert result["sources"][0]["chapter"] == "Chapter 1"
    assert result["sources"][0]["similarity"] == 0.95
    rag_service.llm.embed_text.assert_called_once_with(question)
    rag_service.vector_db.search.assert_called_once()

@pytest.mark.asyncio
async def test_query_selected_text(rag_service):
    """Test the query_selected_text method"""
    # Mock data
    question = "What is bipedal locomotion?"
    selected_text = "Bipedal locomotion refers to walking on two legs."
    embedding = [0.1, 0.2, 0.3]
    search_results = [
        {
            "chunk_id": "chunk1",
            "chapter": "Chapter 2",
            "section": "Section 2.1",
            "url": "/docs/chapter2",
            "difficulty": "intermediate",
            "lang": "en",
            "text": "Bipedal locomotion refers to walking on two legs.",
            "similarity": 0.98
        }
    ]
    llm_response = "Bipedal locomotion refers to walking on two legs."

    # Setup mocks
    rag_service.llm.embed_text.return_value = embedding
    rag_service.vector_db.search.return_value = search_results
    rag_service.llm.generate_response.return_value = llm_response

    # Call the method
    result = await rag_service.query_selected_text(question, selected_text)

    # Assertions
    assert result["response"] == llm_response
    assert len(result["sources"]) == 1
    assert result["sources"][0]["chapter"] == "Chapter 2"
    rag_service.llm.embed_text.assert_called()

@pytest.mark.asyncio
async def test_query_selected_text_too_long(rag_service):
    """Test that query_selected_text raises an error for text that's too long"""
    from app.core.config import settings
    long_text = "x" * (settings.max_selected_text_length + 1)

    with pytest.raises(ValueError):
        await rag_service.query_selected_text("question", long_text)

def test_rag_service_initialization():
    """Test that RAGService initializes properly"""
    service = RAGService()
    assert service is not None
    assert hasattr(service, 'llm')
    assert hasattr(service, 'vector_db')