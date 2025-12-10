import pytest
import asyncio
from fastapi.testclient import TestClient
from app.main import app
from app.services.chat_service import chat_service
from unittest.mock import patch, AsyncMock

# Create test client
client = TestClient(app)

@pytest.mark.asyncio
async def test_full_book_qa_integration():
    """
    Integration test for full-book Q&A flow
    """
    # Mock the external services to avoid actual API calls
    with patch('app.services.rag_service.rag_service.query_full_book', new_callable=AsyncMock) as mock_query:
        # Mock response
        mock_response = {
            "response": "This is a test answer from the textbook.",
            "sources": [
                {
                    "chapter": "Chapter 1",
                    "section": "Section 1.1",
                    "url": "/docs/chapter1",
                    "similarity": 0.95
                }
            ],
            "retrievalMetadata": {
                "chunksRetrieved": 1,
                "retrievalTimeMs": 100
            },
            "sessionId": "test-session-id"
        }
        mock_query.return_value = mock_response

        # Make request
        response = client.post(
            "/api/chat",
            json={
                "question": "What is humanoid robotics?"
            }
        )

        # Assertions
        assert response.status_code == 200
        data = response.json()
        assert "response" in data
        assert "sources" in data
        assert "sessionId" in data
        assert data["response"] == "This is a test answer from the textbook."
        assert len(data["sources"]) == 1
        assert data["sources"][0]["chapter"] == "Chapter 1"

@pytest.mark.asyncio
async def test_selected_text_qa_integration():
    """
    Integration test for selected-text Q&A flow
    """
    # Mock the external services to avoid actual API calls
    with patch('app.services.rag_service.rag_service.query_selected_text', new_callable=AsyncMock) as mock_query:
        # Mock response
        mock_response = {
            "response": "Not in selection.",
            "sources": [],
            "retrievalMetadata": {
                "chunksRetrieved": 0,
                "retrievalTimeMs": 50
            },
            "sessionId": "test-session-id"
        }
        mock_query.return_value = mock_response

        # Make request
        response = client.post(
            "/api/chat/selected",
            json={
                "question": "What is humanoid robotics?",
                "selectedText": "This is some selected text."
            }
        )

        # Assertions
        assert response.status_code == 200
        data = response.json()
        assert "response" in data
        assert "sources" in data
        assert "sessionId" in data
        assert data["response"] == "Not in selection."
        assert len(data["sources"]) == 0

def test_health_endpoint():
    """
    Test health endpoint
    """
    response = client.get("/api/health")
    assert response.status_code == 200
    data = response.json()
    assert "status" in data
    assert "timestamp" in data
    assert "dependencies" in data
    assert data["status"] == "healthy"