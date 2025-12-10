from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from uuid import UUID

# Request models
class ChatRequest(BaseModel):
    question: str
    sessionId: Optional[str] = None
    userId: Optional[str] = None

class SelectedTextChatRequest(ChatRequest):
    selectedText: str

# Response models
class Source(BaseModel):
    chapter: str
    section: str
    url: str
    similarity: float

class RetrievalMetadata(BaseModel):
    chunksRetrieved: int
    retrievalTimeMs: int

class ChatResponse(BaseModel):
    response: str
    sources: List[Source]
    sessionId: str
    retrievalMetadata: RetrievalMetadata