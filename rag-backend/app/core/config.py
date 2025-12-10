from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # API Settings
    app_name: str = "RAG Chatbot API"
    api_v1_str: str = "/api/v1"

    # Database Settings
    database_url: Optional[str] = None

    # Qdrant Settings
    qdrant_url: str
    qdrant_api_key: Optional[str] = None
    qdrant_top_k: int = 5
    qdrant_score_threshold: float = 0.75

    # OpenAI Settings
    openai_api_key: str
    openai_model: str = "gpt-4o-mini"
    embedding_model: str = "text-embedding-3-small"

    # Rate Limiting
    rate_limit_requests: int = 30  # per minute per IP
    rate_limit_sessions: int = 10  # per minute per session

    # Text Processing
    max_selected_text_length: int = 4000

    # Reranking
    rerank_threshold: float = 0.85

    class Config:
        env_file = ".env"
        case_sensitive = True


settings = Settings()