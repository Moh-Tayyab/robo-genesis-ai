from typing import Optional
from pydantic_settings import BaseSettings
from pydantic import Field
import os


class Settings(BaseSettings):
    """Application settings using pydantic-settings"""

    # OpenAI settings
    openai_api_key: str = Field(..., description="OpenAI API key")
    openai_model: str = Field(default="gpt-4o-mini", description="OpenAI model to use")
    embedding_model: str = Field(default="text-embedding-3-small", description="OpenAI embedding model to use")

    # Database settings
    database_url: str = Field(..., description="Database URL")

    # Qdrant settings
    qdrant_url: str = Field(..., description="Qdrant URL")
    qdrant_api_key: Optional[str] = Field(default=None, description="Qdrant API key")
    qdrant_collection_name: str = Field(default="book_chunks", description="Qdrant collection name")

    # Rate limiting settings
    rate_limit_per_minute_ip: int = Field(default=30, description="Rate limit per minute per IP")
    rate_limit_per_minute_session: int = Field(default=10, description="Rate limit per minute per session")

    # Application settings
    environment: str = Field(default="development", description="Environment (development/production)")
    log_level: str = Field(default="INFO", description="Logging level")
    max_content_length: int = Field(default=4000, description="Maximum content length for selected text")
    max_embedding_batch_size: int = Field(default=100, description="Maximum batch size for embedding generation")

    # Performance settings
    retrieval_top_k: int = Field(default=5, description="Number of chunks to retrieve")
    retrieval_score_threshold: float = Field(default=0.75, description="Minimum score threshold for retrieval")

    # API settings
    api_prefix: str = Field(default="/v1", description="API prefix")
    debug: bool = Field(default=False, description="Debug mode")

    class Config:
        """Configuration class"""
        env_file = ".env"  # Load from .env file
        env_file_encoding = 'utf-8'
        case_sensitive = False
        env_nested_delimiter = '__'


class ConfigManager:
    """Manages application configuration"""

    def __init__(self):
        self._settings = None

    @property
    def settings(self) -> Settings:
        """Get application settings, loading them if necessary"""
        if self._settings is None:
            self._settings = Settings()
        return self._settings

    def validate_settings(self):
        """Validate that all required settings are present and correct"""
        s = self.settings

        # Validate required fields are not empty
        if not s.openai_api_key:
            raise ValueError("OPENAI_API_KEY is required")

        if not s.database_url:
            raise ValueError("DATABASE_URL is required")

        if not s.qdrant_url:
            raise ValueError("QDRANT_URL is required")

        # Validate numeric values
        if s.rate_limit_per_minute_ip <= 0:
            raise ValueError("rate_limit_per_minute_ip must be positive")

        if s.rate_limit_per_minute_session <= 0:
            raise ValueError("rate_limit_per_minute_session must be positive")

        if s.max_content_length <= 0:
            raise ValueError("max_content_length must be positive")

        if s.retrieval_top_k <= 0:
            raise ValueError("retrieval_top_k must be positive")

        if s.retrieval_score_threshold < 0 or s.retrieval_score_threshold > 1:
            raise ValueError("retrieval_score_threshold must be between 0 and 1")

        # Validate environment
        if s.environment not in ["development", "staging", "production"]:
            raise ValueError("environment must be one of: development, staging, production")

        # Validate log level
        import logging
        valid_log_levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
        if s.log_level.upper() not in valid_log_levels:
            raise ValueError(f"log_level must be one of: {valid_log_levels}")

    def get_database_config(self):
        """Get database-specific configuration"""
        return {
            "database_url": self.settings.database_url,
            "pool_size": 10,
            "max_overflow": 20,
            "pool_pre_ping": True,
            "pool_recycle": 300
        }

    def get_qdrant_config(self):
        """Get Qdrant-specific configuration"""
        return {
            "url": self.settings.qdrant_url,
            "api_key": self.settings.qdrant_api_key,
            "collection_name": self.settings.qdrant_collection_name,
            "top_k": self.settings.retrieval_top_k,
            "score_threshold": self.settings.retrieval_score_threshold
        }

    def get_openai_config(self):
        """Get OpenAI-specific configuration"""
        return {
            "api_key": self.settings.openai_api_key,
            "model": self.settings.openai_model,
            "embedding_model": self.settings.embedding_model
        }

    def get_rate_limit_config(self):
        """Get rate limiting configuration"""
        return {
            "ip_limit": self.settings.rate_limit_per_minute_ip,
            "session_limit": self.settings.rate_limit_per_minute_session
        }


# Global configuration manager instance
config_manager = ConfigManager()

# ✓ SPEC-KIT PLUS VERIFIED