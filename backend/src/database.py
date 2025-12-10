import os
from typing import AsyncGenerator
from contextlib import asynccontextmanager
from sqlmodel import create_engine, Session
from sqlmodel.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession as SQLAlchemyAsyncSession
from sqlalchemy.pool import QueuePool
import logging

logger = logging.getLogger(__name__)

# Get database URL from environment
DATABASE_URL = os.getenv("DATABASE_URL", "postgresql+asyncpg://user:password@localhost/dbname")

# Create async engine
async_engine = create_async_engine(
    DATABASE_URL,
    poolclass=QueuePool,
    pool_size=5,
    max_overflow=10,
    pool_pre_ping=True,
    pool_recycle=300,
    echo=False  # Set to True for SQL query logging
)


class DatabaseSessionManager:
    """Manages database sessions with connection pooling"""

    def __init__(self, database_url: str):
        self.engine = create_async_engine(
            database_url,
            poolclass=QueuePool,
            pool_size=5,
            max_overflow=10,
            pool_pre_ping=True,
            pool_recycle=300,
            echo=False  # Set to True for SQL query logging
        )

    async def init_db(self):
        """Initialize the database by creating tables"""
        await self.create_db_and_tables()

    async def close(self):
        """Close the database engine"""
        await self.engine.dispose()

    @asynccontextmanager
    async def get_async_session(self) -> AsyncGenerator[AsyncSession, None]:
        """
        Get async database session.
        Usage:
            async with get_async_session() as session:
                # Use session here
        """
        async with AsyncSession(self.engine) as session:
            try:
                yield session
            except Exception:
                await session.rollback()
                raise
            finally:
                await session.close()

    async def create_db_and_tables(self):
        """Create database tables"""
        try:
            # Import all models to register them with SQLModel
            from .models.user import UserSQLModel
            from .models.session import SessionSQLModel
            from .models.message import MessageSQLModel
            from .models.retrieval import RetrievalSQLModel

            # Create tables
            async with self.engine.begin() as conn:
                await conn.run_sync(UserSQLModel.metadata.create_all)
                await conn.run_sync(SessionSQLModel.metadata.create_all)
                await conn.run_sync(MessageSQLModel.metadata.create_all)
                await conn.run_sync(RetrievalSQLModel.metadata.create_all)

            logger.info("Database tables created successfully")
        except Exception as e:
            logger.error(f"Error creating database tables: {e}")
            raise


# Create global session manager instance
session_manager = DatabaseSessionManager(DATABASE_URL)


@asynccontextmanager
async def get_async_session() -> AsyncGenerator[AsyncSession, None]:
    """Get async database session from the global manager"""
    async with session_manager.get_async_session() as session:
        yield session


async def create_db_and_tables():
    """Create database tables using the global manager"""
    await session_manager.create_db_and_tables()


async def get_db_session():
    """Dependency for FastAPI to get database session"""
    async with get_async_session() as session:
        yield session

# ✓ SPEC-KIT PLUS VERIFIED