from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from app.core.config import settings

# Create the async database engine
engine = create_async_engine(
    settings.database_url or "postgresql+asyncpg://postgres:postgres@localhost/rag_chatbot",
    pool_pre_ping=True,
    pool_recycle=300,
)

# Create a configured "AsyncSessionLocal" class
AsyncSessionLocal = sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)

# Create a Base class for declarative models
Base = declarative_base()

async def get_db():
    """
    Dependency function to get database session
    """
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()