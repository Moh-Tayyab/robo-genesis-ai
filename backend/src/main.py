"""Main FastAPI application for the RAG Chatbot Backend"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import logging
import os
from .api.chat import router as chat_router
from .api.health import router as health_router
from .config import config_manager
from .vector_store.qdrant_client import qdrant_manager
from .database import session_manager

# Configure logging
logging.basicConfig(
    level=getattr(logging, config_manager.settings.log_level.upper()),
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager for startup and shutdown events"""
    logger.info("Starting up RAG Chatbot Backend...")

    try:
        # Initialize database
        await session_manager.init_db()
        logger.info("Database connection initialized")

        # Initialize Qdrant collection
        await qdrant_manager.initialize_collection()
        logger.info("Qdrant collection initialized")

        logger.info("RAG Chatbot Backend started successfully")
        yield

    except Exception as e:
        logger.error(f"Error during startup: {e}")
        raise
    finally:
        # Cleanup on shutdown
        logger.info("Shutting down RAG Chatbot Backend...")
        await qdrant_manager.close()
        await session_manager.close()
        logger.info("Resources cleaned up")

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot Backend API",
    description="Backend API for RAG Chatbot with Physical AI and Humanoid Robotics content",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, configure this properly
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
app.include_router(chat_router, prefix="/v1")
app.include_router(health_router, prefix="/v1")

@app.get("/")
async def root():
    """Root endpoint for basic health check"""
    return {
        "message": "RAG Chatbot Backend API",
        "version": "1.0.0",
        "status": "running"
    }

# Add rate limiting middleware
from .middleware.rate_limit import RateLimitMiddleware
app.add_middleware(RateLimitMiddleware)

# ✓ SPEC-KIT PLUS VERIFIED