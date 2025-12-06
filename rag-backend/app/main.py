from fastapi import FastAPI
from app.api.health import router as health_router
from app.api.chat import router as chat_router
from app.api.user import router as user_router
from app.api.personalize import router as personalize_router
from app.api.translate import router as translate_router
from app.core.config import settings
from app.middleware.rate_limit import RateLimitMiddleware

# Create FastAPI app instance
app = FastAPI(
    title="RAG Chatbot API",
    description="RAG Chatbot Backend for Physical AI & Humanoid Robotics Textbook",
    version="0.1.0",
    openapi_url="/api/openapi.json",
    docs_url="/api/docs",
    redoc_url="/api/redoc",
)

# Add middleware
app.add_middleware(RateLimitMiddleware)

# Include API routers
app.include_router(health_router, prefix="/api", tags=["health"])
app.include_router(chat_router, prefix="/api", tags=["chat"])
app.include_router(user_router, prefix="/api", tags=["user"])
app.include_router(personalize_router, prefix="/api", tags=["personalize"])
app.include_router(translate_router, prefix="/api", tags=["translate"])

# Root endpoint
@app.get("/")
async def root():
    return {"message": "Welcome to the RAG Chatbot API"}