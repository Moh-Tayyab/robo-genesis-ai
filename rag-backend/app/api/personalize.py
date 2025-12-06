from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Dict, Any
from app.database import get_db
from app.services.personalization_service import personalization_service
from app.schemas.personalize import PersonalizeRequest, PersonalizeResponse
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)

router = APIRouter()

@router.post("/personalize", response_model=PersonalizeResponse)
async def personalize_content_endpoint(
    request: PersonalizeRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Personalize chapter content based on user background.
    """
    try:
        # Validate request
        if not request.content or len(request.content.strip()) == 0:
            raise HTTPException(status_code=400, detail="Content is required for personalization")

        # Personalize the content
        personalized_content = await personalization_service.personalize_content(
            user_id=request.userId,
            chapter_id=request.chapterId,
            content=request.content,
            user_background=request.userBackground
        )

        # Return the personalized content
        response = PersonalizeResponse(
            personalizedContent=personalized_content,
            originalContent=request.content,
            userId=request.userId,
            chapterId=request.chapterId
        )

        return response
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error in personalization endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")