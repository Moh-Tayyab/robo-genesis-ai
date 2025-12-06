from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Dict, Any
from app.database import get_db
from app.services.translation_service import translation_service
from app.schemas.translate import TranslateRequest, TranslateResponse
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)

router = APIRouter()

@router.post("/translate/urdu", response_model=TranslateResponse)
async def translate_to_urdu_endpoint(
    request: TranslateRequest,
    db: AsyncSession = Depends(get_db)
):
    """
    Translate content to Urdu.
    """
    try:
        # Validate request
        if not request.content or len(request.content.strip()) == 0:
            raise HTTPException(status_code=400, detail="Content is required for translation")

        if request.targetLanguage != 'ur':
            raise HTTPException(status_code=400, detail="Only Urdu translation is supported")

        # Translate the content to Urdu
        translated_content = await translation_service.translate_to_urdu_cached(
            content=request.content
        )

        # Return the translated content
        response = TranslateResponse(
            translatedContent=translated_content,
            originalContent=request.content,
            targetLanguage=request.targetLanguage
        )

        return response
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error in translation endpoint: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")