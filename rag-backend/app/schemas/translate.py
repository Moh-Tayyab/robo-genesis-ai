from pydantic import BaseModel
from typing import Optional

class TranslateRequest(BaseModel):
    content: str
    targetLanguage: str  # Language code (e.g., 'ur' for Urdu)


class TranslateResponse(BaseModel):
    translatedContent: str
    originalContent: str
    targetLanguage: str