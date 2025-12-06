from pydantic import BaseModel
from typing import Dict, Any, Optional

class PersonalizeRequest(BaseModel):
    userId: str
    chapterId: str
    content: str
    userBackground: Dict[str, Any]


class PersonalizeResponse(BaseModel):
    personalizedContent: str
    originalContent: str
    userId: str
    chapterId: str