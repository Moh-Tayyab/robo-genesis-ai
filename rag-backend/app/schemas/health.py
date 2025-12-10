from pydantic import BaseModel
from typing import Dict
from datetime import datetime

class HealthResponse(BaseModel):
    status: str
    timestamp: str
    dependencies: Dict[str, bool]