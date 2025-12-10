from pydantic import BaseModel
from typing import Optional
from datetime import datetime

class UserProfileUpdate(BaseModel):
    programming_experience: Optional[str] = None  # "beginner", "intermediate", "advanced", "expert"
    os_preference: Optional[str] = None  # "windows", "macos", "linux"
    gpu_available: Optional[bool] = None  # Whether user has GPU
    preferred_language: Optional[str] = None  # "python", "typescript", "cpp", etc.
    learning_goals: Optional[str] = None  # Text field for learning goals
    hardware_background: Optional[str] = None  # "none", "maker", "engineer", "researcher"
    software_background: Optional[str] = None  # "none", "developer", "engineer", "researcher"


class UserProfileResponse(BaseModel):
    user_id: str
    email: Optional[str]
    programming_experience: Optional[str]
    os_preference: Optional[str]
    gpu_available: Optional[bool]
    preferred_language: Optional[str]
    learning_goals: Optional[str]
    hardware_background: Optional[str]
    software_background: Optional[str]
    profile_completed: bool
    preferences: Optional[str]
    created_at: datetime
    updated_at: datetime