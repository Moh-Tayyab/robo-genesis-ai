from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional
from app.database import get_db
from app.models.user import User
from app.schemas.user import UserProfileUpdate, UserProfileResponse
from app.core.config import settings
import logging
import uuid

logger = logging.getLogger(__name__)

router = APIRouter()

@router.get("/profile", response_model=UserProfileResponse)
async def get_user_profile(
    user_id: str,
    db: AsyncSession = Depends(get_db)
):
    """
    Get user profile with background information.
    """
    try:
        # Convert user_id string to UUID
        user_uuid = uuid.UUID(user_id)

        # Query user from database
        result = await db.execute(
            User.__table__.select().where(
                User.user_id == user_uuid
            )
        )
        user = result.fetchone()

        if not user:
            raise HTTPException(status_code=404, detail="User not found")

        # Convert to response format
        user_data = dict(user._mapping)
        return UserProfileResponse(
            user_id=str(user_data['user_id']),
            email=user_data['email'],
            programming_experience=user_data['programming_experience'],
            os_preference=user_data['os_preference'],
            gpu_available=user_data['gpu_available'],
            preferred_language=user_data['preferred_language'],
            learning_goals=user_data['learning_goals'],
            hardware_background=user_data['hardware_background'],
            software_background=user_data['software_background'],
            profile_completed=user_data['profile_completed'],
            preferences=user_data['preferences'],
            created_at=user_data['created_at'],
            updated_at=user_data['updated_at']
        )
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid user ID format")
    except Exception as e:
        logger.error(f"Error getting user profile: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.put("/profile", response_model=UserProfileResponse)
async def update_user_profile(
    user_id: str,
    profile_data: UserProfileUpdate,
    db: AsyncSession = Depends(get_db)
):
    """
    Update user profile with background information.
    """
    try:
        # Convert user_id string to UUID
        user_uuid = uuid.UUID(user_id)

        # Query user from database
        result = await db.execute(
            User.__table__.select().where(
                User.user_id == user_uuid
            )
        )
        user = result.fetchone()

        if not user:
            raise HTTPException(status_code=404, detail="User not found")

        # Build update data
        update_data = {}
        if profile_data.programming_experience is not None:
            update_data['programming_experience'] = profile_data.programming_experience
        if profile_data.os_preference is not None:
            update_data['os_preference'] = profile_data.os_preference
        if profile_data.gpu_available is not None:
            update_data['gpu_available'] = profile_data.gpu_available
        if profile_data.preferred_language is not None:
            update_data['preferred_language'] = profile_data.preferred_language
        if profile_data.learning_goals is not None:
            update_data['learning_goals'] = profile_data.learning_goals
        if profile_data.hardware_background is not None:
            update_data['hardware_background'] = profile_data.hardware_background
        if profile_data.software_background is not None:
            update_data['software_background'] = profile_data.software_background

        # Mark profile as completed if any background info is provided
        if any([
            profile_data.programming_experience,
            profile_data.os_preference,
            profile_data.gpu_available is not None,
            profile_data.preferred_language,
            profile_data.learning_goals,
            profile_data.hardware_background,
            profile_data.software_background
        ]):
            update_data['profile_completed'] = True

        # Update the user
        await db.execute(
            User.__table__.update()
            .where(User.user_id == user_uuid)
            .values(**update_data)
        )
        await db.commit()

        # Fetch updated user
        result = await db.execute(
            User.__table__.select().where(
                User.user_id == user_uuid
            )
        )
        updated_user = result.fetchone()

        # Convert to response format
        user_data = dict(updated_user._mapping)
        return UserProfileResponse(
            user_id=str(user_data['user_id']),
            email=user_data['email'],
            programming_experience=user_data['programming_experience'],
            os_preference=user_data['os_preference'],
            gpu_available=user_data['gpu_available'],
            preferred_language=user_data['preferred_language'],
            learning_goals=user_data['learning_goals'],
            hardware_background=user_data['hardware_background'],
            software_background=user_data['software_background'],
            profile_completed=user_data['profile_completed'],
            preferences=user_data['preferences'],
            created_at=user_data['created_at'],
            updated_at=user_data['updated_at']
        )
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid user ID format")
    except Exception as e:
        logger.error(f"Error updating user profile: {e}")
        await db.rollback()
        raise HTTPException(status_code=500, detail="Internal server error")