from typing import Dict, Any, Optional
from app.core.config import settings
from openai import AsyncOpenAI
import logging
import asyncio

logger = logging.getLogger(__name__)

class TranslationService:
    def __init__(self):
        self.client = AsyncOpenAI(api_key=settings.openai_api_key)

    async def translate_to_urdu(self, content: str) -> str:
        """
        Translate content to Urdu using OpenAI.
        """
        try:
            # Use OpenAI to translate content to Urdu
            response = await self.client.chat.completions.create(
                model=settings.openai_model,
                messages=[
                    {
                        "role": "system",
                        "content": "You are a professional translator. Translate the following content to fluent Urdu. Maintain the technical accuracy while making it readable in Urdu. Use proper Urdu grammar and sentence structure."
                    },
                    {
                        "role": "user",
                        "content": f"Please translate the following content to fluent Urdu:\n\n{content}"
                    }
                ],
                max_tokens=len(content) * 2,  # Allow more tokens for Urdu translation
                temperature=0.3
            )

            translated_content = response.choices[0].message.content.strip()
            return translated_content
        except Exception as e:
            logger.error(f"Error translating to Urdu: {e}")
            return content  # Return original content if translation fails

    async def translate_to_urdu_cached(self, content: str) -> str:
        """
        Translate content to Urdu with caching to avoid repeated API calls.
        """
        # In a real implementation, you'd want to implement proper caching
        # using Redis or similar, but for now we'll just call the translation method
        return await self.translate_to_urdu(content)

translation_service = TranslationService()