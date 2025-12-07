
import os
import asyncio
import httpx
from dotenv import load_dotenv

load_dotenv()

api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    print("No GEMINI_API_KEY found")
    exit(1)

async def list_models():
    optimizer_url = f"https://generativelanguage.googleapis.com/v1beta/models?key={api_key}"
    async with httpx.AsyncClient() as client:
        response = await client.get(optimizer_url)
        
        if response.status_code == 200:
            models = response.json().get('models', [])
            print("Available Models:")
            for model in models:
                if 'generateContent' in model.get('supportedGenerationMethods', []):
                    print(f"- {model['name']}")
        else:
            print(f"Error listing models: {response.status_code} {response.text}")

if __name__ == "__main__":
    asyncio.run(list_models())
