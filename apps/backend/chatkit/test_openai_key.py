import os
import sys
from dotenv import load_dotenv
from openai import OpenAI

# Load environment variables
load_dotenv()

api_key = os.getenv("OPENAI_API_KEY")

if not api_key:
    print("❌ Error: OPENAI_API_KEY not found in environment variables.")
    sys.exit(1)

print(f"Checking API Key: {api_key[:8]}...{api_key[-4:]}")

try:
    client = OpenAI(api_key=api_key)
    # Attempt a simple API call (list models is usually cheap/free administrative call)
    print("Attempting to connect to OpenAI API...")
    models = client.models.list()
    
    # If we get here, the key is valid for at least listing models
    print("✅ Success! The OpenAI API key is working.")
    print(f"Successfully retrieved {len(list(models))} models.")
    
    # Try a very small completion just to be sure about generation
    print("\nTesting chat completion (User: 'Hello')...")
    completion = client.chat.completions.create(
      model="gpt-3.5-turbo", # Use a cheap/standard model for testing
      messages=[
        {"role": "user", "content": "Hello"}
      ],
      max_tokens=5
    )
    print("✅ Chat Completion Success!")
    print(f"Response: {completion.choices[0].message.content}")

except Exception as e:
    print("\n❌ API Key Check Failed!")
    print(f"Error: {str(e)}")
    sys.exit(1)
