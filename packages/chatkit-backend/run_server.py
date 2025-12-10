#!/usr/bin/env python3
"""
Script to run the ChatKit backend server with proper environment loading.
"""

import os
import sys
from dotenv import load_dotenv
import uvicorn


def main():
    """Main entry point to run the server."""
    # Load environment variables from .env file
    env_path = os.path.join(os.path.dirname(__file__), '.env')
    if os.path.exists(env_path):
        load_dotenv(env_path)
        print(f"Loaded environment variables from {env_path}")
    else:
        print(f"Warning: .env file not found at {env_path}")

    # Check if DATABASE_URL is available
    database_url = os.getenv('DATABASE_URL')
    if not database_url:
        print("ERROR: DATABASE_URL not configured in environment!")
        sys.exit(1)
    else:
        print("DATABASE_URL is configured")

    # Run the server
    uvicorn.run(
        "src.chatkit_backend.main:app",
        host=os.getenv("HOST", "0.0.0.0"),
        port=int(os.getenv("PORT", 8000)),
        reload=True
    )


if __name__ == "__main__":
    main()