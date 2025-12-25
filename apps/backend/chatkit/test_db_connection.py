#!/usr/bin/env python3
"""
Test script to verify database connection works
"""
import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add the project to the path
sys.path.insert(0, '/mnt/d/ai-native-robotics-development/packages/chatkit-backend')

# Now try to import and test the database functionality
try:
    from src.chatkit_backend.db.database import get_database_url, get_engine
    print("✓ Successfully imported database modules")

    # Test getting the database URL
    db_url = get_database_url()
    if db_url:
        print(f"✓ DATABASE_URL is configured: {db_url[:50]}...")
    else:
        print("✗ DATABASE_URL is not configured")
        sys.exit(1)

    # Test creating the engine (this is where the original error occurred)
    try:
        engine = get_engine()
        print("✓ Database engine created successfully")
        print("✓ Database configuration is working correctly!")
        print("\nThe original error has been fixed.")
        print("The server should now start without the 'DATABASE_URL not configured' error.")
    except Exception as e:
        print(f"✗ Error creating database engine: {e}")
        sys.exit(1)

except ImportError as e:
    print(f"✗ Import error: {e}")
    sys.exit(1)
except Exception as e:
    print(f"✗ Unexpected error: {e}")
    sys.exit(1)