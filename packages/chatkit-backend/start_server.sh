#!/bin/bash
# Script to start the ChatKit backend server with proper environment

# Change to the project directory
cd "$(dirname "$0")"

# Load environment variables from .env file
set -a  # automatically export all variables
source .env
set +a

# Verify that DATABASE_URL is set
if [ -z "$DATABASE_URL" ]; then
    echo "ERROR: DATABASE_URL is not set in .env file"
    exit 1
else
    echo "DATABASE_URL is configured"
fi

# Run the server with uvicorn
exec uvicorn src.chatkit_backend.main:app --host 0.0.0.0 --port 8000 --reload