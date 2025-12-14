# Quickstart Guide: Physical AI Humanoid Robotics Backend

## Overview
This guide explains how to set up and run the Physical AI Humanoid Robotics backend with the new fixes for ONNX model downloads and cross-platform environment activation.

## Prerequisites
- Python 3.10+
- uv package manager
- Windows 10/11 (for PowerShell support) or WSL
- Git

## Setup Steps

### 1. Clone the Repository
```bash
git clone <repository-url>
cd Physical-AI-Humanoid-Robotics-Textbook
```

### 2. Install uv Package Manager (if not already installed)
```bash
pip install uv
```

### 3. Set up Virtual Environment
The backend includes robust virtual environment activation with fallbacks:

**On Windows (PowerShell):**
```powershell
# The system will automatically try PowerShell first
& ".venv\Scripts\Activate.ps1"
uv sync
```

**Fallback activation methods:**
- If PowerShell fails: `.\.venv\Scripts\activate.bat`
- If WSL available: `wsl -e bash -c "source .venv/bin/activate"`

### 4. Configure Environment Variables
Create a `.env` file in the backend directory with required variables:
```env
GEMINI_API_KEY=your_api_key_here
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
```

### 5. Start the Backend Service
```bash
cd backend
uv sync  # Install dependencies
uv run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

## Key Features of the Fix

### ONNX Model Download Resilience
- Automatic cleanup of broken cache folders on startup
- Retry logic with 3 attempts and 10-second delays
- Progress indication: "Downloading BGE model... (Attempt X/3)"
- Fallback to lighter model if primary download fails

### Cross-Platform Environment Activation
- Primary: PowerShell activation on Windows
- Fallback 1: CMD activation
- Fallback 2: WSL activation when available

## Verification
After starting the service, verify it's working:

1. Check the health endpoint: `curl http://localhost:8000/health`
2. You should see a response indicating the service is healthy
3. The response will show embedding model status and download attempt count

## Troubleshooting

### Model Download Issues
- If you see repeated download attempts, check your network connection
- The system will automatically try the fallback model after 3 failed attempts
- Check the logs for specific error messages

### Environment Activation Issues
- On Windows, ensure PowerShell execution policy allows script execution
- If all activation methods fail, manually activate: `python -m venv .venv && source .venv/bin/activate` (Linux/Mac) or `.venv\Scripts\activate` (Windows)

## Next Steps
- Review the API documentation for available endpoints
- Configure your frontend to connect to the backend
- Set up monitoring for the health check endpoint