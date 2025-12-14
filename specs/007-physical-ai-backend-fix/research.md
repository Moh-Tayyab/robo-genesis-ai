# Research: Physical AI Humanoid Robotics Backend Fix & Verification

## Overview
This research document addresses the technical requirements for fixing fastembed ONNX model download crashes and ensuring reliable virtual environment activation across Windows PowerShell, CMD, and WSL environments.

## Decision: ONNX Model Download Retry Logic Implementation
**Rationale**: The original specification requires implementing retry logic with 3 attempts and 10-second delays for ONNX model downloads. This approach ensures resilience against temporary network issues or server unavailability.

**Implementation Strategy**:
- Use a try-catch block around the model download function
- Implement a loop that attempts the download up to 3 times
- Add 10-second delays between attempts using asyncio.sleep or time.sleep
- Log progress using loguru with the format "Downloading BGE model... (Attempt X/3)"

**Alternatives considered**:
1. Simple retry with exponential backoff (e.g., 1s, 2s, 4s) - rejected because the spec specifically requests 10-second delays
2. Using a third-party retry library like tenacity - rejected as it adds an unnecessary dependency for a simple retry mechanism

## Decision: Cache Management Strategy
**Rationale**: The specification requires deleting broken cache folders on first import to prevent startup crashes. This is critical for preventing the application from failing when corrupted ONNX files exist.

**Implementation Strategy**:
- Identify the cache directory: C:\Users\[username]\AppData\Local\Temp\fastembed_cache
- Use pathlib or os.path to construct the path in a cross-platform way
- Check if the cache directory exists before attempting to delete
- Use shutil.rmtree() to remove the entire cache directory
- Handle potential permission errors gracefully

**Alternatives considered**:
1. Only delete specific model files instead of entire cache - rejected because broken cache might affect multiple models
2. Move cache to project directory instead of temp - rejected as it would bloat the project and not follow expected cache behavior

## Decision: Cross-Platform Environment Activation
**Rationale**: The system must activate virtual environments using PowerShell first, then CMD fallback, then WSL if available. This ensures compatibility across different Windows development environments.

**Implementation Strategy**:
1. Try PowerShell activation: `& ".venv\Scripts\Activate.ps1"`
2. If PowerShell fails, try CMD: `.\.venv\Scripts\activate.bat`
3. If needed, WSL fallback: `wsl -e bash -c "source .venv/bin/activate && echo 'WSL activated'"`

**Alternatives considered**:
1. Using Python's subprocess to handle activation - rejected because virtual environment activation doesn't persist when called from Python
2. Cross-platform shell scripts - rejected as the requirement specifically mentions Windows PowerShell/WSL/CMD

## Decision: Fallback Model Strategy
**Rationale**: When primary model download fails, the system must fall back to a lighter model ("sentence-transformers/all-MiniLM-L6-v2") to ensure functionality.

**Implementation Strategy**:
- Try to initialize the primary BGE model first
- If initialization fails after all retry attempts, initialize the fallback model
- Log appropriate warnings when falling back
- Continue operation with reduced functionality if needed

**Alternatives considered**:
1. Multiple fallback models in a chain - rejected as the spec specifies a single fallback model
2. Skip embedding entirely - rejected as it would severely limit functionality

## Technical Dependencies Research
- **fastembed**: Library for efficient text embeddings using ONNX models
- **loguru**: User-friendly logging library that will be used for progress indication
- **pathlib**: Standard library for cross-platform path operations
- **shutil**: Standard library for high-level file operations (for cache deletion)
- **os**: Standard library for environment-specific operations

## Platform-Specific Considerations
- **Windows**: Primary target with PowerShell, needs special handling for path separators
- **WSL**: Linux compatibility layer that requires different activation approach
- **Cross-platform paths**: Use pathlib for proper path handling across platforms