# Feature Specification: Physical AI Humanoid Robotics Backend Fix & Verification

**Feature Branch**: `1-physical-ai-backend-fix`
**Created**: 2025-12-07
**Status**: Draft
**Input**: User description: "PANAVERSITY PHYSICAL AI BOOK — FINAL NO-EXCUSE BACKEND FIX & VERIFICATION SPEC (250/250 GUARANTEED)

Claude, yeh specification ab tumhari zindagi hai. No vibe coding. No assumptions. Sirf perfection.

1. Operating System & Shell (MANDATORY)
   → Windows 10/11 + PowerShell (tumhare jaise exact setup)
   → Agar koi command fail ho → automatically fallback to bash via WSL if available
   → Agar WSL nahi → force activate .venv manually before any test

2. Critical Fix: fastembed ONNX Model Download Crash (ROOT CAUSE)
   → Problem: Incomplete download in Temp folder → ONNX file missing → server crash on startup
   → Permanent Fix:
     - On first import, delete broken cache folder:
       C:\\Users\\Muhammad Tayyab\\AppData\\Local\\Temp\\fastembed_cache\\models--qdrant--bge-small-en-v1.5-onnx-q
     - Force full re-download with retry logic (3 attempts, 10s delay)
     - Use loguru to show progress: \"Downloading BGE model... (Attempt X/3)\"
     - If all fail → fallback to lighter model (\"sentence-transformers/all-MiniLM-L6-v2\") or skip embedding init with warning

3. Virtual Environment Activation (100% GUARANTEED)
   → Before ANY test or startup, run:
     ```powershell
     & \".venv\\Scripts\\Activate.ps1\"   # PowerShell
     # OR if fails:
     .\\.venv\\Scripts\\activate.bat     # CMD fallback
     # OR if WSL available:
     wsl -e bash -c \"source .venv/bin/activate && echo 'WSL activated'\" client = AsyncOpenAI(
    api_key=os.getenv(\"GEMINI_API_KEY\"),
    base_url=\"https://generativelanguage.googleapis.com/v1beta/openai/\"
)
model = OpenAIChatCompletionsModel(model=\"gemini-2.0-flash\", openai_client=client)
result = await Runner.run(starting_agent=agent, input=message)
return result.final_output cd backend
Remove-Item -Recurse -Force \"C:\\Users\\Muhammad Tayyab\\AppData\\Local\\Temp\\fastembed_cache\" -ErrorAction SilentlyContinue
.\\.venv\\Scripts\\Activate.ps1
uv sync
uv run uvicorn src.main:app --reload --host 0.0.0.0 --port 8000 Downloading BGE model... (Attempt 1/3)
Model downloaded successfully!
MCP fallback: Not needed
Gemini Agent SDK: Working
Qdrant: Connected
Server running at http://0.0.0.0:8000
curl http://localhost:8000/health → {\"status\":\"healthy\",...}
"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Backend Service Startup (Priority: P1)

As a developer working with the Physical AI Humanoid Robotics Textbook project, I need the backend service to start reliably on Windows 10/11 systems so that I can test and develop features without encountering crashes during the ONNX model download process.

**Why this priority**: This is the foundational requirement for any development work. Without a stable backend service, no other functionality can be developed or tested.

**Independent Test**: The backend service can be started successfully on Windows 10/11 systems with PowerShell, and the ONNX model downloads properly without crashing during startup.

**Acceptance Scenarios**:

1. **Given** a fresh installation with no cached ONNX models, **When** I start the backend service, **Then** the BGE model downloads successfully with retry logic and progress indication
2. **Given** a system with a broken/corrupted ONNX cache, **When** I start the backend service, **Then** the cache is cleared and a fresh download is initiated

---

### User Story 2 - Cross-Platform Environment Activation (Priority: P2)

As a developer using different operating systems and shells, I need the virtual environment to activate reliably across Windows PowerShell, CMD, and WSL environments so that I can work consistently regardless of my development setup.

**Why this priority**: This ensures development workflow consistency across different environments and prevents environment-specific issues from blocking development.

**Independent Test**: The virtual environment activates successfully using PowerShell, CMD, or WSL fallback mechanisms as appropriate for the system.

**Acceptance Scenarios**:

1. **Given** a Windows system with PowerShell available, **When** I run the startup script, **Then** the virtual environment activates via PowerShell
2. **Given** a Windows system where PowerShell fails, **When** I run the startup script, **Then** the system falls back to CMD activation
3. **Given** a system with WSL available, **When** I run the startup script, **Then** the system can activate via WSL if needed

---

### User Story 3 - Robust Model Download Process (Priority: P3)

As a user of the Physical AI Humanoid Robotics system, I need the embedding model download process to be resilient to network issues and partial failures so that the system can recover gracefully from temporary problems.

**Why this priority**: This ensures reliability in various network conditions and prevents the system from becoming unusable due to temporary connectivity issues.

**Independent Test**: The system can handle network failures during model download and either retry successfully or fall back to an alternative model with appropriate warnings.

**Acceptance Scenarios**:

1. **Given** a stable network connection, **When** the model download starts, **Then** the download completes successfully with progress indication
2. **Given** a network interruption during download, **When** the system encounters the failure, **Then** it retries up to 3 times with 10-second delays
3. **Given** multiple failed download attempts, **When** all retries are exhausted, **Then** the system falls back to a lighter model or continues with appropriate warning

---

### Edge Cases

- What happens when the temp directory is read-only or inaccessible?
- How does the system handle insufficient disk space during model download?
- What occurs when both primary and fallback models fail to download?
- How does the system behave when running in restricted environments with limited internet access?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST delete broken cache folders on first import to prevent startup crashes
- **FR-002**: System MUST implement retry logic with 3 attempts and 10-second delays for ONNX model downloads
- **FR-003**: System MUST show progress indication with "Downloading BGE model... (Attempt X/3)" messages using loguru
- **FR-004**: System MUST fall back to a lighter model ("sentence-transformers/all-MiniLM-L6-v2") if primary model download fails
- **FR-005**: System MUST activate virtual environment using PowerShell first, then CMD fallback, then WSL if available
- **FR-006**: System MUST work reliably on Windows 10/11 with PowerShell as the primary shell
- **FR-007**: System MUST provide graceful degradation when all model download attempts fail

### Key Entities

- **ONNX Model Cache**: Temporary storage location for embedding models, located at C:\\Users\\[username]\\AppData\\Local\\Temp\\fastembed_cache
- **Virtual Environment**: Isolated Python environment containing project dependencies, located at .venv/
- **Embedding Model**: ML model used for text similarity and RAG operations, either primary BGE model or fallback model

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Backend service starts successfully on Windows 10/11 systems 95% of the time without crashing during model download
- **SC-002**: Model download process successfully completes within 3 attempts 90% of the time in various network conditions
- **SC-003**: Users can successfully activate the virtual environment across PowerShell, CMD, and WSL environments 100% of the time
- **SC-004**: System provides clear progress indication during model download with "Downloading BGE model... (Attempt X/3)" messages
- **SC-005**: When primary model download fails, system successfully falls back to lighter model 100% of the time