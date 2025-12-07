# Implementation Tasks: Physical AI Humanoid Robotics Backend Fix & Verification

## Overview
This document outlines the implementation tasks for fixing fastembed ONNX model download crashes and ensuring reliable virtual environment activation across Windows PowerShell, CMD, and WSL environments. The implementation includes retry logic for model downloads, proper cache management, and cross-platform compatibility.

**Feature**: Physical AI Humanoid Robotics Backend Fix & Verification
**Branch**: `007-physical-ai-backend-fix`
**Date**: 2025-12-07
**Priority Order**: US1 (P1) → US2 (P2) → US3 (P3)

## Implementation Strategy
- **MVP First**: Implement US1 (Backend Service Startup) as the minimum viable product
- **Incremental Delivery**: Add US2 (Environment Activation) and US3 (Robust Downloads) in subsequent phases
- **Parallel Opportunities**: Configuration setup and utility functions can be developed in parallel with service implementation

## Dependencies
- US1 (P1) must be completed before US3 (P3) as robust downloads depend on basic download functionality
- US2 (P2) can be implemented independently but is foundational for deployment
- No dependencies between US1 and US2

## Parallel Execution Examples
- T001-T003 (Setup) can run in parallel with T004-T006 (Configuration)
- US2 tasks can run in parallel with US1/US3 implementation tasks
- Model download and cache management can be developed in parallel with health check updates

---

## Phase 1: Setup (Project Initialization)

- [X] T001 Create/update backend/pyproject.toml to include fastembed, loguru, and sentence-transformers dependencies
- [X] T002 Verify backend directory structure exists with src/, api/, services/, vector_store/ directories
- [X] T003 Set up basic testing environment with pytest configuration in backend/

## Phase 2: Foundational (Blocking Prerequisites)

- [X] T004 [P] Create EmbeddingModelConfig data class in backend/src/models/embedding_config.py with fields for retry parameters and model IDs
- [X] T005 [P] Create DownloadStatus and EnvironmentActivationResult data classes in backend/src/models/status.py based on data model
- [X] T006 [P] Update backend/src/config.py to include embedding configuration parameters
- [X] T007 Create utility functions for cross-platform path handling in backend/src/utils/path_utils.py
- [X] T008 Create cache management utilities in backend/src/utils/cache_utils.py for clearing ONNX cache

## Phase 3: User Story 1 - Backend Service Startup (Priority: P1)

**Story Goal**: As a developer, I need the backend service to start reliably on Windows 10/11 systems so that I can test and develop features without encountering crashes during the ONNX model download process.

**Independent Test Criteria**: The backend service can be started successfully on Windows 10/11 systems with PowerShell, and the ONNX model downloads properly without crashing during startup.

**Acceptance Scenarios**:
1. Given a fresh installation with no cached ONNX models, when I start the backend service, then the BGE model downloads successfully with retry logic and progress indication
2. Given a system with a broken/corrupted ONNX cache, when I start the backend service, then the cache is cleared and a fresh download is initiated

- [X] T009 [P] [US1] Update backend/src/embedding_service.py to implement cache cleanup before model initialization
- [X] T010 [P] [US1] Create model initialization function in backend/src/embedding_service.py that handles broken cache scenarios
- [X] T011 [US1] Update backend/src/main.py to call cache cleanup function on application startup
- [ ] T012 [US1] Test backend service startup with clean environment to verify no crashes occur during model initialization

## Phase 4: User Story 2 - Cross-Platform Environment Activation (Priority: P2)

**Story Goal**: As a developer, I need the virtual environment to activate reliably across Windows PowerShell, CMD, and WSL environments so that I can work consistently regardless of my development setup.

**Independent Test Criteria**: The virtual environment activates successfully using PowerShell, CMD, or WSL fallback mechanisms as appropriate for the system.

**Acceptance Scenarios**:
1. Given a Windows system with PowerShell available, when I run the startup script, then the virtual environment activates via PowerShell
2. Given a Windows system where PowerShell fails, when I run the startup script, then the system falls back to CMD activation
3. Given a system with WSL available, when I run the startup script, then the system can activate via WSL if needed

- [X] T013 [P] [US2] Create cross-platform environment activation utility in backend/src/utils/env_activation.py
- [X] T014 [P] [US2] Implement PowerShell activation function with fallback to CMD in env_activation.py
- [X] T015 [P] [US2] Implement WSL activation fallback in env_activation.py
- [ ] T016 [US2] Update backend documentation to reflect new environment activation procedures
- [ ] T017 [US2] Test environment activation on different platforms (Windows, WSL, Linux)

## Phase 5: User Story 3 - Robust Model Download Process (Priority: P3)

**Story Goal**: As a user, I need the embedding model download process to be resilient to network issues and partial failures so that the system can recover gracefully from temporary problems.

**Independent Test Criteria**: The system can handle network failures during model download and either retry successfully or fall back to an alternative model with appropriate warnings.

**Acceptance Scenarios**:
1. Given a stable network connection, when the model download starts, then the download completes successfully with progress indication
2. Given a network interruption during download, when the system encounters the failure, then it retries up to 3 times with 10-second delays
3. Given multiple failed download attempts, when all retries are exhausted, then the system falls back to a lighter model or continues with appropriate warning

- [X] T018 [P] [US3] Update embedding_service.py to implement retry logic with 3 attempts and 10-second delays
- [X] T019 [P] [US3] Add progress indication using loguru with "Downloading BGE model... (Attempt X/3)" format
- [X] T020 [P] [US3] Implement fallback to "sentence-transformers/all-MiniLM-L6-v2" model when primary download fails
- [X] T021 [US3] Update health check endpoint in backend/src/api/health.py to report model download status and attempt count
- [X] T022 [US3] Test model download with simulated network failures to verify retry and fallback behavior

## Phase 6: Polish & Cross-Cutting Concerns

- [X] T023 Update logging configuration in backend/src/config.py to include proper loguru setup for progress indication
- [X] T024 Add error handling for edge cases like read-only temp directories and insufficient disk space
- [ ] T025 Update quickstart documentation with new environment activation instructions
- [ ] T026 Create integration tests to verify all user stories work together
- [ ] T027 Update README with troubleshooting section for common environment activation and model download issues