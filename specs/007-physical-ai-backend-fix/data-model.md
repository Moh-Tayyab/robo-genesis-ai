# Data Model: Physical AI Humanoid Robotics Backend Fix & Verification

## Overview
This document defines the data models relevant to the backend fixes for ONNX model downloads and environment activation. Since this feature primarily focuses on operational fixes rather than new data structures, the data models are minimal and relate to configuration and status tracking.

## Key Entities

### Embedding Model Configuration
**Description**: Configuration parameters for managing embedding model downloads and fallback mechanisms

**Fields**:
- `primary_model_id`: String - The identifier for the primary BGE model ("Qdrant/bge-small-en-v1.5-onnx-q")
- `fallback_model_id`: String - The identifier for the fallback model ("sentence-transformers/all-MiniLM-L6-v2")
- `max_retry_attempts`: Integer - Maximum number of download attempts (default: 3)
- `retry_delay_seconds`: Float - Delay between retry attempts (default: 10.0)
- `cache_directory`: String - Path to the ONNX model cache directory
- `progress_format`: String - Format string for progress indication

### Download Status
**Description**: Status tracking for model download operations

**Fields**:
- `attempt_number`: Integer - Current attempt number (1-indexed)
- `total_attempts`: Integer - Maximum number of attempts
- `status`: String - Current status (e.g., "downloading", "failed", "success", "fallback_active")
- `progress_message`: String - Human-readable progress message
- `timestamp`: DateTime - When this status was recorded
- `error_message`: String? - Error details if status is "failed"

### Environment Activation Result
**Description**: Result of virtual environment activation attempts

**Fields**:
- `activation_method`: String - Method used ("powershell", "cmd", "wsl", "manual")
- `success`: Boolean - Whether activation was successful
- `command_used`: String - The actual command executed
- `output`: String - Output from the activation command
- `error`: String? - Error message if activation failed
- `timestamp`: DateTime - When activation was attempted

## Relationships
- An `Environment Activation Result` is associated with a specific system environment and activation attempt
- A `Download Status` tracks the progress of a single model download operation
- Configuration parameters in `Embedding Model Configuration` influence the behavior of download operations

## Validation Rules
- `max_retry_attempts` must be a positive integer between 1 and 10
- `retry_delay_seconds` must be a positive number between 1.0 and 60.0
- `cache_directory` must be a valid, writable path
- `attempt_number` must not exceed `total_attempts`