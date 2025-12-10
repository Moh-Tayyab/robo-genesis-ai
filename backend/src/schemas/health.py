"""Request/Response models for health endpoints"""
from typing import Dict, Any, Optional
from pydantic import BaseModel
from datetime import datetime


class HealthCheckRequest(BaseModel):
    """Request model for health check endpoint"""
    pass  # Health check doesn't require any specific input


class HealthStatus(BaseModel):
    """Model for individual service health status"""
    service: str
    status: str  # 'healthy', 'unhealthy', 'degraded'
    timestamp: datetime
    details: Optional[Dict[str, Any]] = None


class HealthCheckResponse(BaseModel):
    """Response model for health check endpoint"""
    status: str  # 'healthy', 'unhealthy', 'degraded'
    timestamp: datetime
    version: str
    uptime: str
    services: Optional[Dict[str, HealthStatus]] = None
    details: Optional[Dict[str, Any]] = None


# ✓ SPEC-KIT PLUS VERIFIED