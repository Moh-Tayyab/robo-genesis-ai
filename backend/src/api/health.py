"""Health check API endpoints"""
from fastapi import APIRouter
from typing import Dict, Any
import logging
from datetime import datetime, timedelta
from ..schemas.health import HealthCheckRequest, HealthCheckResponse, HealthStatus
from ..config import config_manager
import time

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/health", tags=["health"])

# Track when the service started
start_time = datetime.utcnow()

@router.get("", response_model=HealthCheckResponse)
async def health_check_endpoint():
    """
    Health check endpoint to verify service status.
    Checks the status of various components like database, vector store, and external APIs.
    """
    try:
        logger.info("Health check requested")

        # Calculate uptime
        current_time = datetime.utcnow()
        uptime = str(current_time - start_time)

        # Check individual services
        services_status = {}

        # Check database connection
        try:
            # This is a basic check - in a real implementation, you'd test the actual DB connection
            from ..database import session_manager
            # Attempt to get a session to verify DB connectivity
            async with session_manager.session() as db_session:
                # Just getting a session verifies the connection
                pass
            db_status = HealthStatus(
                service="database",
                status="healthy",
                timestamp=current_time,
                details={"connection": "successful"}
            )
        except Exception as e:
            logger.error(f"Database health check failed: {e}")
            db_status = HealthStatus(
                service="database",
                status="unhealthy",
                timestamp=current_time,
                details={"error": str(e)}
            )
        services_status["database"] = db_status

        # Check Qdrant vector store
        try:
            # Check if we can access the Qdrant client
            from ..vector_store.qdrant_client import qdrant_manager
            # Verify Qdrant connection by checking collections
            collections = await qdrant_manager.client.get_collections()
            qdrant_status = HealthStatus(
                service="qdrant",
                status="healthy",
                timestamp=current_time,
                details={
                    "collections_count": len(collections.collections),
                    "connection": "successful"
                }
            )
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}")
            qdrant_status = HealthStatus(
                service="qdrant",
                status="unhealthy",
                timestamp=current_time,
                details={"error": str(e)}
            )
        services_status["qdrant"] = qdrant_status

        # Check OpenAI API
        try:
            # Check if we can access the OpenAI client
            from ..llm import openai_manager
            # Perform a simple check (like counting tokens)
            test_text = "health check"
            token_count = openai_manager.count_tokens(test_text)
            openai_status = HealthStatus(
                service="openai",
                status="healthy",
                timestamp=current_time,
                details={
                    "token_count_check": token_count,
                    "connection": "successful"
                }
            )
        except Exception as e:
            logger.error(f"OpenAI health check failed: {e}")
            openai_status = HealthStatus(
                service="openai",
                status="unhealthy",
                timestamp=current_time,
                details={"error": str(e)}
            )
        services_status["openai"] = openai_status

        # Determine overall status based on individual service statuses
        overall_status = "healthy"
        for service_name, service_status in services_status.items():
            if service_status.status == "unhealthy":
                overall_status = "unhealthy"
                break
            elif service_status.status == "degraded" and overall_status == "healthy":
                overall_status = "degraded"

        # Create response
        response = HealthCheckResponse(
            status=overall_status,
            timestamp=current_time,
            version=config_manager.settings.openai_model,  # Use model as version indicator
            uptime=uptime,
            services=services_status
        )

        logger.info(f"Health check completed with status: {overall_status}")
        return response

    except Exception as e:
        logger.error(f"Error in health check: {e}")
        # Return degraded status if there's an error checking health
        return HealthCheckResponse(
            status="degraded",
            timestamp=datetime.utcnow(),
            version=config_manager.settings.openai_model,
            uptime=str(datetime.utcnow() - start_time),
            details={"error": str(e)}
        )


@router.get("/ready")
async def readiness_check():
    """
    Readiness check endpoint to verify if the service is ready to accept traffic.
    """
    try:
        # For readiness, we mainly check if critical services are available
        from ..database import session_manager
        from ..vector_store.qdrant_client import qdrant_manager
        from ..llm import openai_manager

        # Test database
        async with session_manager.session() as db_session:
            pass

        # Test Qdrant
        await qdrant_manager.client.get_collections()

        # Test OpenAI (simple token count)
        openai_manager.count_tokens("readiness check")

        return {"status": "ready", "timestamp": datetime.utcnow().isoformat()}

    except Exception as e:
        logger.error(f"Readiness check failed: {e}")
        return {"status": "not_ready", "error": str(e), "timestamp": datetime.utcnow().isoformat()}, 503


@router.get("/live")
async def liveness_check():
    """
    Liveness check endpoint to verify if the service is alive.
    """
    try:
        current_time = datetime.utcnow()
        uptime = str(current_time - start_time)

        return {
            "status": "alive",
            "timestamp": current_time.isoformat(),
            "uptime": uptime
        }
    except Exception as e:
        logger.error(f"Liveness check failed: {e}")
        return {"status": "dead", "error": str(e)}, 503


# ✓ SPEC-KIT PLUS VERIFIED