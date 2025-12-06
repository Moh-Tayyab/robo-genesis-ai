import time
import asyncio
from typing import Dict, Optional
from collections import defaultdict
from fastapi import Request, HTTPException, status
from datetime import datetime, timedelta
import logging

logger = logging.getLogger(__name__)

class RateLimiter:
    """Rate limiter for API requests based on IP and session"""

    def __init__(self, ip_limit: int = 30, ip_window: int = 60, session_limit: int = 10, session_window: int = 60):
        """
        Initialize rate limiter

        Args:
            ip_limit: Number of requests allowed per IP per window
            ip_window: Time window in seconds for IP limit
            session_limit: Number of requests allowed per session per window
            session_window: Time window in seconds for session limit
        """
        self.ip_limit = ip_limit
        self.ip_window = ip_window
        self.session_limit = session_limit
        self.session_window = session_window

        # Track requests by IP and session
        self.ip_requests: Dict[str, list] = defaultdict(list)
        self.session_requests: Dict[str, list] = defaultdict(list)

    def _clean_old_requests(self, request_times: list, window: int):
        """Remove request times that are older than the window"""
        cutoff_time = time.time() - window
        return [req_time for req_time in request_times if req_time > cutoff_time]

    def _is_rate_limited(self, identifier: str, requests_dict: Dict, limit: int, window: int) -> bool:
        """Check if an identifier is rate limited"""
        # Clean old requests
        requests_dict[identifier] = self._clean_old_requests(requests_dict[identifier], window)

        # Check if limit is exceeded
        if len(requests_dict[identifier]) >= limit:
            return True

        # Add current request
        requests_dict[identifier].append(time.time())
        return False

    def is_ip_limited(self, ip: str) -> bool:
        """Check if IP is rate limited"""
        return self._is_rate_limited(ip, self.ip_requests, self.ip_limit, self.ip_window)

    def is_session_limited(self, session_id: str) -> bool:
        """Check if session is rate limited"""
        return self._is_rate_limited(session_id, self.session_requests, self.session_limit, self.session_window)


# Global rate limiter instance
rate_limiter = RateLimiter(
    ip_limit=30,      # 30 requests per minute per IP
    session_limit=10  # 10 requests per minute per session
)


async def rate_limit_middleware(request: Request):
    """
    Rate limiting middleware for FastAPI
    Checks both IP and session-based limits
    """
    # Get client IP
    client_ip = request.client.host if request.client else "unknown"

    # Check if session ID is provided in the request
    session_id = None
    if "session-id" in request.headers:
        session_id = request.headers["session-id"]
    elif hasattr(request.state, 'session_id'):
        session_id = request.state.session_id

    # Check IP rate limit
    if rate_limiter.is_ip_limited(client_ip):
        logger.warning(f"IP rate limit exceeded for {client_ip}")
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail={
                "error": "RATE_LIMIT_EXCEEDED",
                "message": "Too many requests from this IP address",
                "retry_after": "60"
            }
        )

    # Check session rate limit if session ID is available
    if session_id and rate_limiter.is_session_limited(session_id):
        logger.warning(f"Session rate limit exceeded for {session_id}")
        raise HTTPException(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            detail={
                "error": "RATE_LIMIT_EXCEEDED",
                "message": "Too many requests for this session",
                "retry_after": "60"
            }
        )

    # Add rate limit headers to response
    request.state.x_rate_limit_limit = rate_limiter.ip_limit
    request.state.x_rate_limit_remaining = rate_limiter.ip_limit - len(
        rate_limiter.ip_requests[client_ip]
    )
    request.state.x_rate_limit_reset = int(time.time()) + rate_limiter.ip_window


class RateLimitMiddleware:
    """FastAPI middleware class for rate limiting"""

    def __init__(self, rate_limiter: RateLimiter = rate_limiter):
        self.rate_limiter = rate_limiter

    async def __call__(self, request: Request, call_next):
        # Get client IP
        client_ip = request.client.host if request.client else "unknown"

        # Check if session ID is provided in the request
        session_id = None
        if "session-id" in request.headers:
            session_id = request.headers["session-id"]
        elif hasattr(request.state, 'session_id'):
            session_id = request.state.session_id

        # Check IP rate limit
        if self.rate_limiter.is_ip_limited(client_ip):
            logger.warning(f"IP rate limit exceeded for {client_ip}")
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail={
                    "error": "RATE_LIMIT_EXCEEDED",
                    "message": "Too many requests from this IP address",
                    "retry_after": "60"
                }
            )

        # Check session rate limit if session ID is available
        if session_id and self.rate_limiter.is_session_limited(session_id):
            logger.warning(f"Session rate limit exceeded for {session_id}")
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail={
                    "error": "RATE_LIMIT_EXCEEDED",
                    "message": "Too many requests for this session",
                    "retry_after": "60"
                }
            )

        # Add rate limit headers to response
        request.state.x_rate_limit_limit = self.rate_limiter.ip_limit
        request.state.x_rate_limit_remaining = self.rate_limiter.ip_limit - len(
            self.rate_limiter.ip_requests[client_ip]
        )
        request.state.x_rate_limit_reset = int(time.time()) + self.rate_limiter.ip_window

        # Process the request
        response = await call_next(request)

        # Add rate limit headers to response
        response.headers["X-RateLimit-Limit"] = str(self.rate_limiter.ip_limit)
        response.headers["X-RateLimit-Remaining"] = str(
            self.rate_limiter.ip_limit - len(self.rate_limiter.ip_requests[client_ip])
        )
        response.headers["X-RateLimit-Reset"] = str(int(time.time()) + self.rate_limiter.ip_window)

        return response

# ✓ SPEC-KIT PLUS VERIFIED