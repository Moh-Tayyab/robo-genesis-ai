from fastapi import Request, HTTPException
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response
from collections import defaultdict
import time
from app.core.config import settings
import asyncio
from typing import Dict, Tuple

class RateLimitMiddleware(BaseHTTPMiddleware):
    def __init__(self, app):
        super().__init__(app)
        self.requests = defaultdict(list)  # Store request timestamps by IP
        self.session_requests = defaultdict(list)  # Store request timestamps by session
        self.ip_limit = settings.rate_limit_requests
        self.session_limit = settings.rate_limit_sessions
        self.window = 60  # 1 minute window

    def cleanup_old_requests(self, request_list: list, current_time: float):
        """Remove requests older than the window"""
        cutoff = current_time - self.window
        request_list[:] = [req_time for req_time in request_list if req_time > cutoff]

    def is_rate_limited(self, identifier: str, request_list: Dict[str, list], limit: int, current_time: float):
        """Check if the identifier is rate limited"""
        self.cleanup_old_requests(request_list[identifier], current_time)
        return len(request_list[identifier]) >= limit

    async def dispatch(self, request: Request, call_next):
        current_time = time.time()
        client_ip = request.client.host

        # Check IP-based rate limit
        if self.is_rate_limited(client_ip, self.requests, self.ip_limit, current_time):
            raise HTTPException(
                status_code=429,
                detail={
                    "error": {
                        "code": "RATE_LIMIT_EXCEEDED",
                        "message": f"Rate limit exceeded: {self.ip_limit} requests per minute per IP"
                    }
                }
            )

        # Check session-based rate limit if session_id is provided
        session_id = request.query_params.get("sessionId") or request.headers.get("X-Session-Id")
        if session_id:
            if self.is_rate_limited(session_id, self.session_requests, self.session_limit, current_time):
                raise HTTPException(
                    status_code=429,
                    detail={
                        "error": {
                            "code": "RATE_LIMIT_EXCEEDED",
                            "message": f"Rate limit exceeded: {self.session_limit} requests per minute per session"
                        }
                    }
                )

        # Add current request to tracking
        self.requests[client_ip].append(current_time)
        if session_id:
            self.session_requests[session_id].append(current_time)

        response = await call_next(request)
        return response