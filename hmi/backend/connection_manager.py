"""WebSocket connect/disconnect/broadcast.

auto-dump-bot에서 그대로 재사용 — 로봇 대수와 무관한 범용 코드(수정 불필요).
"""
import asyncio
import logging

from fastapi import WebSocket

logger = logging.getLogger(__name__)


class ConnectionManager:
    def __init__(self) -> None:
        self._active_connections: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        async with self._lock:
            self._active_connections.add(websocket)

    async def disconnect(self, websocket: WebSocket) -> None:
        async with self._lock:
            self._active_connections.discard(websocket)

    async def broadcast(self, message: dict) -> None:
        async with self._lock:
            targets = list(self._active_connections)
        if not targets:
            return
        dead_connections: list[WebSocket] = []
        for connection in targets:
            try:
                await connection.send_json(message)
            except Exception as exc:  # noqa: BLE001
                logger.warning("WebSocket send 실패, 정리 대상으로 등록: %s", exc)
                dead_connections.append(connection)
        if dead_connections:
            async with self._lock:
                for dead in dead_connections:
                    self._active_connections.discard(dead)

    @property
    def connection_count(self) -> int:
        return len(self._active_connections)


connection_manager = ConnectionManager()
