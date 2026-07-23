"""FastAPI 진입점 — lifespan + queue consumer loop + /ws/robot/status.

큐 소비자(_queue_consumer_loop)가 유일하게 "ROS raw → 프론트 포맷" 변환 후
connection_manager.broadcast() 한다. 로봇별 캐시(latest_status)로 확장됨.
"""
from __future__ import annotations

import asyncio
import logging
from contextlib import asynccontextmanager
from datetime import datetime, timezone

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from config import ROBOT_IDS, settings
from connection_manager import connection_manager
from database import init_db
from robot_bridge import bridge_manager
from routers import robot_router

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 로봇별 최신 상태 캐시 (이전엔 dict 하나 → 이번엔 robot_id 키 dict)
latest_status: dict[str, dict] = {
    rid: {"process_state": "대기", "recovery_stage": "IDLE"} for rid in ROBOT_IDS
}

broadcast_queue: asyncio.Queue = asyncio.Queue(maxsize=settings.queue_maxsize)


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


async def _queue_consumer_loop() -> None:
    """브리지 큐를 소비하며 로봇별 캐시 갱신 후 브로드캐스트."""
    while True:
        raw = await broadcast_queue.get()
        robot_id = raw.get("robotId")
        msg_type = raw.get("type")

        if robot_id not in latest_status:
            continue

        if msg_type == "PROCESS_STATE":
            latest_status[robot_id]["process_state"] = raw.get("payload")
            await connection_manager.broadcast(
                {
                    "type": "ROBOT_STATUS",
                    "robotId": robot_id,
                    "payload": dict(latest_status[robot_id]),
                    "timestamp": _now_iso(),
                }
            )
        elif msg_type == "RECOVERY_STAGE":
            latest_status[robot_id]["recovery_stage"] = raw.get("payload")
            await connection_manager.broadcast(
                {
                    "type": "ROBOT_STATUS",
                    "robotId": robot_id,
                    "payload": dict(latest_status[robot_id]),
                    "timestamp": _now_iso(),
                }
            )
        elif msg_type == "SAFETY_EVENT":
            await connection_manager.broadcast(
                {
                    "type": "SAFETY_EVENT",
                    "robotId": robot_id,
                    "error_code": raw.get("error_code"),
                    "error_msg": raw.get("error_msg"),
                    "timestamp": _now_iso(),
                }
            )


@asynccontextmanager
async def lifespan(app: FastAPI):
    init_db()
    loop = asyncio.get_running_loop()
    bridge_manager.start_bridge(loop, broadcast_queue)
    consumer = asyncio.create_task(_queue_consumer_loop())
    logger.info("HMI 백엔드 시작 완료")
    try:
        yield
    finally:
        consumer.cancel()
        bridge_manager.shutdown()


app = FastAPI(title="격리 병동 로봇 관제 HMI", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=list(settings.cors_origins),
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(robot_router.router)


@app.websocket("/ws/robot/status")
async def ws_robot_status(websocket: WebSocket) -> None:
    await connection_manager.connect(websocket)
    # 접속 즉시 현재 캐시 스냅샷을 밀어준다 (초기 화면 공백 방지).
    for rid in ROBOT_IDS:
        await websocket.send_json(
            {
                "type": "ROBOT_STATUS",
                "robotId": rid,
                "payload": dict(latest_status[rid]),
                "timestamp": _now_iso(),
            }
        )
    try:
        while True:
            # 클라이언트로부터의 수신은 사용하지 않지만 연결 유지를 위해 대기.
            await websocket.receive_text()
    except WebSocketDisconnect:
        await connection_manager.disconnect(websocket)
