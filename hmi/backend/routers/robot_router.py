"""REST 엔드포인트 (dump_modes 개념 없음, 플로우 시작/정지 위주).

이력/에러 조회는 auto-dump-bot의 /dump/history, /error/logs와 동일 패턴.
"""
from __future__ import annotations

from typing import Optional

from fastapi import APIRouter

from config import ROBOT_IDS
from database import get_connection, rows_to_dicts
from models import CommandResult, EmergencyStopRequest
from robot_bridge import bridge_manager

router = APIRouter(prefix="/api")


@router.post("/commands/start-all", response_model=CommandResult)
async def start_all() -> CommandResult:
    for rid in ROBOT_IDS:
        bridge_manager.publish_command("START", robot_id=rid)
    return CommandResult(result="SUCCESS")


@router.post("/commands/start/{robot_id}", response_model=CommandResult)
async def start_robot(robot_id: str) -> CommandResult:
    bridge_manager.publish_command("START", robot_id=robot_id)
    return CommandResult(result="SUCCESS", robot_id=robot_id)


@router.post("/commands/estop", response_model=CommandResult)
async def emergency_stop(payload: EmergencyStopRequest) -> CommandResult:
    # robot_id가 None이면 전체 긴급정지
    targets = ROBOT_IDS if payload.robot_id is None else [payload.robot_id]
    for rid in targets:
        bridge_manager.publish_command("EMERGENCY_STOP", robot_id=rid)
    return CommandResult(result="SUCCESS", robot_id=payload.robot_id)


@router.get("/history")
async def get_history(robot_id: Optional[str] = None, limit: int = 50):
    conn = get_connection()
    try:
        if robot_id:
            cur = conn.execute(
                "SELECT * FROM tb_task_history WHERE robot_id = ? "
                "ORDER BY start_time DESC LIMIT ?",
                (robot_id, limit),
            )
        else:
            cur = conn.execute(
                "SELECT * FROM tb_task_history ORDER BY start_time DESC LIMIT ?",
                (limit,),
            )
        return rows_to_dicts(cur.fetchall())
    finally:
        conn.close()


@router.get("/errors")
async def get_errors(robot_id: Optional[str] = None, limit: int = 50):
    conn = get_connection()
    try:
        if robot_id:
            cur = conn.execute(
                "SELECT * FROM tb_error_log WHERE robot_id = ? "
                "ORDER BY error_time DESC LIMIT ?",
                (robot_id, limit),
            )
        else:
            cur = conn.execute(
                "SELECT * FROM tb_error_log ORDER BY error_time DESC LIMIT ?",
                (limit,),
            )
        return rows_to_dicts(cur.fetchall())
    finally:
        conn.close()
