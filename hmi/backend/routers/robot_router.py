"""REST 엔드포인트 (dump_modes 개념 없음, 플로우 시작/정지 위주).

이력/에러 조회는 auto-dump-bot의 /dump/history, /error/logs와 동일 패턴.
"""
from __future__ import annotations

import os
import struct
from typing import Optional

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse

from config import CARTER_IDS, ROBOT_IDS, settings
from database import get_connection, rows_to_dicts
from models import CommandResult, EmergencyStopRequest, MapInfo, NavigateRequest
from robot_bridge import bridge_manager

router = APIRouter(prefix="/api")


# ── 맵 파일 파싱 유틸 (의존성 없이: yaml 은 단순 파서, png 는 IHDR 헤더 직접 읽기) ──
def _parse_map_yaml(yaml_path: str) -> dict:
    """map.yaml 에서 image/resolution/origin 만 뽑는 경량 파서(pyyaml 불필요)."""
    info: dict = {}
    with open(yaml_path, "r", encoding="utf-8") as fh:
        for line in fh:
            line = line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, _, val = line.partition(":")
            key = key.strip()
            val = val.strip()
            if key == "image":
                info["image"] = val
            elif key == "resolution":
                info["resolution"] = float(val)
            elif key == "origin":
                nums = val.strip("[]").split(",")
                info["origin"] = [float(n) for n in nums[:2]]
    return info


def _png_size(png_path: str) -> tuple[int, int]:
    """PNG IHDR 청크에서 (width, height). 8바이트 시그니처 + 8바이트(len+type) 뒤 8바이트."""
    with open(png_path, "rb") as fh:
        header = fh.read(24)
    if header[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("PNG 시그니처 아님")
    width, height = struct.unpack(">II", header[16:24])
    return width, height


def _load_map_paths() -> tuple[str, str, dict]:
    """(yaml_path, png_path, parsed) 반환. png 는 yaml 과 같은 디렉토리의 image 파일명."""
    yaml_path = settings.map_yaml_path
    if not os.path.isfile(yaml_path):
        raise HTTPException(status_code=404, detail=f"map yaml 없음: {yaml_path}")
    parsed = _parse_map_yaml(yaml_path)
    png_path = os.path.join(os.path.dirname(yaml_path), parsed.get("image", ""))
    return yaml_path, png_path, parsed


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


@router.post("/commands/navigate/{robot_id}", response_model=CommandResult)
async def navigate(robot_id: str, payload: NavigateRequest) -> CommandResult:
    # 자유 클릭 이동은 carter1/carter2(ROS 네임스페이스)만 대상 — waste/disinfect(HMI id)와는 별개 체계.
    if robot_id not in CARTER_IDS:
        raise HTTPException(status_code=404, detail=f"알 수 없는 robot_id: {robot_id}")
    ok = bridge_manager.publish_nav_goal(robot_id, payload.x, payload.y, payload.yaw)
    if not ok:
        raise HTTPException(status_code=503, detail="goal publisher 미초기화")
    return CommandResult(result="SUCCESS", robot_id=robot_id)


@router.get("/map-info", response_model=MapInfo)
async def get_map_info() -> MapInfo:
    _, png_path, parsed = _load_map_paths()
    if "resolution" not in parsed or "origin" not in parsed:
        raise HTTPException(status_code=500, detail="map yaml 파싱 실패(resolution/origin 없음)")
    if not os.path.isfile(png_path):
        raise HTTPException(status_code=404, detail=f"map 이미지 없음: {png_path}")
    width, height = _png_size(png_path)
    origin_x, origin_y = parsed["origin"]
    return MapInfo(
        resolution=parsed["resolution"],
        origin_x=origin_x,
        origin_y=origin_y,
        width=width,
        height=height,
    )


@router.get("/map-image")
async def get_map_image() -> FileResponse:
    _, png_path, _ = _load_map_paths()
    if not os.path.isfile(png_path):
        raise HTTPException(status_code=404, detail=f"map 이미지 없음: {png_path}")
    return FileResponse(png_path, media_type="image/png")


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
