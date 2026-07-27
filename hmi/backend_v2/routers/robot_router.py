"""REST 엔드포인트 — [HMI v2] carter1/carter2 단일 식별체계 + 작업선택(task_select) 지원.

이력/에러 조회는 auto-dump-bot의 /dump/history, /error/logs와 동일 패턴.
"""
from __future__ import annotations

import asyncio
import os
import struct
from typing import Optional

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, StreamingResponse

from config import CARTER_IDS, DEFAULT_TASK_ASSIGNMENT, settings
from database import get_connection, rows_to_dicts
from models import CommandResult, EmergencyStopRequest, MapInfo, NavigateRequest, TaskSelectRequest
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
    """[HMI v2 재해석] task-select 모델엔 "대기 중인 미션을 시작"이라는 별도 게이트가 없다
    (task_select 발행 자체가 시작 트리거). "통합 시작" = 두 로봇에 기본 작업 조합을 한번에
    발행(사용자 결정 — 기존 관행 disinfect=carter1/waste=carter2 를 기본값으로 유지)."""
    # ★긴급정지 해제(resume) 겸용★ : 19_ 은 estop 을 /robot/command 의 "START" 로만 푼다(task_select
    #   는 별개 토픽이라 estop 을 안 풂). backend 가 START 를 아예 안 보내면 한번 긴급정지하면 영영
    #   못 풀리므로, "통합 시작" 이 START(estop 해제) 를 겸하도록 먼저 발행 후 기본 작업을 배정한다.
    bridge_manager.publish_command("START", robot_id=None)   # 두 로봇 estop 해제
    for carter_id, task in DEFAULT_TASK_ASSIGNMENT.items():
        bridge_manager.publish_task_select(carter_id, task)
    return CommandResult(result="SUCCESS")


@router.post("/commands/task/{carter_id}", response_model=CommandResult)
async def select_task(carter_id: str, payload: TaskSelectRequest) -> CommandResult:
    """[HMI v2 신규] 로봇 하나에 작업(trash|spray)을 배정. 클릭=배정=시작(19_ 의
    g_task_select_mission 이 이 메시지 수신 자체를 시작 트리거로 씀 — 별도 시작 스텝 없음)."""
    if carter_id not in CARTER_IDS:
        raise HTTPException(status_code=404, detail=f"알 수 없는 carter_id: {carter_id}")
    if payload.task not in ("trash", "spray"):
        raise HTTPException(status_code=422, detail=f"알 수 없는 task: {payload.task}")
    ok = bridge_manager.publish_task_select(carter_id, payload.task)
    if not ok:
        raise HTTPException(status_code=503, detail="task publisher 미초기화")
    return CommandResult(result="SUCCESS", robot_id=carter_id)


@router.post("/commands/estop", response_model=CommandResult)
async def emergency_stop(payload: EmergencyStopRequest) -> CommandResult:
    # robot_id가 None이면 전체 긴급정지
    targets = CARTER_IDS if payload.robot_id is None else [payload.robot_id]
    for cid in targets:
        bridge_manager.publish_command("EMERGENCY_STOP", robot_id=cid)
    return CommandResult(result="SUCCESS", robot_id=payload.robot_id)


@router.post("/commands/resume/{robot_id}", response_model=CommandResult)
async def resume(robot_id: str) -> CommandResult:
    """[HMI v2 신규] 로봇 하나만 긴급정지 해제("START"). start-all 과 달리 작업을 새로 배정하지
    않는다 — 로봇별 개별 긴급정지 버튼의 짝(로봇 카드의 "동작 재개" 버튼에서 호출)."""
    if robot_id not in CARTER_IDS:
        raise HTTPException(status_code=404, detail=f"알 수 없는 robot_id: {robot_id}")
    bridge_manager.publish_command("START", robot_id=robot_id)
    return CommandResult(result="SUCCESS", robot_id=robot_id)


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


@router.get("/queue")
async def get_queue(recent_limit: int = 3):
    """작업 큐 패널용 — 로봇별 진행 중(RUNNING) 작업 1건 + 최근 완료(DONE) 작업 몇 건.
    robot_bridge._track_task_history 가 process_state("대기"↔"RUNNING:...")로 tb_task_history 를
    채운다(자동 임무 기준 — 지도 클릭 경로예약은 프론트 routeQueue.ts 가 별도로 다룸)."""
    conn = get_connection()
    try:
        items: list[dict] = []
        for cid in CARTER_IDS:
            cur = conn.execute(
                "SELECT * FROM tb_task_history WHERE robot_id = ? AND end_time IS NULL "
                "ORDER BY start_time DESC LIMIT 1",
                (cid,),
            )
            items.extend(rows_to_dicts(cur.fetchall()))
            cur = conn.execute(
                "SELECT * FROM tb_task_history WHERE robot_id = ? AND end_time IS NOT NULL "
                "ORDER BY end_time DESC LIMIT ?",
                (cid, recent_limit),
            )
            items.extend(rows_to_dicts(cur.fetchall()))
        return items
    finally:
        conn.close()


@router.post("/commands/navigate/{robot_id}", response_model=CommandResult)
async def navigate(robot_id: str, payload: NavigateRequest) -> CommandResult:
    if robot_id not in CARTER_IDS:
        raise HTTPException(status_code=404, detail=f"알 수 없는 robot_id: {robot_id}")
    ok = bridge_manager.publish_nav_goal(robot_id, payload.x, payload.y, payload.yaw)
    if not ok:
        raise HTTPException(status_code=503, detail="goal publisher 미초기화")
    return CommandResult(result="SUCCESS", robot_id=robot_id)


@router.post("/commands/manual-override/{robot_id}", response_model=CommandResult)
async def manual_override(robot_id: str) -> CommandResult:
    """[HMI v2] 운영자 수동제어 시작 — 해당 로봇의 진행 중 작업을 즉시 중단시킨다(19_ 가 작업
    제너레이터를 버리고 IDLE 로 비켜서, 이후 navigate(goal_pose)를 Nav2 가 바로 주행). 프론트가
    '경로 시작' 시 1회 호출(웨이포인트마다 아님)."""
    if robot_id not in CARTER_IDS:
        raise HTTPException(status_code=404, detail=f"알 수 없는 robot_id: {robot_id}")
    bridge_manager.publish_command("MANUAL_OVERRIDE", robot_id=robot_id)
    return CommandResult(result="SUCCESS", robot_id=robot_id)


@router.post("/commands/dock/{robot_id}", response_model=CommandResult)
async def dock_return(robot_id: str) -> CommandResult:
    """[HMI v2] 도킹스테이션 복귀 + 작업 초기화. robot_id='all' 이면 두 로봇 모두.
    수동제어 경로 완료 시(프론트가 자동 호출) 또는 '도킹 복귀' 버튼에서 호출."""
    if robot_id == "all":
        bridge_manager.publish_command("DOCK_RETURN", robot_id=None)
        return CommandResult(result="SUCCESS", robot_id=None)
    if robot_id not in CARTER_IDS:
        raise HTTPException(status_code=404, detail=f"알 수 없는 robot_id: {robot_id}")
    bridge_manager.publish_command("DOCK_RETURN", robot_id=robot_id)
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


# ── YOLO 비전 스트림 (웹 VisionFeedPanel) ──
_MJPEG_BOUNDARY = "frame"


async def _mjpeg_generator(carter_id: str):
    """robot_bridge 가 들고 있는 '최신 annotated JPEG 1장'을 폴링해 MJPEG multipart 로 스트리밍.
    프레임이 바뀔 때만 내보낸다(뷰어 발행률이 실제 상한 — 여긴 그냥 폴링 주기일 뿐)."""
    last_frame: Optional[bytes] = None
    while True:
        frame = bridge_manager.get_latest_frame(carter_id)
        if frame is not None and frame is not last_frame:
            last_frame = frame
            yield (
                b"--" + _MJPEG_BOUNDARY.encode() + b"\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                + frame + b"\r\n"
            )
        await asyncio.sleep(0.05)


@router.get("/vision/{carter_id}/stream")
async def vision_stream(carter_id: str) -> StreamingResponse:
    """carterN 의 YOLO annotated 프레임(multi_robot_yolo_viewer.py 발행)을
    MJPEG(multipart/x-mixed-replace) 로 중계. 프론트 VisionFeedPanel 의 streamUrl 에 그대로
    넘기면 <img src=...> 로 바로 표시된다. 19_ 은 carter1+carter2 둘 다 YOLO 카메라가 있어
    (18_ 은 carter1 전용) 두 로봇 다 실제 프레임이 들어온다."""
    if carter_id not in CARTER_IDS:
        raise HTTPException(status_code=404, detail=f"알 수 없는 robot_id: {carter_id}")
    return StreamingResponse(
        _mjpeg_generator(carter_id),
        media_type=f"multipart/x-mixed-replace; boundary={_MJPEG_BOUNDARY}",
    )


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
