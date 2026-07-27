"""Pydantic 요청 모델 — carter1/carter2 로 로봇 식별 통일(HMI v2, 작업선택식 대응).
robot_id 필드는 이제 "waste"/"disinfect" 가 아니라 "carter1"/"carter2" 값을 받는다."""
from __future__ import annotations

from typing import Optional

from pydantic import BaseModel


class EmergencyStopRequest(BaseModel):
    # None = 전체 긴급정지, 값 있으면 해당 로봇만(carter1/carter2)
    robot_id: Optional[str] = None


class CommandResult(BaseModel):
    result: str
    robot_id: Optional[str] = None


class TaskSelectRequest(BaseModel):
    # [HMI v2 신규] 로봇에 작업(trash|spray)을 배정 — /carterN/task_select 로 그대로 발행.
    task: str


class NavigateRequest(BaseModel):
    # 자유 클릭 이동 목표(map 프레임 world 좌표). yaw 는 라디안, 미지정 시 0.
    x: float
    y: float
    yaw: float = 0.0


class MapInfo(BaseModel):
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
