"""Pydantic 요청 모델 — 로봇 2대 대응으로 robot_id 필드 추가."""
from __future__ import annotations

from typing import Optional

from pydantic import BaseModel


class EmergencyStopRequest(BaseModel):
    # None = 전체 긴급정지, 값 있으면 해당 로봇만
    robot_id: Optional[str] = None


class CommandResult(BaseModel):
    result: str
    robot_id: Optional[str] = None


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
