"""환경변수 중앙관리 — frozen dataclass (auto-dump-bot 패턴 그대로).

🔧 튜닝: 포트/DB 경로/로봇 id를 바꾸려면 여기 기본값 또는 환경변수로.
"""
from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass(frozen=True)
class Settings:
    host: str = os.getenv("HMI_HOST", "0.0.0.0")
    port: int = int(os.getenv("HMI_PORT", "8000"))
    db_path: str = os.getenv("HMI_DB_PATH", "hmi.db")
    # CORS 허용 오리진 (Vite dev 서버)
    cors_origins: tuple[str, ...] = (
        os.getenv("HMI_CORS_ORIGIN", "http://localhost:5173"),
    )
    # 브로드캐스트 큐 최대 길이 (초과 시 오래된 것부터 폐기 = 링버퍼)
    queue_maxsize: int = int(os.getenv("HMI_QUEUE_MAXSIZE", "1000"))


    # 자유 클릭 내비게이션 맵(Nav2 가 실제로 쓰는 맵과 동일해야 amcl_pose/goal 좌표가 일치).
    # 13/16번 Isaac 스크립트의 Nav2 런치가 쓰는 modified_hospital_map.yaml 을 기본값으로.
    map_yaml_path: str = os.getenv(
        "HMI_MAP_YAML",
        "/home/rokey/IsaacSim-ros_workspaces/humble_ws/src/navigation/"
        "carter_navigation/maps/map/modified_hospital_map.yaml",
    )


settings = Settings()

# 로봇 식별자 — 프론트 constants/robots.ts의 ROBOT_IDS와 반드시 일치시킬 것
# (process_state/command 채널용. HMI 색/라벨 규칙: waste=red, disinfect=blue)
ROBOT_IDS: tuple[str, ...] = ("waste", "disinfect")

# 자유 클릭 내비게이션 전용 식별자 = ROS 네임스페이스(carter1/carter2).
# 매핑: carter1=소독(disinfect, blue), carter2=폐기물(waste, red).
# amcl_pose 구독 / goal_pose 발행 / navigate 엔드포인트는 이 id 로만 동작한다.
CARTER_IDS: tuple[str, ...] = ("carter1", "carter2")
