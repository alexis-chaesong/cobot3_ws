"""환경변수 중앙관리 — frozen dataclass (auto-dump-bot 패턴 그대로).

[HMI v2] 기존 hmi/backend 의 ROBOT_IDS("waste"/"disinfect")+CARTER_IDS("carter1"/"carter2")
이원 체계를 CARTER_IDS 하나로 통일(19_ 작업선택식 대응 — 로봇이 역할 고정이 아니라 런타임에
task(trash|spray)를 배정받으므로, "역할"이 아니라 "물리 로봇"이 유일한 식별자여야 한다).
기존 hmi/backend(포트 8000, hmi.db)는 16_/18_ 용으로 그대로 두고 병행 실행 — 포트/DB/CORS
기본값을 8001/hmi_v2.db/5174 로 분리했다.

🔧 튜닝: 포트/DB 경로/로봇 id를 바꾸려면 여기 기본값 또는 환경변수로.
"""
from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass(frozen=True)
class Settings:
    host: str = os.getenv("HMI_HOST", "0.0.0.0")
    port: int = int(os.getenv("HMI_PORT", "8001"))
    db_path: str = os.getenv("HMI_DB_PATH", "hmi_v2.db")
    # CORS 허용 오리진 (Vite dev 서버, v2 프론트 기본 포트 5174)
    cors_origins: tuple[str, ...] = (
        os.getenv("HMI_CORS_ORIGIN", "http://localhost:5174"),
    )
    # 브로드캐스트 큐 최대 길이 (초과 시 오래된 것부터 폐기 = 링버퍼)
    queue_maxsize: int = int(os.getenv("HMI_QUEUE_MAXSIZE", "1000"))

    # 자유 클릭 내비게이션 맵(Nav2 가 실제로 쓰는 맵과 동일해야 amcl_pose/goal 좌표가 일치).
    # run_nav.sh 가 쓰는 modified_hospital_2_map.yaml 을 기본값으로(기존 backend 와 동일 맵).
    map_yaml_path: str = os.getenv(
        "HMI_MAP_YAML",
        "/home/rokey/IsaacSim-ros_workspaces/humble_ws/src/navigation/"
        "carter_navigation/maps/map/modified_hospital_2_map.yaml",
    )


settings = Settings()

# [HMI v2] 로봇 식별자 = 물리 로봇(carter1/carter2) 하나로 통일 — 기존 ROBOT_IDS(역할고정
# waste/disinfect)는 폐기. process_state/safety_event/command/task_select/pose/tf/vision/goal
# 전부 이 하나의 id 공간으로 동작한다. 프론트 constants/carters.ts 의 CARTER_IDS 와 반드시 일치.
CARTER_IDS: tuple[str, ...] = ("carter1", "carter2")

# [HMI v2] 통합 시작("전체 로봇에 기본 조합 발행") 시 쓰는 기본 작업 배정 — 기존 역할고정
# 관행(disinfect=carter1/waste=carter2)을 기본값으로 유지(사용자 결정, 완전히 자유배정 가능하나
# "통합 시작" 버튼의 기본 동작만 이 조합).
DEFAULT_TASK_ASSIGNMENT: dict[str, str] = {"carter1": "spray", "carter2": "trash"}
