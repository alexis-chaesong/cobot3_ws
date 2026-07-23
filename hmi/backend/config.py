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


settings = Settings()

# 로봇 식별자 — 프론트 constants/robots.ts의 ROBOT_IDS와 반드시 일치시킬 것
ROBOT_IDS: tuple[str, ...] = ("waste", "disinfect")
