"""sqlite3 raw 연결 (ORM 없음, auto-dump-bot 패턴 그대로).

tb_dump_modes 같은 '모드' 테이블은 없다. 이력/에러 두 테이블만 둔다.
"""
from __future__ import annotations

import sqlite3
from typing import Any

from config import settings


def get_connection() -> sqlite3.Connection:
    conn = sqlite3.connect(settings.db_path, check_same_thread=False)
    conn.row_factory = sqlite3.Row  # dict 변환용 (이전과 동일)
    return conn


def init_db() -> None:
    """앱 시작 시 테이블 생성."""
    conn = get_connection()
    try:
        conn.executescript(
            """
            CREATE TABLE IF NOT EXISTS tb_task_history (
                task_id    TEXT PRIMARY KEY,
                robot_id   TEXT NOT NULL,
                start_time TEXT,
                end_time   TEXT,
                status     TEXT
            );

            CREATE TABLE IF NOT EXISTS tb_error_log (
                error_id   INTEGER PRIMARY KEY AUTOINCREMENT,
                robot_id   TEXT NOT NULL,
                error_code TEXT,
                error_msg  TEXT,
                error_time TEXT
            );
            """
        )
        conn.commit()
    finally:
        conn.close()


def rows_to_dicts(rows: list[sqlite3.Row]) -> list[dict[str, Any]]:
    """sqlite3.Row -> dict 리스트 (이전 프로젝트와 동일 변환)."""
    return [dict(row) for row in rows]
