// ══════════════════════════════════════════════════════════════
// useLogs — 로그/이력 조회. 목 모드면 fixtures, 아니면 REST GET /api/history.
// 🔧 튜닝: 폴링 주기(POLL_MS), robotId 필터, limit.
// ══════════════════════════════════════════════════════════════

import { useEffect, useState } from "react";
import { apiClient } from "../lib/apiClient";
import { MOCK } from "../lib/mock/mockSocket";
import { MOCK_LOGS } from "../lib/mock/fixtures";
import type { LogEntry, RobotId } from "../types";

const POLL_MS = 5000;

export function useLogs(robotId?: RobotId, limit = 50): LogEntry[] {
  const [logs, setLogs] = useState<LogEntry[]>(MOCK ? MOCK_LOGS : []);

  useEffect(() => {
    if (MOCK) return; // 목 모드는 정적 데이터 사용

    let alive = true;
    const query = new URLSearchParams();
    if (robotId) query.set("robot_id", robotId);
    query.set("limit", String(limit));

    const fetchLogs = async () => {
      // tb_task_history 컬럼(task_id/robot_id/start_time/...)이 LogEntry 필드와
      // 정확히 일치하진 않음(기존부터의 갭, 이 훅 범위 밖) — kind만 부여해 타입 충족.
      const data = await apiClient.get<Record<string, unknown>[]>(
        `/api/history?${query}`,
      );
      if (alive && data) {
        setLogs(data.map((row) => ({ kind: "robot", ...row }) as LogEntry));
      }
    };

    fetchLogs();
    const t = setInterval(fetchLogs, POLL_MS);
    return () => {
      alive = false;
      clearInterval(t);
    };
  }, [robotId, limit]);

  return logs;
}
