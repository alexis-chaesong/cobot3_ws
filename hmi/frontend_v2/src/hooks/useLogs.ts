// ══════════════════════════════════════════════════════════════
// useLogs — [HMI v2] 로그/이력 조회. 목 모드면 fixtures, 아니면 REST GET /api/history.
// 🔧 튜닝: 폴링 주기(POLL_MS), robotId 필터, limit.
// ══════════════════════════════════════════════════════════════

import { useEffect, useState } from "react";
import { apiClient } from "../lib/apiClient";
import { MOCK } from "../lib/mock/mockSocket";
import { MOCK_LOGS } from "../lib/mock/fixtures";
import { isBeforeHistoryReset } from "../lib/historyResetMarker";
import { useHistoryResetMarker } from "./useHistoryResetMarker";
import type { CarterId, LogEntry } from "../types";

const POLL_MS = 5000;

interface TaskHistoryRow {
  task_id: string;
  robot_id: CarterId;
  start_time: string | null;
  end_time: string | null;
  status: string;
}

export function useLogs(robotId?: CarterId, limit = 50): LogEntry[] {
  const [rows, setRows] = useState<TaskHistoryRow[]>([]);
  useHistoryResetMarker(); // 마커 변경 시 리렌더 트리거 → 아래 필터를 즉시 다시 적용

  useEffect(() => {
    if (MOCK) return; // 목 모드는 정적 데이터 사용

    let alive = true;
    const query = new URLSearchParams();
    if (robotId) query.set("robot_id", robotId);
    query.set("limit", String(limit));

    const fetchLogs = async () => {
      // tb_task_history 컬럼(task_id/robot_id/start_time/...)은 LogEntry 필드(id/robotId/
      // message/level/timestamp)와 이름이 달라 그냥 스프레드하면 robotId 가 undefined 로
      // 남는다 — 그 상태로 LogPanel 이 CARTER_META[undefined] 를 읽어 크래시(백지 화면)했음.
      // 여기서 직접 필드를 매핑한다(매핑 자체는 아래 return 에서, 원본 행만 상태로 보관).
      const data = await apiClient.get<TaskHistoryRow[]>(`/api/history?${query}`);
      if (alive && data) setRows(data);
    };

    fetchLogs();
    const t = setInterval(fetchLogs, POLL_MS);
    return () => {
      alive = false;
      clearInterval(t);
    };
  }, [robotId, limit]);

  if (MOCK) return MOCK_LOGS;

  // [HMI v2 신규] 두 로봇 다 대기 상태가 된 시점(historyResetMarker) 이전 행은 화면에서
  // 숨긴다(DB 는 보존 — RobotStatusContext.tsx 의 전체-idle 전환 감지 참고).
  return rows
    .filter((row) => !isBeforeHistoryReset(row.end_time ?? row.start_time))
    .map((row): LogEntry => ({
      kind: "robot",
      id: row.task_id,
      robotId: row.robot_id,
      level: "info",
      message: row.end_time ? `자동 임무 완료 (${row.status})` : "자동 임무 진행 중",
      timestamp: row.end_time ?? row.start_time ?? new Date().toISOString(),
    }));
}
