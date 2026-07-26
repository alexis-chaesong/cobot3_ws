// ══════════════════════════════════════════════════════════════
// useTaskQueue — 로봇 자동 임무 큐. GET /api/queue(tb_task_history, robot_bridge.py 의
// _track_task_history 가 process_state "대기"↔"RUNNING:..." 전환마다 채움)를 폴링해
// 로봇별 진행 중 작업 1건 + 최근 완료 작업 몇 건을 QueueItem[] 로 변환한다.
// routeQueue(관리자가 지도에서 만든 수동 경로예약)와는 별개 소스 — useQueueStatus 가 합친다.
// 🔧 튜닝: POLL_MS(폴링 주기).
// ══════════════════════════════════════════════════════════════

import { useEffect, useState } from "react";
import { apiClient } from "../lib/apiClient";
import { MOCK } from "../lib/mock/mockSocket";
import { ROBOT_META } from "../constants/robots";
import type { QueueItem, RobotId } from "../types";

const POLL_MS = 3000;

interface TaskRow {
  task_id: string;
  robot_id: RobotId;
  start_time: string | null;
  end_time: string | null;
  status: "RUNNING" | "DONE";
}

function fmtTime(iso: string | null): string {
  if (!iso) return "";
  const d = new Date(iso);
  return Number.isNaN(d.getTime()) ? "" : d.toLocaleTimeString("ko-KR", { hour12: false });
}

function toQueueItem(row: TaskRow): QueueItem {
  const meta = ROBOT_META[row.robot_id];
  const label =
    row.status === "RUNNING"
      ? `${meta.label} · 자동 임무 진행 중 (${fmtTime(row.start_time)}~)`
      : `${meta.label} · 자동 임무 완료 (${fmtTime(row.start_time)}~${fmtTime(row.end_time)})`;
  return {
    taskId: row.task_id,
    variant: meta.variant,
    label,
    status: row.status === "RUNNING" ? "active" : "done",
  };
}

const MOCK_TASKS: QueueItem[] = [
  {
    taskId: "mock-disinfect-running",
    variant: "disinfect",
    label: "소독 로봇 · 자동 임무 진행 중 (09:12~)",
    status: "active",
  },
  {
    taskId: "mock-waste-done",
    variant: "waste",
    label: "폐기물 수거 로봇 · 자동 임무 완료 (08:50~09:00)",
    status: "done",
  },
];

export function useTaskQueue(): QueueItem[] {
  const [items, setItems] = useState<QueueItem[]>(MOCK ? MOCK_TASKS : []);

  useEffect(() => {
    if (MOCK) return;
    let cancelled = false;

    const poll = async () => {
      const rows = await apiClient.get<TaskRow[]>("/api/queue");
      if (cancelled || !rows) return;
      setItems(rows.map(toQueueItem));
    };
    poll();
    const timer = setInterval(poll, POLL_MS);
    return () => {
      cancelled = true;
      clearInterval(timer);
    };
  }, []);

  return items;
}
