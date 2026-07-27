// ══════════════════════════════════════════════════════════════
// useTaskQueue — [HMI v2] 로봇 자동 임무 큐. GET /api/queue(tb_task_history, robot_bridge.py 의
// _track_task_history 가 process_state "대기"↔"RUNNING:..." 전환마다 채움)를 폴링해
// 로봇별 진행 중 작업 1건 + 최근 완료 작업 몇 건을 QueueItem[] 로 변환한다.
// routeQueue(관리자가 지도에서 만든 수동 경로예약)와는 별개 소스 — useQueueStatus 가 합친다.
//
// [HMI v2 알려진 제약] tb_task_history 스키마엔 task(trash|spray) 컬럼이 없어(단계 라벨
// 세분화 안 함, 기존 설계 그대로 유지) 완료(DONE)된 과거 행은 그 작업이 trash 였는지 spray
// 였는지 알 수 없다 — 진행 중(RUNNING) 행만 RobotStatusContext 의 현재 task 로 정확한 색을
// 표시하고, 완료 행은 중립 기본값(waste)으로 폴백한다(색만 부정확, 라벨/상태는 정확).
// 🔧 튜닝: POLL_MS(폴링 주기).
// ══════════════════════════════════════════════════════════════

import { useEffect, useState } from "react";
import { apiClient } from "../lib/apiClient";
import { MOCK } from "../lib/mock/mockSocket";
import { CARTER_META } from "../constants/carters";
import { TASK_META } from "../constants/tasks";
import { useRobotStatusContext } from "../context/RobotStatusContext";
import { isBeforeHistoryReset } from "../lib/historyResetMarker";
import { useHistoryResetMarker } from "./useHistoryResetMarker";
import type { CarterId, QueueItem } from "../types";

const POLL_MS = 3000;

interface TaskRow {
  task_id: string;
  robot_id: CarterId;
  start_time: string | null;
  end_time: string | null;
  status: "RUNNING" | "DONE";
}

function fmtTime(iso: string | null): string {
  if (!iso) return "";
  const d = new Date(iso);
  return Number.isNaN(d.getTime()) ? "" : d.toLocaleTimeString("ko-KR", { hour12: false });
}

const MOCK_TASKS: QueueItem[] = [
  {
    taskId: "mock-carter1-running",
    variant: "disinfect",
    label: "로봇 1 (carter1) · 자동 임무 진행 중 (09:12~)",
    status: "active",
  },
  {
    taskId: "mock-carter2-done",
    variant: "waste",
    label: "로봇 2 (carter2) · 자동 임무 완료 (08:50~09:00)",
    status: "done",
  },
];

export function useTaskQueue(): QueueItem[] {
  const { snapshots } = useRobotStatusContext();
  const [rows, setRows] = useState<TaskRow[]>([]);
  useHistoryResetMarker(); // 마커 변경 시 리렌더만 트리거(값 자체는 isBeforeHistoryReset 이 읽음)

  useEffect(() => {
    if (MOCK) return;
    let cancelled = false;

    const poll = async () => {
      const fetched = await apiClient.get<TaskRow[]>("/api/queue");
      if (cancelled || !fetched) return;
      setRows(fetched);
    };
    poll();
    const timer = setInterval(poll, POLL_MS);
    return () => {
      cancelled = true;
      clearInterval(timer);
    };
  }, []);

  if (MOCK) return MOCK_TASKS;

  return rows
    .filter((row) => !isBeforeHistoryReset(row.end_time ?? row.start_time))
    .map((row) => {
    const meta = CARTER_META[row.robot_id];
    const currentTask = snapshots[row.robot_id]?.task;
    const variant = row.status === "RUNNING" && currentTask
      ? TASK_META[currentTask].variant
      : "waste"; // DONE 행은 과거 task 를 모름 — 중립 기본값 폴백(위 주석 참고)
    const label =
      row.status === "RUNNING"
        ? `${meta.label} · 자동 임무 진행 중 (${fmtTime(row.start_time)}~)`
        : `${meta.label} · 자동 임무 완료 (${fmtTime(row.start_time)}~${fmtTime(row.end_time)})`;
    return {
      taskId: row.task_id,
      variant,
      label,
      status: row.status === "RUNNING" ? "active" : "done",
    };
  });
}
