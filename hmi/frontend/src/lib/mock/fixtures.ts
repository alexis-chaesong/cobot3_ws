// ══════════════════════════════════════════════════════════════
// fixtures.ts — 목 모드용 정적 데이터 (로그/큐).
// 🔧 튜닝: 화면 확인용 더미 데이터. 개수/내용을 자유롭게 바꿔도 됩니다.
// ══════════════════════════════════════════════════════════════

import type { LogEntry, QueueItem } from "../../types";

export const MOCK_LOGS: LogEntry[] = [
  { id: "l1", robotId: "waste", level: "info", message: "폐기물 수거 로봇 START", timestamp: "2026-07-24T05:10:02Z" },
  { id: "l2", robotId: "disinfect", level: "info", message: "소독 로봇 START", timestamp: "2026-07-24T05:10:05Z" },
  { id: "l3", robotId: "waste", level: "info", message: "전방 주행 진입", timestamp: "2026-07-24T05:10:40Z" },
  { id: "l4", robotId: "disinfect", level: "error", message: "ERR_COLLISION: 노즐 접촉 중 장애물 감지", timestamp: "2026-07-24T05:11:12Z" },
  { id: "l5", robotId: "waste", level: "info", message: "폐기물통 파지 완료", timestamp: "2026-07-24T05:11:30Z" },
];

export const MOCK_QUEUE: QueueItem[] = [
  { taskId: "T-1024", robotId: "waste", label: "301호 폐기물 수거", status: "active" },
  { taskId: "T-1025", robotId: "disinfect", label: "복도 A 소독", status: "active" },
  { taskId: "T-1026", robotId: "waste", label: "302호 폐기물 수거", status: "queued" },
  { taskId: "T-1027", robotId: "disinfect", label: "복도 B 소독", status: "queued" },
];
