// ══════════════════════════════════════════════════════════════
// fixtures.ts — 목 모드용 정적 데이터 (로그).
// 큐(MOCK_QUEUE)는 더 이상 정적 데이터가 아님 — useQueueStatus 가 routeQueue.ts(웨이포인트
// 경로, mock/실모드 공통 로직)에서 실시간으로 유도한다.
// 🔧 튜닝: 화면 확인용 더미 데이터. 개수/내용을 자유롭게 바꿔도 됩니다.
// ══════════════════════════════════════════════════════════════

import type { LogEntry } from "../../types";

export const MOCK_LOGS: LogEntry[] = [
  { kind: "robot", id: "l1", robotId: "waste", level: "info", message: "폐기물 수거 로봇 START", timestamp: "2026-07-24T05:10:02Z" },
  { kind: "robot", id: "l2", robotId: "disinfect", level: "info", message: "소독 로봇 START", timestamp: "2026-07-24T05:10:05Z" },
  { kind: "robot", id: "l3", robotId: "waste", level: "info", message: "전방 주행 진입", timestamp: "2026-07-24T05:10:40Z" },
  { kind: "robot", id: "l4", robotId: "disinfect", level: "error", message: "ERR_COLLISION: 노즐 접촉 중 장애물 감지", timestamp: "2026-07-24T05:11:12Z" },
  { kind: "robot", id: "l5", robotId: "waste", level: "info", message: "폐기물통 파지 완료", timestamp: "2026-07-24T05:11:30Z" },
];
