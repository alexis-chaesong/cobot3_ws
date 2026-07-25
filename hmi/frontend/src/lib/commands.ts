// ══════════════════════════════════════════════════════════════
// commands.ts — auto-dump-bot의 api/api_manager.py(AdminAPI) 포팅.
// 역할별 명령 함수 모음. apiClient를 감싸는 얇은 레이어.
// 🔧 튜닝: 새 명령 엔드포인트가 생기면 여기에 함수만 추가.
// ══════════════════════════════════════════════════════════════

import type { CarterId, RobotId } from "../types";
import { apiClient } from "./apiClient";
import { MOCK } from "./mock/mockSocket";

// 목 모드에서는 실제 fetch 대신 콘솔 로깅만 (백엔드 없이 버튼 동작 확인용)
function logMock(name: string, arg?: unknown) {
  // eslint-disable-next-line no-console
  console.info(`[MOCK] command → ${name}`, arg ?? "");
}

export const commands = {
  /** 통합 시작 — 전체 로봇 START */
  startAll: () =>
    MOCK
      ? (logMock("startAll"), Promise.resolve(null))
      : apiClient.post("/api/commands/start-all"),

  /** 개별 로봇 시작 */
  startRobot: (robotId: RobotId) =>
    MOCK
      ? (logMock("startRobot", robotId), Promise.resolve(null))
      : apiClient.post(`/api/commands/start/${robotId}`),

  /** 긴급정지. robotId 없으면 전체 긴급정지 (body robot_id: null). */
  estop: (robotId?: RobotId) =>
    MOCK
      ? (logMock("estop", robotId ?? "ALL"), Promise.resolve(null))
      : apiClient.post("/api/commands/estop", { robot_id: robotId ?? null }),

  /** 자유 클릭 내비게이션 — carter1/carter2 를 (x,y,yaw) 로 이동. "이 위치로 이동" 확인 후에만 호출. */
  navigate: (robotId: CarterId, x: number, y: number, yaw = 0) =>
    MOCK
      ? (logMock("navigate", { robotId, x, y, yaw }), Promise.resolve(null))
      : apiClient.post(`/api/commands/navigate/${robotId}`, { x, y, yaw }),
};
