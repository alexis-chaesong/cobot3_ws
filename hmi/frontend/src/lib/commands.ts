// ══════════════════════════════════════════════════════════════
// commands.ts — auto-dump-bot의 api/api_manager.py(AdminAPI) 포팅.
// 역할별 명령 함수 모음. apiClient를 감싸는 얇은 레이어.
// 관리자가 각 명령을 "선택"하는 시점에 uiActionLog 에 기록한다(MOCK/실모드 공통,
// 백엔드 응답을 기다리지 않고 클릭 즉시 — 로그/이력 패널의 관리자 액션 기록용).
// 🔧 튜닝: 새 명령 엔드포인트가 생기면 여기에 함수만 추가.
// ══════════════════════════════════════════════════════════════

import { CARTER_META } from "../constants/carters";
import { ROBOT_META } from "../constants/robots";
import type { CarterId, RobotId } from "../types";
import { apiClient } from "./apiClient";
import { MOCK } from "./mock/mockSocket";
import { logUiAction } from "./uiActionLog";

// 목 모드에서는 실제 fetch 대신 콘솔 로깅만 (백엔드 없이 버튼 동작 확인용)
function logMock(name: string, arg?: unknown) {
  // eslint-disable-next-line no-console
  console.info(`[MOCK] command → ${name}`, arg ?? "");
}

/**
 * 실제 goal_pose 발행만(로그 없음) — 단발성 자유클릭은 commands.navigate 가 감싸 쓰고,
 * 웨이포인트 경로 자동진행(routeQueue.ts)의 각 구간 발행은 이 함수로 직접 호출한다.
 * (관리자가 "경로 시작"을 선택한 게 기록할 선택이지, 자동 다음-구간 발행은 새 선택이 아님.)
 */
export function sendNavGoal(robotId: CarterId, x: number, y: number, yaw = 0) {
  return MOCK
    ? (logMock("navigate", { robotId, x, y, yaw }), Promise.resolve(null))
    : apiClient.post(`/api/commands/navigate/${robotId}`, { x, y, yaw });
}

export const commands = {
  /** 통합 시작 — 전체 로봇 START */
  startAll: () => {
    logUiAction("전체", "통합 시작을 선택함");
    return MOCK
      ? (logMock("startAll"), Promise.resolve(null))
      : apiClient.post("/api/commands/start-all");
  },

  /** 개별 로봇 시작 */
  startRobot: (robotId: RobotId) => {
    logUiAction(ROBOT_META[robotId].label, "개별 시작을 선택함");
    return MOCK
      ? (logMock("startRobot", robotId), Promise.resolve(null))
      : apiClient.post(`/api/commands/start/${robotId}`);
  },

  /** 긴급정지. robotId 없으면 전체 긴급정지 (body robot_id: null). */
  estop: (robotId?: RobotId) => {
    logUiAction(robotId ? ROBOT_META[robotId].label : "전체", "긴급정지를 선택함");
    return MOCK
      ? (logMock("estop", robotId ?? "ALL"), Promise.resolve(null))
      : apiClient.post("/api/commands/estop", { robot_id: robotId ?? null });
  },

  /** 자유 클릭 내비게이션(단발) — carter1/carter2 를 (x,y,yaw) 로 이동. "이 위치로 이동" 확인 후에만 호출. */
  navigate: (robotId: CarterId, x: number, y: number, yaw = 0) => {
    logUiAction(
      CARTER_META[robotId].label,
      `자유 클릭 이동을 선택함 (${x.toFixed(2)}, ${y.toFixed(2)})`,
    );
    return sendNavGoal(robotId, x, y, yaw);
  },
};
