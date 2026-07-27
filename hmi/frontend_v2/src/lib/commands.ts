// ══════════════════════════════════════════════════════════════
// commands.ts — [HMI v2] carter1/carter2 단일 식별체계 + 작업선택(task_select) 명령.
// apiClient를 감싸는 얇은 레이어. 관리자가 각 명령을 "선택"하는 시점에 uiActionLog 에
// 기록한다(MOCK/실모드 공통, 백엔드 응답을 기다리지 않고 클릭 즉시 — 로그/이력 패널의
// 관리자 액션 기록용).
// 🔧 튜닝: 새 명령 엔드포인트가 생기면 여기에 함수만 추가.
// ══════════════════════════════════════════════════════════════

import { CARTER_META } from "../constants/carters";
import { TASK_META } from "../constants/tasks";
import type { CarterId, TaskId } from "../types";
import { apiClient } from "./apiClient";
import { MOCK, setMockTask } from "./mock/mockSocket";
import { logUiAction } from "./uiActionLog";

// [HMI v2] 백엔드 config.py 의 DEFAULT_TASK_ASSIGNMENT 와 동일 기본 조합(기존 관행 유지).
const DEFAULT_TASK_ASSIGNMENT: Record<CarterId, TaskId> = { carter1: "spray", carter2: "trash" };

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
  /**
   * [HMI v2 재해석] "통합 시작" — task-select 모델엔 별도 시작 게이트가 없으므로(작업선택
   * 자체가 시작 트리거), 두 로봇에 기본 작업 조합을 한번에 발행하는 걸로 재해석
   * (백엔드가 DEFAULT_TASK_ASSIGNMENT 를 적용 — 기존 관행 그대로 carter1=spray/carter2=trash).
   */
  startAll: () => {
    logUiAction("전체", "통합 시작(기본 작업 배정)을 선택함");
    if (MOCK) {
      for (const [carterId, task] of Object.entries(DEFAULT_TASK_ASSIGNMENT) as [
        CarterId,
        TaskId,
      ][]) {
        setMockTask(carterId, task);
      }
      logMock("startAll", DEFAULT_TASK_ASSIGNMENT);
      return Promise.resolve(null);
    }
    return apiClient.post("/api/commands/start-all");
  },

  /** [HMI v2 신규] 로봇 하나에 작업(trash|spray)을 배정 — 클릭=배정=시작. */
  selectTask: (carterId: CarterId, task: TaskId) => {
    logUiAction(
      CARTER_META[carterId].label,
      `${TASK_META[task].label} 작업을 선택함`,
    );
    if (MOCK) {
      setMockTask(carterId, task);
      logMock("selectTask", { carterId, task });
      return Promise.resolve(null);
    }
    return apiClient.post(`/api/commands/task/${carterId}`, { task });
  },

  /** 긴급정지. carterId 없으면 전체 긴급정지 (body robot_id: null). */
  estop: (carterId?: CarterId) => {
    logUiAction(carterId ? CARTER_META[carterId].label : "전체", "긴급정지를 선택함");
    return MOCK
      ? (logMock("estop", carterId ?? "ALL"), Promise.resolve(null))
      : apiClient.post("/api/commands/estop", { robot_id: carterId ?? null });
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
