// ══════════════════════════════════════════════════════════════
// mockSocket.ts — [HMI v2] 백엔드 없이 WebSocket 브로드캐스트를 흉내낸다.
// 19_ 의 실제 동작(작업 미배정=대기, task_select 수신 시 그 작업의 STEPS 순환 시작)을
// 최대한 비슷하게 재현 — commands.ts 의 selectTask(MOCK 모드)가 setMockTask() 를 호출하면
// 이 소켓이 그 배정을 읽어 STEPS_BY_TASK 순환을 시작한다(실제 클릭 → 화면 반영 확인용).
// 데모용으로 4틱째 carter1 에서 한 번 SAFETY_EVENT를 발생시켜 AlertBanner를 확인시킨다.
//
// 이건 백엔드의 mock_robot_node.py에 대응하는 "프론트 목 모드"이다.
// 🔧 튜닝:
//   - TICK_MS: 단계 전환 속도
//   - SAFETY_DEMO: 데모 안전 이벤트를 끄려면 false
// ══════════════════════════════════════════════════════════════

import { STEPS_BY_TASK } from "../../constants/steps";
import type { CarterId, TaskId, WsMessage } from "../../types";

/** VITE_MOCK 플래그 (문자열 "true" 비교). 전역에서 이 상수로 분기한다. */
export const MOCK = import.meta.env.VITE_MOCK === "true";

const TICK_MS = 2500;
const SAFETY_DEMO = true;

// [HMI v2 신규] 목 모드 작업 배정 — commands.ts 의 selectTask 가 여기 기록하면
// connectMockSocket 의 interval 이 다음 틱부터 그 배정을 읽어 순환을 시작한다.
const mockAssignedTask: Record<CarterId, TaskId | null> = {
  carter1: null,
  carter2: null,
};

export function setMockTask(carterId: CarterId, task: TaskId): void {
  mockAssignedTask[carterId] = task;
}

/**
 * apiClient.connectWebSocket과 동일한 시그니처.
 * onMessage로 가짜 메시지를 주기적으로 흘려보내고, cleanup 함수를 반환한다.
 */
export function connectMockSocket(
  onMessage: (data: WsMessage) => void,
): () => void {
  const idx: Record<CarterId, number> = { carter1: 0, carter2: 0 };
  const carterIds: CarterId[] = ["carter1", "carter2"];
  let ticks = 0;

  const emitStatus = (carterId: CarterId) => {
    const task = mockAssignedTask[carterId];
    if (task === null) {
      onMessage({
        type: "ROBOT_STATUS",
        robotId: carterId,
        payload: { process_state: "대기", recovery_stage: "IDLE", task: null },
        timestamp: new Date().toISOString(),
      });
      return;
    }
    const steps = STEPS_BY_TASK[task];
    const i = idx[carterId];
    const label = steps[i];
    onMessage({
      type: "ROBOT_STATUS",
      robotId: carterId,
      payload: {
        process_state: i === 0 ? label : `${label} 중`,
        recovery_stage: "IDLE",
        task,
      },
      timestamp: new Date().toISOString(),
    });
    if (label === "복귀") {
      // 마지막 단계 다음엔 다시 대기로(19_ 의 IDLE 재진입과 동일한 흐름).
      mockAssignedTask[carterId] = null;
      idx[carterId] = 0;
    } else {
      idx[carterId] = Math.min(i + 1, steps.length - 1);
    }
  };

  const interval = setInterval(() => {
    ticks += 1;
    for (const id of carterIds) emitStatus(id);

    // 데모: 4틱째에 carter1 안전 이벤트 1회 발생 → AlertBanner 확인
    if (SAFETY_DEMO && ticks === 4) {
      onMessage({
        type: "SAFETY_EVENT",
        robotId: "carter1",
        error_code: "ERR_COLLISION",
        error_msg: "소독 스윕 중 예상치 못한 장애물 감지",
        timestamp: new Date().toISOString(),
      });
    }
  }, TICK_MS);

  // 첫 프레임 즉시 방출 (초기 화면이 비지 않도록)
  for (const id of carterIds) emitStatus(id);

  return () => clearInterval(interval);
}
