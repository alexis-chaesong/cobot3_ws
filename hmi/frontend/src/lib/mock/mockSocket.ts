// ══════════════════════════════════════════════════════════════
// mockSocket.ts — 백엔드 없이 WebSocket 브로드캐스트를 흉내낸다.
// 두 로봇이 각자의 steps 배열을 따라 진행하며 ROBOT_STATUS를 방출하고,
// 데모용으로 소독 로봇에서 한 번 SAFETY_EVENT를 발생시켜 AlertBanner를 확인시킨다.
//
// 이건 백엔드의 mock_robot_node.py에 대응하는 "프론트 목 모드"이다.
// 🔧 튜닝:
//   - TICK_MS: 단계 전환 속도
//   - SAFETY_DEMO: 데모 안전 이벤트를 끄려면 false
// ══════════════════════════════════════════════════════════════

import { WASTE_STEPS, DISINFECT_STEPS } from "../../constants/steps";
import type { RobotId, WsMessage } from "../../types";

/** VITE_MOCK 플래그 (문자열 "true" 비교). 전역에서 이 상수로 분기한다. */
export const MOCK = import.meta.env.VITE_MOCK === "true";

const TICK_MS = 2500;
const SAFETY_DEMO = true;

const STEPS: Record<RobotId, readonly string[]> = {
  waste: WASTE_STEPS,
  disinfect: DISINFECT_STEPS,
};

/**
 * apiClient.connectWebSocket과 동일한 시그니처.
 * onMessage로 가짜 메시지를 주기적으로 흘려보내고, cleanup 함수를 반환한다.
 */
export function connectMockSocket(
  onMessage: (data: WsMessage) => void,
): () => void {
  const idx: Record<RobotId, number> = { waste: 0, disinfect: 0 };
  let ticks = 0;

  const emitStatus = (robotId: RobotId) => {
    const steps = STEPS[robotId];
    const i = idx[robotId];
    const label = steps[i];
    onMessage({
      type: "ROBOT_STATUS",
      robotId,
      payload: {
        process_state: i === 0 ? label : `${label} 중`,
        recovery_stage: "IDLE",
      },
      timestamp: new Date().toISOString(),
    });
    idx[robotId] = (i + 1) % steps.length; // 끝나면 다시 대기부터 반복
  };

  const interval = setInterval(() => {
    ticks += 1;
    emitStatus("waste");
    emitStatus("disinfect");

    // 데모: 4틱째에 소독 로봇 안전 이벤트 1회 발생 → AlertBanner 확인
    if (SAFETY_DEMO && ticks === 4) {
      onMessage({
        type: "SAFETY_EVENT",
        robotId: "disinfect",
        error_code: "ERR_COLLISION",
        error_msg: "노즐 접촉 중 예상치 못한 장애물 감지",
        timestamp: new Date().toISOString(),
      });
    }
  }, TICK_MS);

  // 첫 프레임 즉시 방출 (초기 화면이 비지 않도록)
  emitStatus("waste");
  emitStatus("disinfect");

  return () => clearInterval(interval);
}
