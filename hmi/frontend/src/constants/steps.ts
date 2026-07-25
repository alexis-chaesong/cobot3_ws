// 플로우차트 기준으로 확정된 로봇별 단계 배열.
// 🔧 튜닝: 플로우가 바뀌면 이 배열만 수정. FlowStepRail이 자동으로 단계 수에 맞춰 렌더한다.

import type { RobotId } from "../types";

export const WASTE_STEPS = [
  "대기",
  "전방 주행",
  "폐기물통 파지",
  "수거함 이동",
  "폐기물 투하",
  "수거통 원위치",
  "복귀",
] as const;

// 16번 dual-SG : carter1 은 먼저 거치대에서 노즐을 "접촉→장착(툴체인지)"한 뒤 복도로 진입해
// 소독한다. 따라서 물리 순서 = 노즐 접촉 → 노즐 장착 → 복도 진입 → 소독 분사 → 유턴 재분사 → 복귀.
// (spray_waypoint_mission 의 dock_first=True 발행 라벨과 1:1 매칭. dock_first=False[13번]에서는
//  복도 진입이 먼저지만, 현재 운용은 16번 기준이라 이 순서를 채택.)
export const DISINFECT_STEPS = [
  "대기",
  "노즐 접촉",
  "노즐 장착",
  "복도 진입",
  "소독 분사",
  "복도 진입", // 2차 벽면으로의 이동(같은 라벨). resolveStepIndex 가 '앞으로' 매칭해 뒤로 안 감.
  "유턴 재분사",
  "복귀",
] as const;

// robotId -> 단계 배열 매핑 (컴포넌트에서 조회용)
export const STEPS_BY_ROBOT: Record<RobotId, readonly string[]> = {
  waste: WASTE_STEPS,
  disinfect: DISINFECT_STEPS,
};
