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

export const DISINFECT_STEPS = [
  "대기",
  "복도 진입",
  "노즐 접촉",
  "소독 분사",
  "유턴 재분사",
  "복귀",
] as const;

// robotId -> 단계 배열 매핑 (컴포넌트에서 조회용)
export const STEPS_BY_ROBOT: Record<RobotId, readonly string[]> = {
  waste: WASTE_STEPS,
  disinfect: DISINFECT_STEPS,
};
