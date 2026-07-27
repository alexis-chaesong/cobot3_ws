// [HMI v2] 작업(task)별 단계 배열 — 기존 로봇별(WASTE_STEPS/DISINFECT_STEPS) 배열을
// task 기준(TRASH_STEPS/SPRAY_STEPS)으로 재사용. 어느 로봇이 이 작업을 하든 동일 배열을 쓴다.
// 19_dual_task_select_yolo_integrated.py 의 publish_hmi_state() 호출 라벨과 1:1 대응
// (HMI v2 계획 §3 표 참고) — ⚠ 라이브 [HB] 로그와 대조 확인 필요.
// 🔧 튜닝: 플로우가 바뀌면 이 배열만 수정. FlowStepRail이 자동으로 단계 수에 맞춰 렌더한다.

import type { TaskId } from "../types";

export const TRASH_STEPS = [
  "대기",
  "전방 주행",
  "폐기물통 파지",
  "수거함 이동",
  "폐기물 투하",
  "수거통 원위치",
  "복귀",
] as const;

// 19_ 의 g_spray_mission_body 는 공용 분사 웨이포인트 1곳을 왕복만 한다(16번의 다중 웨이포인트
// 반복 스윕과 달리 "유턴 재분사"/2차 "복도 진입" 없음) — 기존 DISINFECT_STEPS 에서 그 두 단계를
// 제거했다.
export const SPRAY_STEPS = [
  "대기",
  "노즐 접촉",
  "노즐 장착",
  "복도 진입",
  "소독 분사",
  "복귀",
] as const;

// taskId -> 단계 배열 매핑 (컴포넌트에서 조회용)
export const STEPS_BY_TASK: Record<TaskId, readonly string[]> = {
  trash: TRASH_STEPS,
  spray: SPRAY_STEPS,
};
