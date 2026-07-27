// [HMI v2 신규] 작업(task) 식별자 및 메타데이터 — "trash"|"spray". 색상(variant)은 기존
// 역할고정 시절의 색 관례를 그대로 물려받는다(trash 는 기존 폐기물 로봇=red, spray 는 기존
// 소독 로봇=blue) — tokens.css 의 --waste-accent/--disinfect-accent CSS 변수명은 안 건드리고
// 그대로 재사용(FlowStepRail/QueuePanel/MapPanel 등 variant 소비처 무변경).
// 🔧 튜닝: 작업 종류를 늘리려면 TASK_IDS에 id를 추가하고 TASK_META/steps.ts에 항목 추가.

import type { TaskId } from "../types";

export const TASK_IDS: readonly TaskId[] = ["trash", "spray"] as const;

interface TaskMeta {
  id: TaskId;
  label: string; // 화면 표시명(작업선택 버튼 라벨로도 사용)
  variant: "waste" | "disinfect"; // CSS 색 분기 키(tokens.css의 --{variant}-accent) — 이름은 레거시
}

export const TASK_META: Record<TaskId, TaskMeta> = {
  trash: { id: "trash", label: "폐기물 수거", variant: "waste" },
  spray: { id: "spray", label: "소독 분사", variant: "disinfect" },
};
