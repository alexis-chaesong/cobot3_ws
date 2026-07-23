// 로봇 식별자 및 메타데이터. 로봇을 추가/변경할 때 여기와 steps.ts만 손대면 된다.
// 🔧 튜닝: 로봇을 늘리려면 ROBOT_IDS에 id를 추가하고 ROBOT_META/steps.ts에 항목 추가.

import type { RobotId } from "../types";

export const ROBOT_IDS = ["waste", "disinfect"] as const;

interface RobotMeta {
  id: RobotId;
  label: string; // 화면 표시명
  variant: "waste" | "disinfect"; // CSS 색 분기 키 (tokens.css의 --{variant}-accent)
}

export const ROBOT_META: Record<RobotId, RobotMeta> = {
  waste: { id: "waste", label: "폐기물 수거 로봇", variant: "waste" },
  disinfect: { id: "disinfect", label: "소독 로봇", variant: "disinfect" },
};
