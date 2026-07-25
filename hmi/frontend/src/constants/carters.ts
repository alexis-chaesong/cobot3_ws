// 자유 클릭 내비게이션 전용 로봇 식별자(carter1/carter2) 메타데이터.
// RobotId(waste/disinfect)와는 별개 체계 — 매핑: carter1=소독(disinfect,blue), carter2=폐기물(waste,red).
// 🔧 튜닝: 도킹 스테이션 색 규칙과 동일(waste=red, disinfect=blue). tokens.css 의 --waste-accent/--disinfect-accent 재사용.

import type { CarterId } from "../types";

export const CARTER_IDS: readonly CarterId[] = ["carter1", "carter2"] as const;

interface CarterMeta {
  id: CarterId;
  label: string;
  variant: "waste" | "disinfect"; // CSS 색 분기 키(tokens.css의 --{variant}-accent)
}

export const CARTER_META: Record<CarterId, CarterMeta> = {
  carter1: { id: "carter1", label: "소독 로봇 (carter1)", variant: "disinfect" },
  carter2: { id: "carter2", label: "폐기물 로봇 (carter2)", variant: "waste" },
};
