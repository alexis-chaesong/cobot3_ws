// [HMI v2] 물리 로봇 식별자(carter1/carter2) — 유일한 로봇 식별체계.
// 기존 hmi/frontend 는 robots.ts(RobotId="waste"|"disinfect", 역할고정)와 carters.ts
// (CarterId="carter1"|"carter2", 자유클릭내비 전용)가 별개 체계였다. 19_ 은 역할고정이
// 없으므로(로봇이 런타임에 task 를 스스로 고름) carter1/carter2 하나로 통일한다.
// 색상(variant)은 더 이상 로봇에 고정되지 않는다 — 현재 배정된 task 에서 유도한다(tasks.ts 참고).
// 🔧 튜닝: 로봇을 늘리려면 CARTER_IDS에 id를 추가하고 CARTER_META 에 항목 추가.

import type { CarterId } from "../types";

export const CARTER_IDS: readonly CarterId[] = ["carter1", "carter2"] as const;

interface CarterMeta {
  id: CarterId;
  label: string; // 화면 표시명
  dock: { x: number; y: number }; // 도킹스테이션 좌표(map 프레임) — 19_ HOME1/2_XY 와 일치, 지도 마커용
}

// ★도킹스테이션 좌표★ : 19_ 의 C1/C2_START_POSE(=HOME1/2_XY, 작업 완료·복귀 지점)와 동일해야 한다.
//   carter1=(18.5, 0.2317), carter2=(16.6629..., 0.2287...). 스폰/도킹 좌표가 바뀌면 여기도 맞출 것.
export const CARTER_META: Record<CarterId, CarterMeta> = {
  carter1: { id: "carter1", label: "로봇 1 (carter1)", dock: { x: 18.5, y: 0.2317 } },
  carter2: { id: "carter2", label: "로봇 2 (carter2)", dock: { x: 16.66290495232035, y: 0.2287482072408726 } },
};
