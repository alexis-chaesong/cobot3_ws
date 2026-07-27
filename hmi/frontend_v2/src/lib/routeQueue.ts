// ══════════════════════════════════════════════════════════════
// routeQueue.ts — 관리자가 지도에 여러 지점을 찍어 만든 "웨이포인트 경로"를
// carter1/carter2 별로 순서대로 실행하는 모듈 싱글턴(uiActionLog.ts와 동일 pub/sub 스타일).
//
// 동작: enqueueRoute 로 시작하면 첫 구간을 즉시 발행(active)하고, 다음 신호가 오면
// 다음 구간으로 자동 전진한다.
//   - 실모드: robotPoseStore(실제 amcl_pose)로 목표점까지 거리가 ARRIVAL_TOLERANCE_M
//     이내면 "도착"으로 보고 다음 구간 발행. Nav2 액션 피드백이 없는 구조라, 혹시
//     도착 신호가 영영 안 오는 경우(경로 막힘 등)에 대비해 LEG_TIMEOUT_MS 지나면
//     강제로 다음 구간으로 넘어간다(전체 큐가 무한정 멈추는 것 방지용 안전장치).
//   - 목 모드: 실제 위치 피드백이 의미 없으므로 MOCK_LEG_MS 고정 지연 후 자동 전진.
//
// 관리자의 "선택"으로 기록하는 건 경로 시작/취소 1회씩뿐 — 자동 다음-구간 전진은
// 새 선택이 아니라 시스템 동작이라 uiActionLog 에 남기지 않는다.
// 🔧 튜닝: ARRIVAL_TOLERANCE_M / MOCK_LEG_MS / LEG_TIMEOUT_MS.
// ══════════════════════════════════════════════════════════════

import { CARTER_IDS, CARTER_META } from "../constants/carters";
import { sendNavGoal } from "./commands";
import { logUiAction } from "./uiActionLog";
import { subscribeRobotPoseStore } from "./robotPoseStore";
import { MOCK } from "./mock/mockSocket";
import type { CarterId } from "../types";

export interface RouteWaypoint {
  id: string;
  x: number;
  y: number;
  yaw: number;
  status: "queued" | "active" | "done";
}
export type RouteMap = Record<CarterId, RouteWaypoint[]>;

const ARRIVAL_TOLERANCE_M = 0.4;
const MOCK_LEG_MS = 2500;
const LEG_TIMEOUT_MS = 120_000;

function emptyRouteMap(): RouteMap {
  return CARTER_IDS.reduce((acc, id) => {
    acc[id] = [];
    return acc;
  }, {} as RouteMap);
}

let routes: RouteMap = emptyRouteMap();
type Listener = (routes: RouteMap) => void;
const listeners = new Set<Listener>();

let poseUnsub: (() => void) | null = null;
const legTimer: Record<CarterId, ReturnType<typeof setTimeout> | null> = {
  carter1: null,
  carter2: null,
};

function notify() {
  for (const listener of listeners) listener(routes);
}

function setRoute(carterId: CarterId, waypoints: RouteWaypoint[]) {
  routes = { ...routes, [carterId]: waypoints };
  notify();
}

function clearLegTimer(carterId: CarterId) {
  const t = legTimer[carterId];
  if (t) {
    clearTimeout(t);
    legTimer[carterId] = null;
  }
}

function activeIndex(carterId: CarterId): number {
  return routes[carterId].findIndex((w) => w.status === "active");
}

function ensurePoseWatch() {
  if (poseUnsub) return;
  poseUnsub = subscribeRobotPoseStore((poses) => {
    for (const carterId of CARTER_IDS) {
      const idx = activeIndex(carterId);
      if (idx < 0) continue;
      const wp = routes[carterId][idx];
      const pose = poses[carterId];
      if (!pose) continue;
      const dist = Math.hypot(pose.x - wp.x, pose.y - wp.y);
      if (dist <= ARRIVAL_TOLERANCE_M) advance(carterId);
    }
  });
}

function stopPoseWatchIfIdle() {
  const anyActive = CARTER_IDS.some((id) => activeIndex(id) >= 0);
  if (!anyActive && poseUnsub) {
    poseUnsub();
    poseUnsub = null;
  }
}

function dispatchLeg(carterId: CarterId, wp: RouteWaypoint) {
  sendNavGoal(carterId, wp.x, wp.y, wp.yaw);
  if (MOCK) {
    legTimer[carterId] = setTimeout(() => advanceIfStillActive(carterId, wp.id), MOCK_LEG_MS);
    return;
  }
  ensurePoseWatch();
  // 안전장치: 도착 신호(pose 근접)가 끝내 안 와도 전체 큐가 멈추지 않도록 강제 진행.
  legTimer[carterId] = setTimeout(
    () => advanceIfStillActive(carterId, wp.id),
    LEG_TIMEOUT_MS,
  );
}

function advanceIfStillActive(carterId: CarterId, legId: string) {
  const cur = routes[carterId].find((w) => w.id === legId);
  if (cur && cur.status === "active") advance(carterId);
}

/** 현재 active 구간을 done 처리하고, 다음 queued 구간을 active 로 승격+발행. 없으면 종료. */
function advance(carterId: CarterId) {
  clearLegTimer(carterId);
  const route = routes[carterId];
  const idx = activeIndex(carterId);
  if (idx < 0) return;
  const next = route.map((w, i) => (i === idx ? { ...w, status: "done" as const } : w));
  const nextIdx = next.findIndex((w) => w.status === "queued");
  if (nextIdx >= 0) {
    next[nextIdx] = { ...next[nextIdx], status: "active" };
    setRoute(carterId, next);
    dispatchLeg(carterId, next[nextIdx]);
  } else {
    setRoute(carterId, next);
    stopPoseWatchIfIdle();
  }
}

/** 관리자가 지도에 찍은 지점들로 경로를 시작(기존 경로가 있으면 취소하고 대체). */
export function enqueueRoute(
  carterId: CarterId,
  points: { x: number; y: number; yaw?: number }[],
): void {
  if (points.length === 0) return;
  clearLegTimer(carterId);
  const waypoints: RouteWaypoint[] = points.map((p, i) => ({
    id: `${carterId}-${Date.now()}-${i}`,
    x: p.x,
    y: p.y,
    yaw: p.yaw ?? 0,
    status: i === 0 ? "active" : "queued",
  }));
  logUiAction(
    CARTER_META[carterId].label,
    `경로 이동 시작을 선택함 (${waypoints.length}개 지점)`,
  );
  setRoute(carterId, waypoints);
  dispatchLeg(carterId, waypoints[0]);
}

/** 진행 중인 경로 취소(남은 대기 구간 전부 제거). 이미 빈 경로면 아무 것도 안 함. */
export function cancelRoute(carterId: CarterId): void {
  if (routes[carterId].length === 0) return;
  clearLegTimer(carterId);
  logUiAction(CARTER_META[carterId].label, "경로 취소를 선택함");
  setRoute(carterId, []);
  stopPoseWatchIfIdle();
}

export function getRouteSnapshot(): RouteMap {
  return routes;
}

export function subscribeRouteQueue(listener: Listener): () => void {
  listeners.add(listener);
  listener(routes);
  return () => listeners.delete(listener);
}
