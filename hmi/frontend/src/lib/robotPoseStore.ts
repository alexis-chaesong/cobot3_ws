// ══════════════════════════════════════════════════════════════
// robotPoseStore.ts — carter1/carter2 실시간 위치(ROBOT_POSE)를 구독하는 모듈 싱글턴.
// uiActionLog.ts와 동일한 pub/sub 스타일. hooks/useRobotPose.ts(화면 표시)와
// lib/routeQueue.ts(웨이포인트 도착 판정)가 이 하나의 소스를 공유한다 — WS 커넥션/
// mock 타이머를 구독자 수만큼 중복 생성하지 않기 위함.
// 🔧 튜닝: MOCK 가짜 경로는 MOCK_PATHS.
// ══════════════════════════════════════════════════════════════

import { CARTER_IDS } from "../constants/carters";
import { apiClient } from "./apiClient";
import { MOCK } from "./mock/mockSocket";
import type { CarterId, WsMessage } from "../types";

export interface Pose {
  x: number;
  y: number;
  yaw: number;
}
export type PoseMap = Record<CarterId, Pose | null>;

function initialPoseMap(): PoseMap {
  return CARTER_IDS.reduce((acc, id) => {
    acc[id] = null;
    return acc;
  }, {} as PoseMap);
}

// 목 모드 가짜 경로 — 복도 왕복(6-15의 실제 스윕 좌표 근사, 지도 범위 내 값).
const MOCK_PATHS: Record<
  CarterId,
  { cx: number; cy: number; r: number; speed: number; phase: number }
> = {
  carter1: { cx: 18.5, cy: 13.0, r: 5.5, speed: 0.4, phase: 0 },
  carter2: { cx: 10.0, cy: 10.0, r: 6.0, speed: 0.3, phase: Math.PI },
};

let poses: PoseMap = initialPoseMap();
type Listener = (poses: PoseMap) => void;
const listeners = new Set<Listener>();
let stopFn: (() => void) | null = null;

function notify() {
  for (const listener of listeners) listener(poses);
}

function start(): void {
  if (MOCK) {
    const t0 = performance.now();
    const interval = setInterval(() => {
      const t = (performance.now() - t0) / 1000;
      const next: PoseMap = { ...poses };
      for (const id of CARTER_IDS) {
        const p = MOCK_PATHS[id];
        const angle = p.phase + t * p.speed;
        next[id] = {
          x: p.cx + p.r * Math.cos(angle),
          y: p.cy + p.r * Math.sin(angle) * 0.5,
          yaw: angle + Math.PI / 2,
        };
      }
      poses = next;
      notify();
    }, 500);
    stopFn = () => clearInterval(interval);
    return;
  }

  const handle = (msg: WsMessage) => {
    if (msg.type !== "ROBOT_POSE") return;
    poses = { ...poses, [msg.robotId]: { x: msg.x, y: msg.y, yaw: msg.yaw } };
    notify();
  };
  stopFn = apiClient.connectWebSocket(handle);
}

/** 현재 스냅샷(구독 없이 1회성 조회 — routeQueue 의 도착 판정용). */
export function getPoseSnapshot(): PoseMap {
  return poses;
}

/** 구독 시작(첫 구독자일 때만 WS/mock 타이머 기동) + 즉시 현재값 1회 전달. */
export function subscribeRobotPoseStore(listener: Listener): () => void {
  if (listeners.size === 0) start();
  listeners.add(listener);
  listener(poses);
  return () => {
    listeners.delete(listener);
    if (listeners.size === 0 && stopFn) {
      stopFn();
      stopFn = null;
    }
  };
}
