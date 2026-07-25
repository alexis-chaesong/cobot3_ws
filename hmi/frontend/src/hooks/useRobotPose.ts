// carter1/carter2 실시간 위치(ROBOT_POSE) 구독. RobotStatusContext와 별개의 독립 구독.
// (WsMessage엔 포함되지만 RobotStatusContext.handle()이 ROBOT_POSE를 무시하도록 가드돼 있음.)
// 🔧 튜닝: MOCK 모드는 mockSocket.ts를 건드리지 않고 이 훅 자체 setInterval로 가짜 좌표를 방출한다
//   (mockSocket에 얹으면 상태 스텝과 이중으로 진행되어버림 — 인수인계서 참조).
import { useEffect, useState } from "react";
import { CARTER_IDS } from "../constants/carters";
import { apiClient } from "../lib/apiClient";
import { MOCK } from "../lib/mock/mockSocket";
import type { CarterId, WsMessage } from "../types";

export type PoseMap = Record<CarterId, { x: number; y: number; yaw: number } | null>;

function initialPoseMap(): PoseMap {
  return CARTER_IDS.reduce((acc, id) => {
    acc[id] = null;
    return acc;
  }, {} as PoseMap);
}

// 목 모드 가짜 경로 — 복도 왕복(6-15의 실제 스윕 좌표 근사, 지도 범위 내 값).
const MOCK_PATHS: Record<CarterId, { cx: number; cy: number; r: number; speed: number; phase: number }> = {
  carter1: { cx: 18.5, cy: 13.0, r: 5.5, speed: 0.4, phase: 0 },
  carter2: { cx: 10.0, cy: 10.0, r: 6.0, speed: 0.3, phase: Math.PI },
};

export function useRobotPose(): PoseMap {
  const [poses, setPoses] = useState<PoseMap>(initialPoseMap);

  useEffect(() => {
    if (MOCK) {
      const t0 = performance.now();
      const interval = setInterval(() => {
        const t = (performance.now() - t0) / 1000;
        setPoses((prev) => {
          const next = { ...prev };
          for (const id of CARTER_IDS) {
            const p = MOCK_PATHS[id];
            const angle = p.phase + t * p.speed;
            const x = p.cx + p.r * Math.cos(angle);
            const y = p.cy + p.r * Math.sin(angle) * 0.5;
            const yaw = angle + Math.PI / 2;
            next[id] = { x, y, yaw };
          }
          return next;
        });
      }, 500);
      return () => clearInterval(interval);
    }

    const handle = (msg: WsMessage) => {
      if (msg.type !== "ROBOT_POSE") return;
      setPoses((prev) => ({
        ...prev,
        [msg.robotId]: { x: msg.x, y: msg.y, yaw: msg.yaw },
      }));
    };

    return apiClient.connectWebSocket(handle);
  }, []);

  return poses;
}
