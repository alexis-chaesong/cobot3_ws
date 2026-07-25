// ══════════════════════════════════════════════════════════════
// RobotStatusContext — WebSocket(또는 mockSocket) 구독을 앱 최상단에서
// 딱 한 번만 열고, robotId별 스냅샷을 Context로 하위에 배포한다.
// (기획: Redux/Zustand 없이 React Context + 커스텀 훅)
//
// 여기가 "WsMessage → RobotSnapshot" 변환의 유일한 지점이다.
// 🔧 튜닝: 상태(state) 판정 규칙은 deriveState()에서 수정.
// ══════════════════════════════════════════════════════════════

import {
  createContext,
  useContext,
  useEffect,
  useState,
  type ReactNode,
} from "react";
import { ROBOT_IDS } from "../constants/robots";
import { STEPS_BY_ROBOT } from "../constants/steps";
import { apiClient } from "../lib/apiClient";
import { MOCK, connectMockSocket } from "../lib/mock/mockSocket";
import type {
  RobotId,
  RobotSnapshot,
  RobotState,
  WsMessage,
} from "../types";

type SnapshotMap = Record<RobotId, RobotSnapshot>;

function initialSnapshots(): SnapshotMap {
  const now = new Date().toISOString();
  return ROBOT_IDS.reduce((acc, id) => {
    acc[id] = {
      robotId: id,
      state: "idle",
      processStateLabel: "대기",
      currentStepIndex: 0,
      recoveryStage: "IDLE",
      updatedAt: now,
    };
    return acc;
  }, {} as SnapshotMap);
}

/**
 * process_state 문자열 → steps 배열 인덱스.
 * "폐기물통 파지 중"처럼 뒤에 " 중"이 붙어도 매칭되도록 startsWith 비교.
 * ★같은 라벨이 배열에 두 번 있을 때(소독 로봇의 "복도 진입" 1·2차)★ 현재 인덱스부터 앞으로
 * 먼저 탐색해 '뒤로 점프'를 막는다(예: 소독 분사(4) → 복도 진입은 3 이 아니라 5 로 전진).
 * 앞에서 못 찾으면(대기로의 리셋 등) 처음부터 다시 탐색한다. 매칭 실패 시 -1.
 */
function resolveStepIndex(
  robotId: RobotId,
  processState: string,
  fromIndex = 0,
): number {
  const steps = STEPS_BY_ROBOT[robotId];
  const clean = processState.replace(/\s*중$/, "").trim();
  const matches = (s: string) => s === clean || processState.startsWith(s);
  // 1) 현재 단계부터 앞으로 (반복 라벨은 다음 것으로 전진)
  for (let i = Math.max(0, fromIndex); i < steps.length; i++) {
    if (matches(steps[i])) return i;
  }
  // 2) 앞에 없으면 처음부터 (사이클 재시작 · 대기 복귀 등 뒤로 가는 정상 전이)
  for (let i = 0; i < steps.length; i++) {
    if (matches(steps[i])) return i;
  }
  return -1;
}

/** 상태 판정 규칙. 🔧 튜닝 포인트. */
function deriveState(processState: string, recoveryStage: string): RobotState {
  if (recoveryStage && recoveryStage !== "IDLE") return "error";
  if (processState === "대기") return "idle";
  return "running";
}

interface RobotStatusContextValue {
  snapshots: SnapshotMap;
  /** 하나라도 error면 true (AlertBanner 조건) */
  hasError: boolean;
}

const RobotStatusContext = createContext<RobotStatusContextValue | null>(null);

export function RobotStatusProvider({ children }: { children: ReactNode }) {
  const [snapshots, setSnapshots] = useState<SnapshotMap>(initialSnapshots);

  useEffect(() => {
    const handle = (msg: WsMessage) => {
      // ROBOT_POSE 는 carter1/carter2(CarterId) 대상이라 이 스냅샷맵(waste/disinfect)엔 없다.
      // useRobotPose 가 별도로 구독하므로 여기선 무시(가드 없으면 prev[carterId] 인덱싱 에러).
      if (msg.type === "ROBOT_POSE") return;

      setSnapshots((prev) => {
        const cur = prev[msg.robotId];
        if (!cur) return prev;

        if (msg.type === "ROBOT_STATUS") {
          const { process_state, recovery_stage } = msg.payload;
          const idx = resolveStepIndex(
            msg.robotId,
            process_state,
            cur.currentStepIndex,
          );
          return {
            ...prev,
            [msg.robotId]: {
              ...cur,
              processStateLabel: process_state,
              recoveryStage: recovery_stage,
              currentStepIndex: idx >= 0 ? idx : cur.currentStepIndex,
              state: deriveState(process_state, recovery_stage),
              updatedAt: msg.timestamp,
            },
          };
        }

        // SAFETY_EVENT → 해당 로봇 error 처리
        return {
          ...prev,
          [msg.robotId]: {
            ...cur,
            state: "error",
            lastError: {
              code: msg.error_code,
              message: msg.error_msg,
              at: msg.timestamp,
            },
            updatedAt: msg.timestamp,
          },
        };
      });
    };

    // 목 모드 / 실제 모드 분기. 두 함수 모두 동일한 cleanup 시그니처.
    const cleanup = MOCK
      ? connectMockSocket(handle)
      : apiClient.connectWebSocket(handle);

    return cleanup;
  }, []);

  const hasError = ROBOT_IDS.some((id) => snapshots[id].state === "error");

  return (
    <RobotStatusContext.Provider value={{ snapshots, hasError }}>
      {children}
    </RobotStatusContext.Provider>
  );
}

export function useRobotStatusContext(): RobotStatusContextValue {
  const ctx = useContext(RobotStatusContext);
  if (!ctx) {
    throw new Error("useRobotStatusContext must be used within RobotStatusProvider");
  }
  return ctx;
}
