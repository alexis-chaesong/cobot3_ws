// ══════════════════════════════════════════════════════════════
// RobotStatusContext — [HMI v2] WebSocket(또는 mockSocket) 구독을 앱 최상단에서
// 딱 한 번만 열고, carterId별 스냅샷을 Context로 하위에 배포한다.
// (기획: Redux/Zustand 없이 React Context + 커스텀 훅)
//
// 여기가 "WsMessage → RobotSnapshot" 변환의 유일한 지점이다.
// 🔧 튜닝: 상태(state) 판정 규칙은 deriveState()에서 수정.
// ══════════════════════════════════════════════════════════════

import {
  createContext,
  useContext,
  useEffect,
  useRef,
  useState,
  type ReactNode,
} from "react";
import { CARTER_IDS } from "../constants/carters";
import { STEPS_BY_TASK } from "../constants/steps";
import { apiClient } from "../lib/apiClient";
import { MOCK, connectMockSocket } from "../lib/mock/mockSocket";
import { markHistoryReset } from "../lib/historyResetMarker";
import { clearUiActionLog } from "../lib/uiActionLog";
import type {
  CarterId,
  RobotSnapshot,
  RobotState,
  TaskId,
  WsMessage,
} from "../types";

type SnapshotMap = Record<CarterId, RobotSnapshot>;

function initialSnapshots(): SnapshotMap {
  const now = new Date().toISOString();
  return CARTER_IDS.reduce((acc, id) => {
    acc[id] = {
      robotId: id,
      task: null,
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
 * process_state 문자열 → STEPS_BY_TASK[task] 배열 인덱스.
 * "폐기물통 파지 중"처럼 뒤에 " 중"이 붙어도 매칭되도록 startsWith 비교.
 * ★같은 라벨이 배열에 두 번 있을 때★ 현재 인덱스부터 앞으로 먼저 탐색해 '뒤로 점프'를 막는다.
 * 앞에서 못 찾으면(대기로의 리셋 등) 처음부터 다시 탐색한다. task 가 null(작업 미배정)이면
 * 매칭할 배열이 없으므로 항상 -1. 매칭 실패 시에도 -1.
 */
function resolveStepIndex(
  task: TaskId | null,
  processState: string,
  fromIndex = 0,
): number {
  if (task === null) return -1;
  const steps = STEPS_BY_TASK[task];
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
      // ROBOT_POSE 는 위치(x/y/yaw) 전용 메시지 — 이 스냅샷(RobotSnapshot)엔 위치 필드가
      // 없으므로 여기선 무시한다(useRobotPose 가 robotPoseStore 를 통해 별도로 구독).
      if (msg.type === "ROBOT_POSE") return;

      setSnapshots((prev) => {
        const cur = prev[msg.robotId];
        if (!cur) return prev;

        if (msg.type === "ROBOT_STATUS") {
          const { process_state, recovery_stage, task } = msg.payload;
          const idx = resolveStepIndex(task, process_state, cur.currentStepIndex);
          return {
            ...prev,
            [msg.robotId]: {
              ...cur,
              task,
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

  // [HMI v2 신규] 두 로봇이 모두 대기(idle) 상태가 되는 순간 = "전체 작업 완료"로 보고
  // 작업 큐/로그 화면을 정리한다(DB는 보존 — historyResetMarker.ts 참고). 최초 마운트 시
  // (기본값이 이미 둘 다 idle) 곧바로 초기화가 발동하지 않도록 이전값을 true 로 시작한다.
  const prevAllIdleRef = useRef(true);
  useEffect(() => {
    const allIdle = CARTER_IDS.every((id) => snapshots[id].state === "idle");
    if (allIdle && !prevAllIdleRef.current) {
      markHistoryReset();
      clearUiActionLog();
    }
    prevAllIdleRef.current = allIdle;
  }, [snapshots]);

  const hasError = CARTER_IDS.some((id) => snapshots[id].state === "error");

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
