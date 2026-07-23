// ── 공용 타입 정의 (프론트 전역에서 재사용) ──

export type RobotId = "waste" | "disinfect";

export type RobotState = "idle" | "running" | "stopped" | "error";

/**
 * 백엔드가 WebSocket으로 보내는 실시간 상태 메시지.
 * 7번 섹션의 ROBOT_STATUS / SAFETY_EVENT 두 종류.
 */
export interface RobotStatusMessage {
  type: "ROBOT_STATUS";
  robotId: RobotId;
  payload: {
    process_state: string; // 예: "폐기물통 파지 중"
    recovery_stage: string; // 예: "IDLE"
  };
  timestamp: string;
}

export interface SafetyEventMessage {
  type: "SAFETY_EVENT";
  robotId: RobotId;
  error_code: string; // 예: "ERR_COLLISION"
  error_msg: string;
  timestamp: string;
}

export type WsMessage = RobotStatusMessage | SafetyEventMessage;

/**
 * 프론트가 화면 렌더에 쓰는, robotId별로 정규화된 상태 스냅샷.
 * WsMessage들을 훅에서 이 형태로 누적/변환한다.
 */
export interface RobotSnapshot {
  robotId: RobotId;
  state: RobotState;
  processStateLabel: string; // 원본 process_state (예: "폐기물통 파지 중")
  currentStepIndex: number; // steps 배열 내 현재 단계 인덱스
  recoveryStage: string;
  lastError?: { code: string; message: string; at: string };
  updatedAt: string;
}

/** 로그/이력 항목 (REST /api/history, /api/errors 응답) */
export interface LogEntry {
  id: string;
  robotId: RobotId;
  level: "info" | "error";
  message: string;
  timestamp: string;
}

/** 큐 상태 항목 */
export interface QueueItem {
  taskId: string;
  robotId: RobotId;
  label: string;
  status: "queued" | "active" | "done";
}
