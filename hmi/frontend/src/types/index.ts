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

// 자유 클릭 내비게이션 전용 식별자 = ROS 네임스페이스(carter1/carter2).
// 매핑: carter1=소독(disinfect, blue), carter2=폐기물(waste, red). RobotId(waste/disinfect)와는 별개 체계.
export type CarterId = "carter1" | "carter2";

/**
 * 로봇 위치(map 프레임) 실시간 메시지. amcl_pose 에서 유도됨.
 * robotId 는 RobotId(waste/disinfect)가 아니라 CarterId(carter1/carter2)를 쓴다.
 */
export interface RobotPoseMessage {
  type: "ROBOT_POSE";
  robotId: CarterId;
  x: number;
  y: number;
  yaw: number; // 라디안
  timestamp: string;
}

export type WsMessage = RobotStatusMessage | SafetyEventMessage | RobotPoseMessage;

/** Nav2 맵 메타데이터 (map.yaml 값 그대로 + png 크기). 픽셀↔world 좌표 변환에 사용. */
export interface MapInfo {
  resolution: number;
  originX: number;
  originY: number;
  width: number;
  height: number;
}

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

/** 로그/이력 항목 — 로봇 쪽 이벤트(REST /api/history, /api/errors 응답). */
export interface RobotLogEntry {
  kind: "robot";
  id: string;
  robotId: RobotId;
  level: "info" | "error";
  message: string;
  timestamp: string;
}

/**
 * 관리자가 UI(프론트엔드)에서 명령(통합/개별 시작·긴급정지·자유클릭 내비 이동 등)을
 * 선택할 때마다 클라이언트에서 즉시 기록되는 로그. lib/uiActionLog.ts 가 적재.
 */
export interface UiActionLogEntry {
  kind: "ui";
  id: string;
  targetLabel: string; // 대상 표시명 (예: "소독 로봇 (carter1)", "전체")
  level: "info";
  message: string; // 예: "통합 시작을 선택함"
  timestamp: string;
}

/** 로그/이력 항목 (LogPanel 이 표시하는 통합 타입) */
export type LogEntry = RobotLogEntry | UiActionLogEntry;

/**
 * 큐 상태 항목 — routeQueue.ts(웨이포인트 경로)에서 유도됨. carter1/carter2 웨이포인트를
 * 표시해야 해서 RobotId 대신 색 분기 키(variant)를 직접 들고 있다.
 */
export interface QueueItem {
  taskId: string;
  variant: "waste" | "disinfect";
  label: string;
  status: "queued" | "active" | "done";
}
