// ── 공용 타입 정의 (프론트 전역에서 재사용) ── [HMI v2] carter1/carter2 단일 식별체계.
// 기존 hmi/frontend 는 RobotId(waste/disinfect, 상태/명령용)와 CarterId(carter1/carter2,
// 지도/내비/비전용)가 같은 필드명(robotId)에 서로 다른 타입으로 쓰이는 이원 체계였다.
// 19_ 은 역할고정이 없으므로(로봇이 런타임에 task 를 스스로 고름) CarterId 하나로 통일하고,
// "지금 이 로봇이 무슨 작업 중인지"는 TaskId 필드로 별도로 표현한다.

export type CarterId = "carter1" | "carter2";

export type TaskId = "trash" | "spray";

export type RobotState = "idle" | "running" | "stopped" | "error";

/**
 * 백엔드가 WebSocket으로 보내는 실시간 상태 메시지.
 * payload.task 는 [HMI v2 신규] — 현재 이 로봇에 배정된 작업(없으면 null).
 */
export interface RobotStatusMessage {
  type: "ROBOT_STATUS";
  robotId: CarterId;
  payload: {
    process_state: string; // 예: "폐기물통 파지 중"
    recovery_stage: string; // 예: "IDLE"
    task: TaskId | null;
  };
  timestamp: string;
}

export interface SafetyEventMessage {
  type: "SAFETY_EVENT";
  robotId: CarterId;
  error_code: string; // 예: "ERR_COLLISION"
  error_msg: string;
  timestamp: string;
}

/** 로봇 위치(map 프레임) 실시간 메시지. amcl_pose 에서 유도됨. */
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
 * 프론트가 화면 렌더에 쓰는, carterId별로 정규화된 상태 스냅샷.
 * WsMessage들을 훅에서 이 형태로 누적/변환한다. task 가 null 이면 "작업 미배정"
 * (steps/variant 매칭 자체를 건너뛰고 대기 화면 표시).
 */
export interface RobotSnapshot {
  robotId: CarterId;
  task: TaskId | null;
  state: RobotState;
  processStateLabel: string; // 원본 process_state (예: "폐기물통 파지 중")
  currentStepIndex: number; // STEPS_BY_TASK[task] 내 현재 단계 인덱스
  recoveryStage: string;
  lastError?: { code: string; message: string; at: string };
  updatedAt: string;
}

/** 로그/이력 항목 — 로봇 쪽 이벤트(REST /api/history, /api/errors 응답). */
export interface RobotLogEntry {
  kind: "robot";
  id: string;
  robotId: CarterId;
  level: "info" | "error";
  message: string;
  timestamp: string;
}

/**
 * 관리자가 UI(프론트엔드)에서 명령(통합/작업선택·긴급정지·자유클릭 내비 이동 등)을
 * 선택할 때마다 클라이언트에서 즉시 기록되는 로그. lib/uiActionLog.ts 가 적재.
 */
export interface UiActionLogEntry {
  kind: "ui";
  id: string;
  targetLabel: string; // 대상 표시명 (예: "로봇 1 (carter1)", "전체")
  level: "info";
  message: string; // 예: "폐기물 수거 작업을 선택함"
  timestamp: string;
}

/** 로그/이력 항목 (LogPanel 이 표시하는 통합 타입) */
export type LogEntry = RobotLogEntry | UiActionLogEntry;

/**
 * 큐 상태 항목 — routeQueue.ts(웨이포인트 경로)에서 유도됨. carter1/carter2 웨이포인트를
 * 표시해야 해서 CarterId 대신 색 분기 키(variant)를 직접 들고 있다. [HMI v2] variant 는
 * 이제 로봇 고정색이 아니라 해당 작업(task)의 색(TASK_META[task].variant)에서 유도된다.
 */
export interface QueueItem {
  taskId: string;
  variant: "waste" | "disinfect";
  label: string;
  status: "queued" | "active" | "done";
}
