// [HMI v2] 로봇 1대분 레인. carter1/carter2 공용이며, 색(variant)은 고정이 아니라 "현재
// 배정된 작업"에서 유도한다 — 작업 미배정이면 중립색(CSS 기본값)으로 표시.
// 기존 "개별 시작" 버튼을 "작업선택" 버튼(trash/spray)으로 대체 — 클릭=배정=시작(별도
// 시작 스텝 없음, 19_ 의 task_select 모델과 일치).
import { Circle, OctagonX, Play } from "lucide-react";
import { CARTER_IDS, CARTER_META } from "../../constants/carters";
import { TASK_IDS, TASK_META } from "../../constants/tasks";
import { STEPS_BY_TASK } from "../../constants/steps";
import { useRobotStatus } from "../../hooks/useRobotStatus";
import { useCommands } from "../../hooks/useCommands";
import { useRobotStatusContext } from "../../context/RobotStatusContext";
import { FlowStepRail } from "../FlowStepRail/FlowStepRail";
import type { CarterId, RobotState, TaskId } from "../../types";
import "./RobotLane.css";

const STATE_LABEL: Record<RobotState, string> = {
  idle: "대기",
  running: "동작 중",
  stopped: "정지",
  error: "오류",
};

export function RobotLane({ carterId }: { carterId: CarterId }) {
  const meta = CARTER_META[carterId];
  const snapshot = useRobotStatus(carterId);
  const commands = useCommands();
  const task = snapshot.task;
  const variantClass = task ? `robot-lane--${TASK_META[task].variant}` : "";
  // 19_ 가 EMERGENCY_STOP 수신 시 publish_hmi_state(ctx, "긴급정지") 로 발행하는 라벨(MapPanel.tsx
  // 의 ESTOP_LABEL_PREFIX 와 동일 규칙). 이 로봇이 지금 긴급정지 중일 때만 재개 버튼을 보여준다.
  const isEstopped = snapshot.processStateLabel.startsWith("긴급정지");

  // [HMI v2 신규] 같은 작업(trash|spray)을 두 로봇에 중복 배정하는 것 방지 — 다른 로봇이 이미
  // 그 작업을 실행 중이면 배정을 막고 안내한다. task 필드는 완료 후에도 안 남아있을 수 있어
  // (process_state 만 "대기"로 리셋) state==="running" 도 같이 확인해야 "완료된 작업" 오탐을 피한다.
  const { snapshots } = useRobotStatusContext();
  const otherCarterId = CARTER_IDS.find((id) => id !== carterId);
  const otherSnapshot = otherCarterId ? snapshots[otherCarterId] : undefined;
  const handleSelectTask = (t: TaskId) => {
    if (otherSnapshot && otherSnapshot.task === t && otherSnapshot.state === "running") {
      window.alert("이미 다른 로봇이 실행중인 작업입니다");
      return;
    }
    commands.selectTask(carterId, t);
  };

  return (
    <section className={`robot-lane ${variantClass} panel`}>
      <header className="robot-lane__head">
        <div className="robot-lane__title">
          <span className="robot-lane__badge" />
          <h2>{meta.label}</h2>
        </div>
        <div className="robot-lane__head-right">
          <div className={`robot-lane__state robot-lane__state--${snapshot.state}`}>
            <Circle size={9} fill="currentColor" strokeWidth={0} />
            {STATE_LABEL[snapshot.state]}
          </div>
          {/* [HMI v2 신규] 긴급정지 중일 때만 노출 — 이 로봇만 재개(START, 새 작업 배정 없음) */}
          {isEstopped && (
            <button
              type="button"
              className="robot-lane__resume-btn"
              title={`${meta.label} 동작 재개(긴급정지 해제)`}
              onClick={() => commands.resume(carterId)}
            >
              <Play size={13} strokeWidth={2.5} />
              동작 재개
            </button>
          )}
          {/* [HMI v2 신규] 로봇별 개별 긴급정지 — TopBar 의 전체 긴급정지와 별개로 이 로봇만 정지 */}
          <button
            type="button"
            className="robot-lane__estop-btn"
            title={`${meta.label} 긴급정지`}
            onClick={() => commands.estop(carterId)}
          >
            <OctagonX size={13} strokeWidth={2.5} />
            긴급정지
          </button>
        </div>
      </header>

      <p className="robot-lane__process">{snapshot.processStateLabel}</p>

      {task ? (
        <FlowStepRail
          steps={STEPS_BY_TASK[task]}
          currentStep={snapshot.currentStepIndex}
          variant={TASK_META[task].variant}
        />
      ) : (
        <p className="robot-lane__unassigned">작업 미배정 — 아래에서 작업을 선택하세요.</p>
      )}

      <footer className="robot-lane__foot">
        {TASK_IDS.map((t) => (
          <button
            key={t}
            type="button"
            className={`robot-lane__task-btn robot-lane__task-btn--${TASK_META[t].variant}`}
            onClick={() => handleSelectTask(t)}
          >
            {TASK_META[t].label} 선택
          </button>
        ))}
      </footer>
    </section>
  );
}
