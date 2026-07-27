// [HMI v2] 로봇 1대분 레인. carter1/carter2 공용이며, 색(variant)은 고정이 아니라 "현재
// 배정된 작업"에서 유도한다 — 작업 미배정이면 중립색(CSS 기본값)으로 표시.
// 기존 "개별 시작" 버튼을 "작업선택" 버튼(trash/spray)으로 대체 — 클릭=배정=시작(별도
// 시작 스텝 없음, 19_ 의 task_select 모델과 일치).
import { Circle } from "lucide-react";
import { CARTER_META } from "../../constants/carters";
import { TASK_IDS, TASK_META } from "../../constants/tasks";
import { STEPS_BY_TASK } from "../../constants/steps";
import { useRobotStatus } from "../../hooks/useRobotStatus";
import { useCommands } from "../../hooks/useCommands";
import { FlowStepRail } from "../FlowStepRail/FlowStepRail";
import type { CarterId, RobotState } from "../../types";
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

  return (
    <section className={`robot-lane ${variantClass} panel`}>
      <header className="robot-lane__head">
        <div className="robot-lane__title">
          <span className="robot-lane__badge" />
          <h2>{meta.label}</h2>
        </div>
        <div className={`robot-lane__state robot-lane__state--${snapshot.state}`}>
          <Circle size={9} fill="currentColor" strokeWidth={0} />
          {STATE_LABEL[snapshot.state]}
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
            onClick={() => commands.selectTask(carterId, t)}
          >
            {TASK_META[t].label} 선택
          </button>
        ))}
      </footer>
    </section>
  );
}
