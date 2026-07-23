// 로봇 1대분 레인. waste/disinfect 공용이며 variant prop으로 색을 분기한다.
import { Play, Circle } from "lucide-react";
import { ROBOT_META } from "../../constants/robots";
import { STEPS_BY_ROBOT } from "../../constants/steps";
import { useRobotStatus } from "../../hooks/useRobotStatus";
import { useCommands } from "../../hooks/useCommands";
import { FlowStepRail } from "../FlowStepRail/FlowStepRail";
import type { RobotId, RobotState } from "../../types";
import "./RobotLane.css";

const STATE_LABEL: Record<RobotState, string> = {
  idle: "대기",
  running: "동작 중",
  stopped: "정지",
  error: "오류",
};

export function RobotLane({ robotId }: { robotId: RobotId }) {
  const meta = ROBOT_META[robotId];
  const snapshot = useRobotStatus(robotId);
  const commands = useCommands();
  const steps = STEPS_BY_ROBOT[robotId];

  return (
    <section className={`robot-lane robot-lane--${meta.variant} panel`}>
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

      <FlowStepRail
        steps={steps}
        currentStep={snapshot.currentStepIndex}
        variant={meta.variant}
      />

      <footer className="robot-lane__foot">
        <button
          type="button"
          className="robot-lane__start"
          onClick={() => commands.startRobot(robotId)}
        >
          <Play size={14} strokeWidth={2.5} />
          개별 시작
        </button>
      </footer>
    </section>
  );
}
