// 상단 바: 통합 시작(좌), 긴급정지(우, 분리 배치). 긴급정지는 확인창 없이 즉시 실행.
import { Play, OctagonX, ShieldCheck } from "lucide-react";
import { useCommands } from "../../hooks/useCommands";
import { useRobotStatusContext } from "../../context/RobotStatusContext";
import "./TopBar.css";

export function TopBar() {
  const commands = useCommands();
  const { hasError } = useRobotStatusContext();

  return (
    <header className="topbar panel">
      <div className="topbar__brand">
        <ShieldCheck size={20} className="topbar__brand-icon" />
        <div>
          <h1 className="topbar__title">격리 병동 로봇 관제</h1>
          <p className="topbar__subtitle">관리자 대시보드</p>
        </div>
      </div>

      <div className="topbar__actions">
        <button
          type="button"
          className="topbar__start-all"
          onClick={() => commands.startAll()}
        >
          <Play size={16} strokeWidth={2.5} />
          통합 시작
        </button>

        {/* 긴급정지: 시각적으로 분리 + 확인창 없이 즉시 전체 정지 */}
        <button
          type="button"
          className="topbar__estop"
          onClick={() => commands.estop()}
          aria-live="off"
        >
          <OctagonX size={18} strokeWidth={2.5} />
          긴급정지
          {hasError && <span className="topbar__estop-pulse" />}
        </button>
      </div>
    </header>
  );
}
