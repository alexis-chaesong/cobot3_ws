// 작업 큐 상태 패널.
import { ListChecks } from "lucide-react";
import { useQueueStatus } from "../../hooks/useQueueStatus";
import { ROBOT_META } from "../../constants/robots";
import "./QueuePanel.css";

const STATUS_LABEL = { queued: "대기열", active: "진행", done: "완료" } as const;

export function QueuePanel() {
  const queue = useQueueStatus();

  return (
    <div className="panel queue-panel">
      <div className="queue-panel__head">
        <span className="panel-title">
          <ListChecks size={14} /> 작업 큐
        </span>
        <span className="queue-panel__count">{queue.length}건</span>
      </div>

      <ul className="queue-panel__list">
        {queue.length === 0 && <li className="queue-panel__empty">대기 중인 작업 없음</li>}
        {queue.map((item) => (
          <li key={item.taskId} className="queue-panel__item">
            <span
              className="queue-panel__dot"
              style={{ background: `var(--${ROBOT_META[item.robotId].variant}-accent)` }}
            />
            <span className="queue-panel__label">{item.label}</span>
            <span className={`queue-panel__status queue-panel__status--${item.status}`}>
              {STATUS_LABEL[item.status]}
            </span>
          </li>
        ))}
      </ul>
    </div>
  );
}
