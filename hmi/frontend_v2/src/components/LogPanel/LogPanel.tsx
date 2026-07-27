// 로그/이력 패널. 로봇 이벤트(useLogs, REST/목)와 관리자 UI 액션(useUiActionLog)을
// 합쳐 시간순(최신 우선)으로 보여준다. info/error 색 구분 + UI 액션은 아이콘으로 구분.
import { ScrollText, MousePointerClick } from "lucide-react";
import { useLogs } from "../../hooks/useLogs";
import { useUiActionLog } from "../../hooks/useUiActionLog";
import { CARTER_META } from "../../constants/carters";
import type { LogEntry } from "../../types";
import "./LogPanel.css";

function fmtTime(iso: string): string {
  const d = new Date(iso);
  return Number.isNaN(d.getTime())
    ? iso
    : d.toLocaleTimeString("ko-KR", { hour12: false });
}

export function LogPanel() {
  const robotLogs = useLogs();
  const uiLogs = useUiActionLog();

  const merged: LogEntry[] = [...robotLogs, ...uiLogs].sort(
    (a, b) => new Date(b.timestamp).getTime() - new Date(a.timestamp).getTime(),
  );

  return (
    <div className="panel log-panel">
      <div className="log-panel__head">
        <span className="panel-title">
          <ScrollText size={14} /> 로그 / 이력
        </span>
      </div>

      <ul className="log-panel__list">
        {merged.length === 0 && <li className="log-panel__empty">로그 없음</li>}
        {merged.map((log) => {
          const isUi = log.kind === "ui";
          const label = isUi ? log.targetLabel : CARTER_META[log.robotId].label;
          return (
            <li
              key={log.id}
              className={`log-panel__row log-panel__row--${log.level}${
                isUi ? " log-panel__row--ui" : ""
              }`}
            >
              <span className="log-panel__time">{fmtTime(log.timestamp)}</span>
              <span className="log-panel__robot">{label}</span>
              <span className="log-panel__msg">
                {isUi && (
                  <MousePointerClick size={11} className="log-panel__ui-icon" />
                )}
                {log.message}
              </span>
            </li>
          );
        })}
      </ul>
    </div>
  );
}
