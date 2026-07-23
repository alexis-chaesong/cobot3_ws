// 로그/이력 패널. useLogs로 조회(목 or REST). info/error 색 구분.
import { ScrollText } from "lucide-react";
import { useLogs } from "../../hooks/useLogs";
import { ROBOT_META } from "../../constants/robots";
import "./LogPanel.css";

function fmtTime(iso: string): string {
  const d = new Date(iso);
  return Number.isNaN(d.getTime())
    ? iso
    : d.toLocaleTimeString("ko-KR", { hour12: false });
}

export function LogPanel() {
  const logs = useLogs();

  return (
    <div className="panel log-panel">
      <div className="log-panel__head">
        <span className="panel-title">
          <ScrollText size={14} /> 로그 / 이력
        </span>
      </div>

      <ul className="log-panel__list">
        {logs.length === 0 && <li className="log-panel__empty">로그 없음</li>}
        {logs.map((log) => (
          <li key={log.id} className={`log-panel__row log-panel__row--${log.level}`}>
            <span className="log-panel__time">{fmtTime(log.timestamp)}</span>
            <span className="log-panel__robot">{ROBOT_META[log.robotId].label}</span>
            <span className="log-panel__msg">{log.message}</span>
          </li>
        ))}
      </ul>
    </div>
  );
}
