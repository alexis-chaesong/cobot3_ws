// 에러/긴급정지 상태일 때만 렌더되는 경고 배너.
// 레이아웃 내 고정 슬롯이므로, 조건 미충족 시 null을 반환해 자리를 비운다.
import { AlertTriangle } from "lucide-react";
import { CARTER_IDS, CARTER_META } from "../../constants/carters";
import { useRobotStatusContext } from "../../context/RobotStatusContext";
import "./AlertBanner.css";

export function AlertBanner() {
  const { snapshots, hasError } = useRobotStatusContext();
  if (!hasError) return null; // 오류 없으면 렌더 안 함

  const errored = CARTER_IDS.map((id) => snapshots[id]).filter(
    (s) => s.state === "error",
  );

  return (
    <div className="alert-banner" role="alert">
      <AlertTriangle size={20} strokeWidth={2.5} className="alert-banner__icon" />
      <div className="alert-banner__body">
        {errored.map((s) => (
          <div key={s.robotId} className="alert-banner__line">
            <strong>{CARTER_META[s.robotId].label}</strong>
            <span>
              {s.lastError
                ? `${s.lastError.code} — ${s.lastError.message}`
                : "오류 상태"}
            </span>
          </div>
        ))}
      </div>
    </div>
  );
}
