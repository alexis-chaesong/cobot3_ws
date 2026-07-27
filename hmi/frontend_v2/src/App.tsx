// ══════════════════════════════════════════════════════════════
// App — [HMI v2] 전체 레이아웃 조립. carter1/carter2 통일 식별체계 + 작업선택식.
//   TopBar
//   RobotLaneGrid  (2열: carter1 | carter2, 각자 작업선택 버튼)
//   MapPanel       (자유 클릭 내비게이션)
//   MonitoringRow  (중앙 1열: 로봇 부착 카메라 — 19_ 은 carter1+carter2 둘 다 YOLO 있음,
//                   토글로 전환. 18_ 은 carter1 전용이라 고정이었음)
//   OperationsRow  (2열: 큐 | 로그)
//   AlertBanner    (조건부)
//
// 🔧 튜닝: 실제 스트림을 붙일 때 VisionFeedPanel에 streamUrl={...} 을 넘기면 즉시 표시됩니다.
// ══════════════════════════════════════════════════════════════

import { useState } from "react";
import { RobotStatusProvider } from "./context/RobotStatusContext";
import { TopBar } from "./components/TopBar/TopBar";
import { RobotLane } from "./components/RobotLane/RobotLane";
import { MapPanel } from "./components/MapPanel/MapPanel";
import { VisionFeedPanel } from "./components/VisionFeedPanel/VisionFeedPanel";
import { QueuePanel } from "./components/QueuePanel/QueuePanel";
import { LogPanel } from "./components/LogPanel/LogPanel";
import { AlertBanner } from "./components/AlertBanner/AlertBanner";
import { CARTER_IDS, CARTER_META } from "./constants/carters";
import type { CarterId } from "./types";

function VisionSection() {
  const [visionCarter, setVisionCarter] = useState<CarterId>("carter1");
  return (
    <div className="vision-section panel">
      <div className="vision-section__toggle">
        {CARTER_IDS.map((id) => (
          <button
            key={id}
            type="button"
            className={`vision-section__toggle-btn${visionCarter === id ? " vision-section__toggle-btn--active" : ""}`}
            onClick={() => setVisionCarter(id)}
            aria-pressed={visionCarter === id}
          >
            {CARTER_META[id].label}
          </button>
        ))}
      </div>
      <VisionFeedPanel
        title={`비전 (${CARTER_META[visionCarter].label} 사람 감지)`}
        streamUrl={`${import.meta.env.VITE_API_BASE}/api/vision/${visionCarter}/stream`}
      />
    </div>
  );
}

export default function App() {
  return (
    <RobotStatusProvider>
      <div className="app-shell">
        <TopBar />

        {/* [2026-07-27 디자인 개편] 좌: 로봇 상태 2장 / 중앙: 내비 지도 / 우: 비전+큐+로그.
            컴포넌트는 이전과 동일 — 배치용 그리드 컨테이너만 새로 도입(App.css: .dashboard*). */}
        <div className="dashboard">
          <div className="dashboard__robots">
            {CARTER_IDS.map((id) => (
              <RobotLane key={id} carterId={id} />
            ))}
          </div>

          <div className="dashboard__map">
            <MapPanel />
          </div>

          <div className="dashboard__side">
            <VisionSection />
            <QueuePanel />
            <LogPanel />
          </div>
        </div>

        {/* 오류 시에만 렌더 — 고정 오버레이라 그리드 레이아웃을 안 밀어냄(AlertBanner.css) */}
        <AlertBanner />
      </div>
    </RobotStatusProvider>
  );
}
