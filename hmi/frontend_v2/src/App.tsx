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
    <div className="monitoring-row">
      <div style={{ display: "flex", flexDirection: "column", gap: 8, width: "100%" }}>
        <div style={{ display: "flex", gap: 8, justifyContent: "center" }}>
          {CARTER_IDS.map((id) => (
            <button
              key={id}
              type="button"
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
    </div>
  );
}

export default function App() {
  return (
    <RobotStatusProvider>
      <div className="app-shell">
        <TopBar />

        {/* 로봇 레인 2열 */}
        <div className="grid-2col">
          {CARTER_IDS.map((id) => (
            <RobotLane key={id} carterId={id} />
          ))}
        </div>

        {/* 자유 클릭 내비게이션 지도 — 별도 행(넓어서 2열 그리드엔 안 어울림) */}
        <MapPanel />

        {/* 모니터링 — 19_ 은 carter1+carter2 둘 다 우측 RealSense YOLO 검출 스트림이 있어
            (hmi/backend_v2 MJPEG 중계, src/perception/perception/multi_robot_yolo_viewer.py
            가 원본) 토글로 전환한다. */}
        <VisionSection />

        {/* 운영 2열: 큐 | 로그 */}
        <div className="grid-2col">
          <QueuePanel />
          <LogPanel />
        </div>

        {/* 오류 시에만 렌더 */}
        <AlertBanner />
      </div>
    </RobotStatusProvider>
  );
}
