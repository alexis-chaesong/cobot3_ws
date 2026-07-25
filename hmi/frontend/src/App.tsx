// ══════════════════════════════════════════════════════════════
// App — 전체 레이아웃 조립. (기획 5번 컴포넌트 트리 그대로)
//   TopBar
//   RobotLaneGrid  (2열: 폐기물 | 소독)
//   MonitoringRow  (2열: Vision | CCTV)
//   OperationsRow  (2열: 큐 | 로그)
//   AlertBanner    (조건부)
//
// 🔧 튜닝: 실제 스트림을 붙일 때 VisionFeedPanel/CctvFeedPanel에
//          streamUrl={...} 을 넘기면 즉시 표시됩니다.
// ══════════════════════════════════════════════════════════════

import { RobotStatusProvider } from "./context/RobotStatusContext";
import { TopBar } from "./components/TopBar/TopBar";
import { RobotLane } from "./components/RobotLane/RobotLane";
import { MapPanel } from "./components/MapPanel/MapPanel";
import { VisionFeedPanel } from "./components/VisionFeedPanel/VisionFeedPanel";
import { CctvFeedPanel } from "./components/CctvFeedPanel/CctvFeedPanel";
import { QueuePanel } from "./components/QueuePanel/QueuePanel";
import { LogPanel } from "./components/LogPanel/LogPanel";
import { AlertBanner } from "./components/AlertBanner/AlertBanner";

export default function App() {
  return (
    <RobotStatusProvider>
      <div className="app-shell">
        <TopBar />

        {/* 로봇 레인 2열 */}
        <div className="grid-2col">
          <RobotLane robotId="waste" />
          <RobotLane robotId="disinfect" />
        </div>

        {/* 자유 클릭 내비게이션 지도 — 별도 행(넓어서 2열 그리드엔 안 어울림) */}
        <MapPanel />

        {/* 모니터링 2열: Vision | CCTV
            streamUrl 미지정 → placeholder 표시. 실제 URL 주입 시 바로 표시. */}
        <div className="grid-2col">
          <VisionFeedPanel title="비전 (폐기물 인식)" />
          <CctvFeedPanel title="CCTV (격리 병동)" />
        </div>

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
