// ══════════════════════════════════════════════════════════════
// App — 전체 레이아웃 조립. (기획 5번 컴포넌트 트리 그대로)
//   TopBar
//   RobotLaneGrid  (2열: 폐기물 | 소독)
//   MonitoringRow  (중앙 1열: 로봇 부착 카메라)
//   OperationsRow  (2열: 큐 | 로그)
//   AlertBanner    (조건부)
//
// 🔧 튜닝: 실제 스트림을 붙일 때 VisionFeedPanel에 streamUrl={...} 을 넘기면 즉시 표시됩니다.
// ══════════════════════════════════════════════════════════════

import { RobotStatusProvider } from "./context/RobotStatusContext";
import { TopBar } from "./components/TopBar/TopBar";
import { RobotLane } from "./components/RobotLane/RobotLane";
import { MapPanel } from "./components/MapPanel/MapPanel";
import { VisionFeedPanel } from "./components/VisionFeedPanel/VisionFeedPanel";
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

        {/* 모니터링 — 로봇 부착 카메라 1개, 중앙 정렬.
            carter1(소독) 우측 RealSense YOLO 검출 스트림(hmi/backend MJPEG 중계,
            src/perception/perception/multi_robot_yolo_viewer.py 가 원본). */}
        <div className="monitoring-row">
          <VisionFeedPanel
            title="비전 (소독 로봇 사람 감지)"
            streamUrl={`${import.meta.env.VITE_API_BASE}/api/vision/carter1/stream`}
          />
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
