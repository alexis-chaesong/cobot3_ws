// 비전(로봇 카메라 + 인식 결과) 패널.
// streamUrl이 없으면 placeholder, 있으면 <img>로 표시. bbox 오버레이 자리 포함.
// 🔧 튜닝: 실제 스트림 연결 시 App에서 streamUrl prop만 주입하면 된다.
import { ScanEye } from "lucide-react";
import "./VisionFeedPanel.css";

interface BBox {
  x: number; // 0~1 정규화 좌표
  y: number;
  w: number;
  h: number;
  label?: string;
}

interface Props {
  title?: string;
  streamUrl?: string; // MJPEG/스냅샷 URL 등
  boxes?: BBox[]; // 인식 결과 bounding box
}

export function VisionFeedPanel({ title = "Vision", streamUrl, boxes = [] }: Props) {
  return (
    <div className="panel feed-panel">
      <div className="feed-panel__head">
        <span className="panel-title">
          <ScanEye size={14} /> {title}
        </span>
      </div>

      <div className="feed-panel__stage">
        {streamUrl ? (
          <img className="feed-panel__img" src={streamUrl} alt="vision feed" />
        ) : (
          <div className="feed-panel__placeholder">
            <ScanEye size={28} />
            <span>비전 스트림 없음</span>
            <small>streamUrl prop 주입 시 표시</small>
          </div>
        )}

        {/* bbox 오버레이 (정규화 좌표 → %) */}
        {boxes.map((b, i) => (
          <div
            key={i}
            className="feed-panel__bbox"
            style={{
              left: `${b.x * 100}%`,
              top: `${b.y * 100}%`,
              width: `${b.w * 100}%`,
              height: `${b.h * 100}%`,
            }}
          >
            {b.label && <span className="feed-panel__bbox-label">{b.label}</span>}
          </div>
        ))}
      </div>
    </div>
  );
}
