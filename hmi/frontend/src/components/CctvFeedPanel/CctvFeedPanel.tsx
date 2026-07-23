// CCTV(고정 카메라) 패널. Vision과 동일 레이아웃, 오버레이 없음.
// 스타일은 VisionFeedPanel.css를 공유한다.
import { Cctv } from "lucide-react";
import "../VisionFeedPanel/VisionFeedPanel.css";

interface Props {
  title?: string;
  streamUrl?: string;
}

export function CctvFeedPanel({ title = "CCTV", streamUrl }: Props) {
  return (
    <div className="panel feed-panel">
      <div className="feed-panel__head">
        <span className="panel-title">
          <Cctv size={14} /> {title}
        </span>
      </div>
      <div className="feed-panel__stage">
        {streamUrl ? (
          <img className="feed-panel__img" src={streamUrl} alt="cctv feed" />
        ) : (
          <div className="feed-panel__placeholder">
            <Cctv size={28} />
            <span>CCTV 스트림 없음</span>
            <small>streamUrl prop 주입 시 표시</small>
          </div>
        )}
      </div>
    </div>
  );
}
