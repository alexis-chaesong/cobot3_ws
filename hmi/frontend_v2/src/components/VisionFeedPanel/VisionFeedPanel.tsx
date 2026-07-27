// 비전(로봇 카메라 + 인식 결과) 패널.
// streamUrl이 없으면 placeholder, 있으면 <img>로 MJPEG(multipart/x-mixed-replace) 표시.
// bbox 오버레이 자리 포함(현재는 서버가 박스를 이미지에 직접 그려 보내므로 boxes 는 예비용).
//
// ★자동 재연결★ : <img> 기반 MJPEG는 연결이 한 번 끊기면(백엔드 재시작, 네트워크 순단 등)
// 브라우저가 스스로 재연결하지 않는다 — 그대로 두면 새로고침 전까진 화면이 죽어 있다.
// onError 시 RETRY_MS 후 캐시버스터를 붙여 새 <img>(key 로 강제 리마운트)를 다시 연결한다.
// 🔧 튜닝: 실제 스트림 연결 시 App에서 streamUrl prop만 주입하면 된다. RETRY_MS로 재연결 주기 조절.
import { useEffect, useState } from "react";
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

const RETRY_MS = 3000;

export function VisionFeedPanel({ title = "Vision", streamUrl, boxes = [] }: Props) {
  const [attempt, setAttempt] = useState(0);
  const [errored, setErrored] = useState(false);

  // streamUrl 자체가 바뀌면(로봇 전환 등) 재시도 상태 리셋
  useEffect(() => {
    setAttempt(0);
    setErrored(false);
  }, [streamUrl]);

  const handleError = () => {
    setErrored(true);
    setTimeout(() => {
      setErrored(false);
      setAttempt((n) => n + 1);
    }, RETRY_MS);
  };

  const src = streamUrl
    ? `${streamUrl}${streamUrl.includes("?") ? "&" : "?"}retry=${attempt}`
    : undefined;

  return (
    <div className="panel feed-panel">
      <div className="feed-panel__head">
        <span className="panel-title">
          <ScanEye size={14} /> {title}
        </span>
      </div>

      <div className="feed-panel__stage">
        {src && !errored ? (
          <img
            key={attempt}
            className="feed-panel__img"
            src={src}
            alt="vision feed"
            onError={handleError}
          />
        ) : (
          <div className="feed-panel__placeholder">
            <ScanEye size={28} />
            <span>{streamUrl ? "비전 스트림 재연결 중…" : "비전 스트림 없음"}</span>
            {!streamUrl && <small>streamUrl prop 주입 시 표시</small>}
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
