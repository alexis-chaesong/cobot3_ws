// 자유 클릭 내비게이션 지도. Nav2 occupancy grid 위에 carter1(소독/blue)·carter2(폐기물/red)
// 실시간 위치를 아이콘으로 표시하고, 클릭 → 미리보기 마커 → "이 위치로 이동" 확인 후에만
// commands.navigate 를 호출한다(즉시 이동 금지 — 안전 UX).
// 🔧 튜닝: 로봇 색은 CARTER_META.variant → tokens.css --waste-accent/--disinfect-accent.
import { useEffect, useRef, useState, type MouseEvent } from "react";
import { MapPin, Navigation2 } from "lucide-react";
import { CARTER_IDS, CARTER_META } from "../../constants/carters";
import { useMapInfo } from "../../hooks/useMapInfo";
import { useRobotPose } from "../../hooks/useRobotPose";
import { commands } from "../../lib/commands";
import type { CarterId, MapInfo } from "../../types";
import "./MapPanel.css";

const ACCENT: Record<"waste" | "disinfect", string> = {
  waste: "#b3433f",
  disinfect: "#2f6690",
};

function worldToPixel(x: number, y: number, mapInfo: MapInfo) {
  const px = (x - mapInfo.originX) / mapInfo.resolution;
  const py = mapInfo.height - (y - mapInfo.originY) / mapInfo.resolution;
  return { px, py };
}

// 클릭 좌표 변환: worldX=originX+px*res, worldY=originY+(height-py)*res (y축 반전).
function pixelToWorld(px: number, py: number, mapInfo: MapInfo) {
  const x = mapInfo.originX + px * mapInfo.resolution;
  const y = mapInfo.originY + (mapInfo.height - py) * mapInfo.resolution;
  return { x, y };
}

interface PreviewMarker {
  px: number;
  py: number;
  x: number;
  y: number;
}

export function MapPanel() {
  const { mapInfo, imageUrl } = useMapInfo();
  const poses = useRobotPose();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imgRef = useRef<HTMLImageElement | null>(null);
  const [selected, setSelected] = useState<CarterId>("carter1");
  const [preview, setPreview] = useState<PreviewMarker | null>(null);
  const [imgLoaded, setImgLoaded] = useState(false);
  const [sending, setSending] = useState(false);

  // 지도 이미지 로드(목 모드는 imageUrl=null → placeholder 배경 유지)
  useEffect(() => {
    if (!imageUrl) {
      setImgLoaded(false);
      return;
    }
    let cancelled = false;
    const img = new Image();
    img.onload = () => {
      if (cancelled) return;
      imgRef.current = img;
      setImgLoaded(true);
    };
    img.src = imageUrl;
    return () => {
      cancelled = true;
    };
  }, [imageUrl]);

  // 캔버스 렌더 — 맵 배경 + 로봇 아이콘 + 미리보기 마커.
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !mapInfo) return;
    canvas.width = mapInfo.width;
    canvas.height = mapInfo.height;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (imgLoaded && imgRef.current) {
      ctx.drawImage(imgRef.current, 0, 0, canvas.width, canvas.height);
    } else {
      ctx.fillStyle = "#dce3e7";
      ctx.fillRect(0, 0, canvas.width, canvas.height);
    }

    for (const id of CARTER_IDS) {
      const pose = poses[id];
      if (!pose) continue;
      const { px, py } = worldToPixel(pose.x, pose.y, mapInfo);
      const color = ACCENT[CARTER_META[id].variant];
      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(-pose.yaw); // 캔버스 y축이 world 와 반대라 부호 반전
      ctx.beginPath();
      ctx.arc(0, 0, 9, 0, Math.PI * 2);
      ctx.fillStyle = color;
      ctx.fill();
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 2;
      ctx.stroke();
      // 진행방향 화살표
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(16, 0);
      ctx.strokeStyle = color;
      ctx.lineWidth = 3;
      ctx.stroke();
      ctx.restore();
    }

    if (preview) {
      ctx.beginPath();
      ctx.arc(preview.px, preview.py, 10, 0, Math.PI * 2);
      ctx.fillStyle = "rgba(28, 133, 119, 0.25)";
      ctx.fill();
      ctx.strokeStyle = "#1c8577";
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(preview.px, preview.py - 15);
      ctx.lineTo(preview.px, preview.py + 15);
      ctx.moveTo(preview.px - 15, preview.py);
      ctx.lineTo(preview.px + 15, preview.py);
      ctx.strokeStyle = "#1c8577";
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
  }, [mapInfo, poses, preview, imgLoaded]);

  const handleCanvasClick = (e: MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas || !mapInfo) return;
    const rect = canvas.getBoundingClientRect();
    // 캔버스 표시 스케일(CSS 크기)과 내부 픽셀 해상도가 다를 수 있어 보정.
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const px = (e.clientX - rect.left) * scaleX;
    const py = (e.clientY - rect.top) * scaleY;
    const { x, y } = pixelToWorld(px, py, mapInfo);
    setPreview({ px, py, x, y });
  };

  const confirmMove = async () => {
    if (!preview || sending) return;
    setSending(true);
    try {
      await commands.navigate(selected, preview.x, preview.y, 0);
    } finally {
      setSending(false);
      setPreview(null);
    }
  };

  return (
    <div className="panel map-panel">
      <div className="map-panel__head">
        <span className="panel-title">
          <Navigation2 size={14} /> 내비게이션 지도
        </span>
        <div className="map-panel__toggle">
          {CARTER_IDS.map((id) => {
            const meta = CARTER_META[id];
            const active = selected === id;
            return (
              <button
                key={id}
                type="button"
                className={`map-panel__toggle-btn map-panel__toggle-btn--${meta.variant}${
                  active ? " map-panel__toggle-btn--active" : ""
                }`}
                onClick={() => setSelected(id)}
              >
                {meta.label}
              </button>
            );
          })}
        </div>
      </div>

      <div className="map-panel__stage">
        {!mapInfo ? (
          <div className="map-panel__placeholder">
            <Navigation2 size={28} />
            <span>지도 정보 로딩 중…</span>
          </div>
        ) : (
          <>
            <canvas
              ref={canvasRef}
              className="map-panel__canvas"
              onClick={handleCanvasClick}
            />
            {preview && (
              <div
                className="map-panel__confirm"
                style={{
                  left: `${(preview.px / mapInfo.width) * 100}%`,
                  top: `${(preview.py / mapInfo.height) * 100}%`,
                }}
              >
                <div className="map-panel__confirm-coord">
                  <MapPin size={12} /> ({preview.x.toFixed(2)}, {preview.y.toFixed(2)})
                </div>
                <div className="map-panel__confirm-actions">
                  <button
                    type="button"
                    className="map-panel__confirm-go"
                    disabled={sending}
                    onClick={confirmMove}
                  >
                    이 위치로 이동
                  </button>
                  <button
                    type="button"
                    className="map-panel__confirm-cancel"
                    disabled={sending}
                    onClick={() => setPreview(null)}
                  >
                    취소
                  </button>
                </div>
              </div>
            )}
          </>
        )}
      </div>
    </div>
  );
}
