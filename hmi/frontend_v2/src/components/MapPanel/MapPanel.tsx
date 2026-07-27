// [HMI v2] 자유 클릭 내비게이션 지도. Nav2 occupancy grid 위에 carter1·carter2 실시간
// 위치를 아이콘으로 표시한다. 지도를 여러 번 클릭해 "웨이포인트 경로"(드래프트)를 만들고,
// "경로 시작"을 눌러야만 routeQueue 에 등록되어 로봇이 순서대로 이동한다(즉시 이동 금지 —
// 안전 UX). 진행 중인 경로는 실선/구간 마커로, 완료 구간은 회색으로 표시.
// 🔧 튜닝: 로봇 색은 더 이상 고정이 아니라 "현재 배정된 작업"에서 유도(colorForCarter 참고).
//   미배정이면 중립색(--state-idle).
import { useEffect, useRef, useState, type MouseEvent } from "react";
import { Navigation2, Route, X, Home } from "lucide-react";
import { CARTER_IDS, CARTER_META } from "../../constants/carters";
import { commands } from "../../lib/commands";
import { TASK_META } from "../../constants/tasks";
import { useMapInfo } from "../../hooks/useMapInfo";
import { useRobotPose } from "../../hooks/useRobotPose";
import { useRouteQueue } from "../../hooks/useRouteQueue";
import { useRobotStatusContext } from "../../context/RobotStatusContext";
import { enqueueRoute, cancelRoute } from "../../lib/routeQueue";
import type { CarterId, MapInfo } from "../../types";
import type { Pose } from "../../lib/robotPoseStore";
import "./MapPanel.css";

const ACCENT: Record<"waste" | "disinfect", string> = {
  waste: "#b3433f",
  disinfect: "#2f6690",
};
const NEUTRAL_COLOR = "#888780"; // --state-idle — 작업 미배정 로봇
const DRAFT_COLOR = "#1c8577";
const CANVAS_FONT = "10px 'IBM Plex Mono', monospace";
// ★수동제어 = 대기 + 도킹스테이션일 때만★ : 19_ 는 MANUAL_OVERRIDE 로 제너레이터를 스왑하는
// 순간 위치와 무관하게 "대기"를 발행하므로(g_task_select_mission 최상단), processStateLabel
// 만으론 "지금 도킹에 있는지"를 보장 못 한다 — 실좌표까지 같이 확인해야 함. 오차 허용치는
// 도킹 복귀 시 실측된 x축 편차(~0.35m, g_nav_to_home 은 오픈루프라 반복보정 없음)보다 넉넉하게.
const DOCK_PROXIMITY_TOLERANCE_M = 0.6;
// ★긴급정지 중에는 위치 무관하게 수동제어 허용★ : 19_ 의 EMERGENCY_STOP 핸들러가 이 라벨을
// publish_hmi_state 로 발행한다(코드 "긴급정지" → 와이어 "RUNNING:긴급정지 중" → 백엔드가
// state 부분 벗겨내 payload="긴급정지 중"만 옴 — startsWith 로 접두만 비교).
const ESTOP_LABEL_PREFIX = "긴급정지";

function isAtDock(carterId: CarterId, pose: Pose | null): boolean {
  if (!pose) return false;
  const dock = CARTER_META[carterId].dock;
  return Math.hypot(pose.x - dock.x, pose.y - dock.y) <= DOCK_PROXIMITY_TOLERANCE_M;
}

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

interface DraftPoint {
  x: number;
  y: number;
}

export function MapPanel() {
  const { mapInfo, imageUrl } = useMapInfo();
  const poses = useRobotPose();
  const routes = useRouteQueue();
  const { snapshots } = useRobotStatusContext();
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imgRef = useRef<HTMLImageElement | null>(null);
  const [selected, setSelected] = useState<CarterId>("carter1");
  const [draftPoints, setDraftPoints] = useState<DraftPoint[]>([]);
  const [imgLoaded, setImgLoaded] = useState(false);

  // [HMI v2] 로봇 색 = 현재 배정된 작업의 색(미배정이면 중립색).
  const colorForCarter = (id: CarterId): string => {
    const task = snapshots[id]?.task;
    return task ? ACCENT[TASK_META[task].variant] : NEUTRAL_COLOR;
  };

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

  // 캔버스 렌더 — 맵 배경 + 경로(양쪽 로봇) + 로봇 아이콘 + 드래프트(선택 로봇, 미확정).
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

    // 0) 도킹스테이션 마커(양쪽 로봇) — 집 모양 + "도킹" 라벨. 작업 완료·복귀 지점.
    for (const id of CARTER_IDS) {
      const dock = CARTER_META[id].dock;
      const { px, py } = worldToPixel(dock.x, dock.y, mapInfo);
      const color = colorForCarter(id);
      ctx.beginPath();
      ctx.rect(px - 6, py - 4, 12, 10);
      ctx.globalAlpha = 0.85;
      ctx.fillStyle = color;
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.beginPath(); // 지붕
      ctx.moveTo(px - 8, py - 4);
      ctx.lineTo(px, py - 11);
      ctx.lineTo(px + 8, py - 4);
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.fill();
      ctx.stroke();
      ctx.fillStyle = color;
      ctx.font = CANVAS_FONT;
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText("도킹", px, py + 16);
    }

    // 1) 등록된 웨이포인트 경로(양쪽 로봇 — 진행 중이면 항상 표시)
    for (const id of CARTER_IDS) {
      const wps = routes[id];
      if (wps.length === 0) continue;
      const color = colorForCarter(id);
      const pts = wps.map((w) => worldToPixel(w.x, w.y, mapInfo));

      ctx.beginPath();
      pts.forEach((p, i) => (i === 0 ? ctx.moveTo(p.px, p.py) : ctx.lineTo(p.px, p.py)));
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      ctx.stroke();
      ctx.setLineDash([]);

      wps.forEach((w, i) => {
        const { px, py } = pts[i];
        const radius = w.status === "active" ? 9 : 7;
        ctx.beginPath();
        ctx.arc(px, py, radius, 0, Math.PI * 2);
        if (w.status === "done") {
          ctx.fillStyle = "#9aa5ab";
          ctx.fill();
        } else if (w.status === "active") {
          ctx.fillStyle = color;
          ctx.fill();
          ctx.strokeStyle = "#fff";
          ctx.lineWidth = 2;
          ctx.stroke();
        } else {
          ctx.fillStyle = "#fff";
          ctx.fill();
          ctx.strokeStyle = color;
          ctx.lineWidth = 2;
          ctx.stroke();
        }
        ctx.fillStyle = w.status === "queued" ? color : "#fff";
        ctx.font = CANVAS_FONT;
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(String(i + 1), px, py);
      });
    }

    // 2) 실시간 로봇 위치 아이콘
    for (const id of CARTER_IDS) {
      const pose = poses[id];
      if (!pose) continue;
      const { px, py } = worldToPixel(pose.x, pose.y, mapInfo);
      const color = colorForCarter(id);
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
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(16, 0);
      ctx.strokeStyle = color;
      ctx.lineWidth = 3;
      ctx.stroke();
      ctx.restore();
    }

    // 3) 드래프트 경로(선택된 로봇, 아직 미확정 — 클릭할 때마다 점 추가)
    if (draftPoints.length > 0) {
      const pts = draftPoints.map((p) => worldToPixel(p.x, p.y, mapInfo));
      ctx.beginPath();
      pts.forEach((p, i) => (i === 0 ? ctx.moveTo(p.px, p.py) : ctx.lineTo(p.px, p.py)));
      ctx.strokeStyle = DRAFT_COLOR;
      ctx.lineWidth = 2;
      ctx.setLineDash([4, 3]);
      ctx.stroke();
      ctx.setLineDash([]);

      pts.forEach((p, i) => {
        ctx.beginPath();
        ctx.arc(p.px, p.py, 9, 0, Math.PI * 2);
        ctx.fillStyle = DRAFT_COLOR;
        ctx.fill();
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.fillStyle = "#fff";
        ctx.font = CANVAS_FONT;
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(String(i + 1), p.px, p.py);
      });
    }
  }, [mapInfo, poses, routes, draftPoints, imgLoaded, snapshots]);

  // ★수동제어 게이팅★ : 선택된 로봇이 (a) "대기" 상태 + 도킹스테이션 근처, 또는 (b) 긴급정지
  // 중일 때만 지도 클릭으로 움직일 수 있다(사용자 결정) — 작업 중인데 그냥 조작하면 충돌/작업
  // 꼬임 위험이 있지만, 긴급정지로 이미 멈춰둔 다음이라면 위치와 무관하게 운영자가 수동으로
  // 다시 움직여줘야 하므로 예외로 허용.
  const selectedIdleAndDocked =
    snapshots[selected]?.state === "idle" && isAtDock(selected, poses[selected]);
  const selectedEstopped =
    (snapshots[selected]?.processStateLabel ?? "").startsWith(ESTOP_LABEL_PREFIX);
  const selectedCanManualControl = selectedIdleAndDocked || selectedEstopped;

  const handleCanvasClick = (e: MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas || !mapInfo || !selectedCanManualControl) return;
    const rect = canvas.getBoundingClientRect();
    // 캔버스 표시 스케일(CSS 크기)과 내부 픽셀 해상도가 다를 수 있어 보정.
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const px = (e.clientX - rect.left) * scaleX;
    const py = (e.clientY - rect.top) * scaleY;
    const { x, y } = pixelToWorld(px, py, mapInfo);
    setDraftPoints((prev) => [...prev, { x, y }]);
  };

  const startRoute = () => {
    if (draftPoints.length === 0 || !selectedCanManualControl) return;
    enqueueRoute(
      selected,
      draftPoints.map((p) => ({ x: p.x, y: p.y, yaw: 0 })),
    );
    setDraftPoints([]);
  };

  const selectedRoute = routes[selected];
  const selectedRouteActive = selectedRoute.length > 0;
  const doneCount = selectedRoute.filter((w) => w.status === "done").length;
  const routeFinished = selectedRouteActive && doneCount === selectedRoute.length;

  return (
    <div className="panel map-panel">
      <div className="map-panel__head">
        <span className="panel-title">
          <Navigation2 size={14} /> 내비게이션 지도
        </span>
        <div className="map-panel__head-right">
        <div className="map-panel__toggle">
          {CARTER_IDS.map((id) => {
            const meta = CARTER_META[id];
            const task = snapshots[id]?.task;
            const variant = task ? TASK_META[task].variant : undefined;
            const active = selected === id;
            return (
              <button
                key={id}
                type="button"
                className={`map-panel__toggle-btn${variant ? ` map-panel__toggle-btn--${variant}` : ""}${
                  active ? " map-panel__toggle-btn--active" : ""
                }`}
                onClick={() => setSelected(id)}
              >
                {meta.label}
              </button>
            );
          })}
        </div>
        {/* ★도킹 복귀★ : 선택 로봇의 진행 중 작업을 초기화하고 도킹스테이션으로 복귀시킨다.
            진행 중이던 수동 경로(routeQueue)를 먼저 취소해야 함 — 안 그러면 남은 웨이포인트가
            LEG_TIMEOUT_MS 뒤에 뒤늦게 재발행돼 도킹 goal_pose 를 덮어쓸 수 있음. */}
        <button
          type="button"
          className="map-panel__dock-btn"
          title={`${CARTER_META[selected].label} 도킹스테이션으로 복귀(작업 초기화)`}
          onClick={() => {
            cancelRoute(selected);
            commands.dockReturn(selected);
          }}
        >
          <Home size={13} /> 도킹 복귀
        </button>
        </div>
      </div>

      <div className="map-panel__stage">
        {!mapInfo ? (
          <div className="map-panel__placeholder">
            <Navigation2 size={28} />
            <span>지도 정보 로딩 중…</span>
          </div>
        ) : (
          <canvas
            ref={canvasRef}
            className={`map-panel__canvas${selectedCanManualControl ? "" : " map-panel__canvas--locked"}`}
            onClick={handleCanvasClick}
          />
        )}
      </div>

      <div className="map-panel__toolbar">
        {selectedRouteActive && (
          <div className="map-panel__route-status">
            <span>
              {CARTER_META[selected].label}{" "}
              {routeFinished
                ? `경로 완료 (${doneCount}/${selectedRoute.length})`
                : `경로 진행 중 (${doneCount}/${selectedRoute.length})`}
            </span>
            <button
              type="button"
              className="map-panel__route-cancel"
              onClick={() => cancelRoute(selected)}
            >
              <X size={12} /> {routeFinished ? "지우기" : "경로 취소"}
            </button>
          </div>
        )}

        {draftPoints.length > 0 ? (
          <div className="map-panel__draft-bar">
            <span className="map-panel__draft-count">
              {draftPoints.length}개 지점 추가됨
            </span>
            <button
              type="button"
              className="map-panel__draft-undo"
              onClick={() => setDraftPoints((prev) => prev.slice(0, -1))}
            >
              마지막 취소
            </button>
            <button
              type="button"
              className="map-panel__draft-clear"
              onClick={() => setDraftPoints([])}
            >
              초기화
            </button>
            <button type="button" className="map-panel__draft-start" onClick={startRoute}>
              <Route size={13} /> 경로 시작 ({draftPoints.length})
            </button>
          </div>
        ) : (
          !selectedRouteActive &&
          mapInfo &&
          (selectedCanManualControl ? (
            <p className="map-panel__hint">
              지도를 클릭해 {CARTER_META[selected].label}의 이동 경로를 만드세요(여러 지점 가능)
            </p>
          ) : (
            <p className="map-panel__hint map-panel__hint--locked">
              {CARTER_META[selected].label}이(가) 도킹스테이션에서 대기 중이거나 긴급정지 상태일
              때만 수동 제어할 수 있습니다
            </p>
          ))
        )}
      </div>
    </div>
  );
}
