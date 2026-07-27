// 맵 메타데이터(GET /api/map-info) + 이미지 URL. MapPanel이 캔버스에 그릴 때 사용.
// 🔧 튜닝: MOCK_MAP_INFO 값은 modified_hospital_map.yaml 실측치와 동일하게 맞춰둠(백엔드 인수인계서 참조).
import { useEffect, useState } from "react";
import { apiClient } from "../lib/apiClient";
import { MOCK } from "../lib/mock/mockSocket";
import type { MapInfo } from "../types";

const API_BASE = import.meta.env.VITE_API_BASE;

// 실제 modified_hospital_map.yaml 값(png 720x520, resolution 0.05, origin [-13.975,-4.975]).
const MOCK_MAP_INFO: MapInfo = {
  resolution: 0.05,
  originX: -13.975,
  originY: -4.975,
  width: 720,
  height: 520,
};

interface UseMapInfoResult {
  mapInfo: MapInfo | null;
  imageUrl: string | null; // MOCK 모드에선 실제 이미지가 없어 null(placeholder 렌더)
  loading: boolean;
}

export function useMapInfo(): UseMapInfoResult {
  const [mapInfo, setMapInfo] = useState<MapInfo | null>(MOCK ? MOCK_MAP_INFO : null);
  const [loading, setLoading] = useState(!MOCK);

  useEffect(() => {
    if (MOCK) return; // 목 모드는 고정값 사용, fetch 불필요
    let cancelled = false;

    (async () => {
      const raw = await apiClient.get<{
        resolution: number;
        origin_x: number;
        origin_y: number;
        width: number;
        height: number;
      }>("/api/map-info");
      if (cancelled || !raw) return;
      setMapInfo({
        resolution: raw.resolution,
        originX: raw.origin_x,
        originY: raw.origin_y,
        width: raw.width,
        height: raw.height,
      });
      setLoading(false);
    })();

    return () => {
      cancelled = true;
    };
  }, []);

  return {
    mapInfo,
    imageUrl: MOCK ? null : `${API_BASE}/api/map-image`,
    loading,
  };
}
