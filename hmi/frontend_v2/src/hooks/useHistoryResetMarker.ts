// historyResetMarker.ts 구독 훅 — 마커가 바뀌면(전체 작업 완료 시점) 리렌더돼
// useTaskQueue/useLogs 가 새 기준으로 다시 필터링하도록 한다.
import { useEffect, useState } from "react";
import {
  getHistoryResetMarker,
  subscribeHistoryResetMarker,
} from "../lib/historyResetMarker";

export function useHistoryResetMarker(): string | null {
  const [marker, setMarker] = useState<string | null>(getHistoryResetMarker);
  useEffect(() => subscribeHistoryResetMarker(setMarker), []);
  return marker;
}
