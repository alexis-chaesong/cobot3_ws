// ══════════════════════════════════════════════════════════════
// useQueueStatus — 작업 큐 상태. routeQueue.ts(관리자가 지도에서 만든 웨이포인트 경로)를
// carter1/carter2 별 QueueItem[] 로 변환해서 보여준다(mock/실모드 공통 — 둘 다 같은
// routeQueue 모듈을 쓰므로 별도 분기 불필요).
// ══════════════════════════════════════════════════════════════

import { CARTER_IDS, CARTER_META } from "../constants/carters";
import { useRouteQueue } from "./useRouteQueue";
import type { QueueItem } from "../types";

export function useQueueStatus(): QueueItem[] {
  const routes = useRouteQueue();
  const items: QueueItem[] = [];

  for (const carterId of CARTER_IDS) {
    const meta = CARTER_META[carterId];
    routes[carterId].forEach((wp, i) => {
      items.push({
        taskId: wp.id,
        variant: meta.variant,
        label: `${meta.label} · 지점 ${i + 1} (${wp.x.toFixed(1)}, ${wp.y.toFixed(1)})`,
        status: wp.status,
      });
    });
  }

  return items;
}
