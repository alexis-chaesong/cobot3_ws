// ══════════════════════════════════════════════════════════════
// useQueueStatus — 작업 큐 상태. 두 소스를 합친다:
//   1) useTaskQueue: 로봇 자동 임무(진행 중/완료) — 백엔드 tb_task_history 기반, 평소
//      운영 중(수동 경로예약 없이도) 항상 채워지는 주 소스.
//   2) routeQueue.ts: 관리자가 지도에서 만든 수동 웨이포인트 경로(프론트 전용, 이벤트성).
// ══════════════════════════════════════════════════════════════

import { CARTER_IDS, CARTER_META } from "../constants/carters";
import { useRouteQueue } from "./useRouteQueue";
import { useTaskQueue } from "./useTaskQueue";
import type { QueueItem } from "../types";

export function useQueueStatus(): QueueItem[] {
  const routes = useRouteQueue();
  const taskItems = useTaskQueue();
  const items: QueueItem[] = [...taskItems];

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
