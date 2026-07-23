// ══════════════════════════════════════════════════════════════
// useQueueStatus — 작업 큐 상태. 목 모드면 fixtures.
// 실제 백엔드에 큐 조회 엔드포인트가 생기면 useLogs와 동일 패턴으로 확장.
// ══════════════════════════════════════════════════════════════

import { useState } from "react";
import { MOCK } from "../lib/mock/mockSocket";
import { MOCK_QUEUE } from "../lib/mock/fixtures";
import type { QueueItem } from "../types";

export function useQueueStatus(): QueueItem[] {
  // 목 모드: 정적 큐. 실제 모드: (엔드포인트 미정) 빈 배열로 시작.
  const [queue] = useState<QueueItem[]>(MOCK ? MOCK_QUEUE : []);
  return queue;
}
