// ══════════════════════════════════════════════════════════════
// historyResetMarker.ts — [HMI v2 신규] "전체 작업 완료"(두 로봇 다 대기 상태) 시점을
// 기록하는 모듈 싱글턴(uiActionLog.ts와 동일 pub/sub 스타일).
//
// DB(tb_task_history)는 그대로 두고(사용자 결정 — 이력 보존), 이 시각 이전의 작업 큐/로그
// 항목만 화면에서 걸러낸다(useTaskQueue/useLogs 가 이 마커보다 오래된 행을 숨김). 새로고침
// 하면 마커가 사라져 다시 DB 전체가 보인다(인메모리 상태, 영속화 안 함 — 화면만 치우는
// 목적이라 이 정도면 충분하다는 사용자 결정).
// 🔧 튜닝: 트리거 지점은 RobotStatusContext.tsx(두 로봇 모두 idle 전환 감지)에서 호출.
// ══════════════════════════════════════════════════════════════

let resetAt: string | null = null;
type Listener = (resetAt: string | null) => void;
const listeners = new Set<Listener>();

function notify() {
  for (const listener of listeners) listener(resetAt);
}

/** 지금 시각을 새 기준점으로 기록 — 이 시각 이전 항목은 화면에서 숨겨진다. */
export function markHistoryReset(): void {
  resetAt = new Date().toISOString();
  notify();
}

export function getHistoryResetMarker(): string | null {
  return resetAt;
}

export function subscribeHistoryResetMarker(listener: Listener): () => void {
  listeners.add(listener);
  listener(resetAt);
  return () => listeners.delete(listener);
}

/** 마커 이전 시각(ISO 문자열 또는 null)이면 화면에서 숨겨야 하는지 판정하는 공용 헬퍼. */
export function isBeforeHistoryReset(iso: string | null | undefined): boolean {
  if (!resetAt || !iso) return false;
  const t = new Date(iso).getTime();
  return !Number.isNaN(t) && t <= new Date(resetAt).getTime();
}
