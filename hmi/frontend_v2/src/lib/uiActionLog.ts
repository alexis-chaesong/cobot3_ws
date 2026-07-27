// ══════════════════════════════════════════════════════════════
// uiActionLog.ts — 관리자가 UI에서 명령을 선택할 때마다(통합/개별 시작, 긴급정지,
// 자유클릭 내비 이동 등) 즉시 기록되는 로그. apiClient/mockSocket과 동일하게
// React Context 없이 모듈 싱글턴 + 구독자 패턴으로 관리한다.
// 🔧 튜닝: MAX_ENTRIES(보관 개수)만 조절하면 된다.
// ══════════════════════════════════════════════════════════════

import type { UiActionLogEntry } from "../types";

const MAX_ENTRIES = 100;

let entries: UiActionLogEntry[] = [];
type Listener = (entries: UiActionLogEntry[]) => void;
const listeners = new Set<Listener>();

function notify() {
  for (const listener of listeners) listener(entries);
}

/** 관리자가 UI에서 명령을 선택한 순간 호출. commands.ts 각 함수에서 호출한다. */
export function logUiAction(targetLabel: string, message: string): void {
  const entry: UiActionLogEntry = {
    kind: "ui",
    id: `ui-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
    targetLabel,
    level: "info",
    message,
    timestamp: new Date().toISOString(),
  };
  entries = [entry, ...entries].slice(0, MAX_ENTRIES);
  notify();
}

/** LogPanel/useUiActionLog 이 구독. 즉시 현재값 1회 전달 + 이후 변경 시마다 통지. */
export function subscribeUiActionLog(listener: Listener): () => void {
  listeners.add(listener);
  listener(entries);
  return () => listeners.delete(listener);
}
