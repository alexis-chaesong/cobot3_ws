// 관리자 UI 액션 로그(uiActionLog.ts) 구독 훅. LogPanel에서 사용.
import { useEffect, useState } from "react";
import { subscribeUiActionLog } from "../lib/uiActionLog";
import type { UiActionLogEntry } from "../types";

export function useUiActionLog(): UiActionLogEntry[] {
  const [entries, setEntries] = useState<UiActionLogEntry[]>([]);
  useEffect(() => subscribeUiActionLog(setEntries), []);
  return entries;
}
