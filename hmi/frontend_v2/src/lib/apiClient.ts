// ══════════════════════════════════════════════════════════════
// apiClient.ts — auto-dump-bot의 api/api_base.py 포팅.
//   - _post()            → post()
//   - start_websocket_listener() → connectWebSocket() (3초 재연결)
// 통신 실패는 이전 프로젝트와 동일하게 "조용히" 처리하고, 호출부가 판단한다.
// ══════════════════════════════════════════════════════════════

import type { WsMessage } from "../types";

export class RobotApiClient {
  constructor(
    private baseUrl: string = import.meta.env.VITE_API_BASE,
    private wsUrl: string = import.meta.env.VITE_WS_URL,
  ) {}

  /** REST POST. 실패 시 null 반환 (예외를 던지지 않음). */
  async post(endpoint: string, data?: unknown): Promise<Response | null> {
    try {
      return await fetch(`${this.baseUrl}${endpoint}`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: data ? JSON.stringify(data) : undefined,
      });
    } catch {
      return null; // 통신 실패는 조용히 null. (이전 프로젝트와 동일)
    }
  }

  /** REST GET (history/errors 조회용). 실패 시 null. */
  async get<T>(endpoint: string): Promise<T | null> {
    try {
      const res = await fetch(`${this.baseUrl}${endpoint}`);
      if (!res.ok) return null;
      return (await res.json()) as T;
    } catch {
      return null;
    }
  }

  /**
   * WebSocket 구독. onclose 시 3초 후 자동 재연결(이전 프로젝트와 동일).
   * 반환값(cleanup)을 호출하면 재연결을 멈추고 소켓을 닫는다.
   */
  connectWebSocket(onMessage: (data: WsMessage) => void): () => void {
    let ws: WebSocket | null = null;
    let retryTimer: ReturnType<typeof setTimeout> | null = null;
    let closedByUs = false;

    const connect = () => {
      ws = new WebSocket(this.wsUrl);
      ws.onmessage = (e) => {
        try {
          onMessage(JSON.parse(e.data) as WsMessage);
        } catch {
          /* 파싱 실패 메시지는 무시 */
        }
      };
      ws.onclose = () => {
        if (closedByUs) return;
        retryTimer = setTimeout(connect, 3000); // 3초 재연결
      };
      ws.onerror = () => ws?.close();
    };

    connect();

    return () => {
      closedByUs = true;
      if (retryTimer) clearTimeout(retryTimer);
      ws?.close();
    };
  }
}

// 앱 전역에서 공유하는 단일 인스턴스
export const apiClient = new RobotApiClient();
