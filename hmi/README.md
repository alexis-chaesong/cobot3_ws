# 격리 병동 로봇 관제 HMI

폐기물 수거 로봇 1대 + 소독 로봇 1대를 실시간 모니터링·제어하는 **관리자 전용** 대시보드.
`auto-dump-bot`의 백엔드·통신 아키텍처(FastAPI + 비동기 큐 + WebSocket 브로드캐스트)를 재사용하며,
로봇이 1→2대로 늘고 프론트가 Tkinter→React로 바뀐 버전이다.

```
hmi/
  frontend/   React 18 + Vite + TypeScript (순수 CSS + CSS 변수)
  backend/    FastAPI + sqlite3(raw) + ROS2 브리지 + mock 노드
```

> ⚠️ 이 디렉토리는 colcon 소스 스페이스(`cobot3_ws/src/`)가 **아니라** 레포 루트에 둔다.
> `src/` 안에 두면 `colcon build`가 ROS 패키지로 오인한다.

---

## 1. 빠른 실행 (목 모드 — 백엔드 불필요)

```bash
cd hmi/frontend
npm install
npm run dev          # http://localhost:5173
```

`.env`의 `VITE_MOCK=true`가 기본값이라 백엔드 없이도 전체 화면이 동작한다.
두 로봇이 각자 플로우 단계를 진행하고, 몇 초 뒤 소독 로봇에서 데모 안전 이벤트가 떠 AlertBanner가 나타난다.
시작 버튼들은 콘솔에 `[MOCK] command → ...` 로그를 남긴다.

## 2. 실제 백엔드 연동

```bash
# 터미널 A — 백엔드
cd hmi/backend
pip install -r requirements.txt
uvicorn main:app --host 0.0.0.0 --port 8000

# 터미널 B — 로봇 없이 테스트할 때 mock ROS2 노드
source /opt/ros/humble/setup.bash   # 사용 중인 배포판에 맞게
python3 mock_robot_node.py

# 터미널 C — 프론트 (실제 모드)
cd hmi/frontend
# .env 에서 VITE_MOCK=false 로 변경
npm run dev
```

---

## 3. 어디를 건드려야 하나 (튜닝 맵)

| 하고 싶은 것 | 건드릴 파일 |
| --- | --- |
| 색/폰트 변경 | `frontend/src/styles/tokens.css` |
| 로봇 추가/이름 변경 | `frontend/src/constants/robots.ts` + `backend/config.py`(`ROBOT_IDS` 동기화) |
| 플로우 단계 수정 | `frontend/src/constants/steps.ts` + `backend/mock_robot_node.py` |
| 목 시나리오(속도·안전이벤트) | `frontend/src/lib/mock/mockSocket.ts` (`TICK_MS`, `SAFETY_DEMO`) |
| 목 로그/큐 더미데이터 | `frontend/src/lib/mock/fixtures.ts` |
| 상태(state) 판정 규칙 | `frontend/src/context/RobotStatusContext.tsx` (`deriveState`) |
| process_state → 단계 매핑 | 같은 파일 `resolveStepIndex` |
| REST 엔드포인트/주소 | `frontend/src/lib/commands.ts`, `frontend/.env` |
| WebSocket 재연결 간격 | `frontend/src/lib/apiClient.ts` (3000ms) |
| 비전/CCTV 실제 스트림 | `frontend/src/App.tsx` 에서 `streamUrl={...}` 주입 |
| 백엔드 토픽 이름 | `backend/robot_bridge.py` |
| DB 스키마 | `backend/database.py` |

## 4. 아키텍처 요약

- **프론트 상태 흐름**: `RobotStatusContext`가 WebSocket(또는 mockSocket)을 **한 번만** 구독 →
  `WsMessage`를 `RobotSnapshot`으로 변환 → Context로 배포 → 각 컴포넌트는 `useRobotStatus(id)`로 소비.
- **통신 2단 구조**(auto-dump-bot 포팅): `apiClient.ts`(공용 POST/GET + WS 재연결) ↔ `commands.ts`(역할별 명령).
- **백엔드 경계 분리**: `robot_bridge.py`(ROS 스레드, 큐 적재만) → `main._queue_consumer_loop`(변환+브로드캐스트) →
  `connection_manager.broadcast`(WS 캡슐화). 각 계층은 옆 계층을 모른다.
