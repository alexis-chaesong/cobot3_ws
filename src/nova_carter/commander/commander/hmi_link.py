#!/usr/bin/env python3
"""
hmi_link.py — 미션 노드 ↔ HMI 백엔드 브리지 헬퍼 (계측 레이어)
============================================================================
역할 : 튜닝 완료된 Nav2 미션 노드(spray/trash)에 "웹 HMI 연동"만 얇게 얹는다.
  (1) 진행 단계를 /robot/{robot_id}/process_state (std_msgs/String) 로 발행
      → HMI 백엔드(robot_bridge.py)가 구독 → WebSocket → 리액트 FlowStepRail.
  (2) /robot/command (JSON) 를 구독 → START / EMERGENCY_STOP 를 미션에 전달.
      - START          : self.active = True  (미션 진행 게이트 개방)
      - EMERGENCY_STOP : self.active = False + self.estop = True + on_estop() 콜백
                         (미션이 Nav2 goal 취소 + cmd_vel 0 발행)

HMI robotId ↔ 실제 로봇 매핑
  - "disinfect" = 소독 로봇  = carter1 (spray_waypoint_mission)
  - "waste"     = 폐기물 로봇 = carter2 (trash_can_nav_pick_mission)

토픽 계약 (HMI 백엔드 spec v2 그대로)
  발행 : /robot/{robot_id}/process_state  "STATE:사람이 읽는 라벨"  (STATE_KO_MAP/_split_colon)
         /robot/{robot_id}/safety_event    "ERR_CODE:메시지"
  구독 : /robot/command  {"command": "START|EMERGENCY_STOP", "robotId": "waste|disinfect|null", ...}

주의(단일스레드 협조 모델)
  - 미션 노드는 spin_once/spin_until_future_complete 로 이미 노드를 spin 하므로,
    _on_command 콜백은 그 spin 중에 자연히 호출된다(별도 스레드/executor 불필요).
  - 그래서 콜백 안에서는 절대 spin 하지 않는다(재진입 금지). publish 는 spin 없이 즉시 나간다.
    stop_wheels() 도 spin 없이 publish + time.sleep 만 사용한다.
============================================================================
"""
import json
import time

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class HmiLink:
    def __init__(self, node, robot_id, cmd_vel_topic, on_estop=None, on_start=None):
        self.node = node
        self.robot_id = robot_id            # "waste" | "disinfect"
        self._on_estop = on_estop
        self._on_start = on_start

        # 미션 진행 게이트/상태 플래그 (미션 루프가 폴링)
        self.active = False                 # START 수신 시 True
        self.estop = False                  # 마지막 명령이 긴급정지였는지 (엣지 소비는 호출부)
        self._last_label = None             # 동일 라벨 중복 발행 억제

        self._state_pub = node.create_publisher(
            String, f"/robot/{robot_id}/process_state", 10)
        self._safety_pub = node.create_publisher(
            String, f"/robot/{robot_id}/safety_event", 10)
        self._cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)
        node.create_subscription(String, "/robot/command", self._on_command, 10)

        node.get_logger().info(
            f"[HMI] robot_id='{robot_id}' "
            f"state_pub='/robot/{robot_id}/process_state' cmd_vel='{cmd_vel_topic}'")

    # ── 상태 발행 ────────────────────────────────────────────────
    def publish_state(self, label, *, force=False):
        """진행 단계 라벨을 발행. 백엔드 _split_colon 이 'STATE:message' 로 파싱한다.
        - 대기        → "대기"           (payload="대기", 프론트 deriveState=idle)
        - 그 외 단계  → "RUNNING:라벨 중" (payload="라벨 중", 프론트 running + 단계매칭)
        중복 라벨은 (force 아니면) 스킵해 브로드캐스트 노이즈를 줄인다."""
        if label == self._last_label and not force:
            return
        self._last_label = label
        data = "대기" if label == "대기" else f"RUNNING:{label} 중"
        self._state_pub.publish(String(data=data))

    def publish_safety(self, code, message):
        """안전 이벤트 발행 → 프론트 AlertBanner. 형식 'ERR_CODE:메시지'."""
        self._safety_pub.publish(String(data=f"{code}:{message}"))

    # ── 긴급정지: 바퀴 0속도 버스트 (spin 없이) ──────────────────
    def stop_wheels(self, n=12, dt=0.02):
        """cmd_vel 에 0속도 Twist 를 여러 번 발행해 즉시 정지시킨다.
        콜백 컨텍스트에서 호출되므로 spin 하지 않는다(publish 는 spin 불필요)."""
        zero = Twist()
        for _ in range(n):
            self._cmd_pub.publish(zero)
            time.sleep(dt)

    # ── /robot/command 수신 (spin 중 호출됨) ─────────────────────
    def _on_command(self, msg):
        try:
            body = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError, ValueError):
            self.node.get_logger().warn(f"[HMI] 명령 파싱 실패: {msg.data!r}")
            return
        cmd = body.get("command")
        rid = body.get("robotId")
        # robotId 가 None(전체) 이거나 내 id 일 때만 반응
        if rid not in (None, self.robot_id):
            return
        self.node.get_logger().info(f"[HMI] 명령 수신: {cmd} (robotId={rid})")
        if cmd == "START":
            self.estop = False
            self.active = True
            if self._on_start is not None:
                self._on_start()
        elif cmd == "EMERGENCY_STOP":
            self.estop = True
            self.active = False
            if self._on_estop is not None:
                self._on_estop()
