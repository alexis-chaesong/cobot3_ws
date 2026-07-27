"""[HMI v2] 로봇 2대(carter1/carter2) 없이 백엔드만 테스트하는 ROS2 목 노드.

기존 hmi/backend/mock_robot_node.py 는 "waste"/"disinfect" 역할고정으로 무조건 자동 순환했다.
19_ 은 task_select 메시지 수신이 곧 시작 트리거이므로, 이 목 노드도 실제처럼
/{carter_id}/task_select 를 구독해 작업을 받아야 그때부터 해당 STEPS 를 순환한다(대기 중엔
"대기" 만 발행) — 백엔드/프론트를 19_ 없이도 좀 더 사실적으로 검증할 수 있다.

역할:
  - /{carter1,carter2}/task_select 구독 → 수신 시 그 작업(trash|spray)의 STEPS 순환 시작
  - /{carter1,carter2}/process_state 를 주기적으로 발행 → 백엔드 큐/브로드캐스트 검증
  - /robot/command 를 구독 → START / EMERGENCY_STOP 수신 로그 출력
  - 데모용으로 carter1 safety_event 1회 발행

실행:
  ros2 run 대신 단독 실행도 가능:
    python3 mock_robot_node.py
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

CARTER_IDS = ("carter1", "carter2")

# 19_dual_task_select_yolo_integrated.py 의 publish_hmi_state 호출 라벨과 동일 어휘
# (HMI v2 계획 §3 표 참고).
TRASH_STEPS = [
    "대기", "전방 주행", "폐기물통 파지", "수거함 이동",
    "폐기물 투하", "수거통 원위치", "복귀",
]
SPRAY_STEPS = [
    "대기", "노즐 접촉", "노즐 장착", "복도 진입", "소독 분사", "복귀",
]
STEPS_BY_TASK = {"trash": TRASH_STEPS, "spray": SPRAY_STEPS}


class MockRobotNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_robot_node_v2")

        self._state_pubs = {
            cid: self.create_publisher(String, f"/{cid}/process_state", 10)
            for cid in CARTER_IDS
        }
        self._safety_pubs = {
            cid: self.create_publisher(String, f"/{cid}/safety_event", 10)
            for cid in CARTER_IDS
        }
        for cid in CARTER_IDS:
            self.create_subscription(
                String, f"/{cid}/task_select",
                lambda msg, c=cid: self._on_task_select(msg, c), 10,
            )
        self.create_subscription(String, "/robot/command", self._on_command, 10)

        # None = 대기 중(task 미배정), 문자열이면 진행 중인 STEPS 순환 인덱스.
        self._active_task: dict[str, str | None] = {cid: None for cid in CARTER_IDS}
        self._idx: dict[str, int] = {cid: 0 for cid in CARTER_IDS}
        self._ticks = 0
        self.create_timer(2.5, self._tick)  # 2.5초마다 상태 발행
        self._publish_idle_all()
        self.get_logger().info("mock_robot_node(v2) 시작 — task_select 대기 중")

    def _publish_idle_all(self) -> None:
        for cid in CARTER_IDS:
            self._state_pubs[cid].publish(String(data="대기"))

    def _on_task_select(self, msg: String, carter_id: str) -> None:
        task = str(msg.data).strip().lower()
        if task not in STEPS_BY_TASK:
            self.get_logger().warn(f"[{carter_id}] 알 수 없는 task_select: {task!r}")
            return
        self._active_task[carter_id] = task
        self._idx[carter_id] = 1  # 0번(대기)은 이미 지났으니 다음 단계부터
        self.get_logger().info(f"[{carter_id}] task_select 수신 = '{task}' → STEPS 순환 시작")

    def _tick(self) -> None:
        self._ticks += 1
        for cid in CARTER_IDS:
            task = self._active_task[cid]
            if task is None:
                continue  # 대기 중 — 이미 "대기" 발행돼 있음, 재발행 불필요(dedup 과 동일 원칙)
            steps = STEPS_BY_TASK[task]
            i = self._idx[cid]
            label = steps[i]
            payload = "대기" if label == "대기" else f"RUNNING:{label} 중"
            self._state_pubs[cid].publish(String(data=payload))
            if label == "복귀":
                # 마지막 단계 다음엔 다시 대기로 복귀(19_ 의 IDLE 재진입과 동일한 흐름).
                self._active_task[cid] = None
                self._idx[cid] = 0
                self._state_pubs[cid].publish(String(data="대기"))
            else:
                self._idx[cid] = min(i + 1, len(steps) - 1)

        # 데모: 4틱째 carter1 안전 이벤트(진행 중일 때만 의미 있지만 데모 목적으로 무조건 발행)
        if self._ticks == 4:
            self._safety_pubs["carter1"].publish(
                String(data="ERR_COLLISION:소독 스윕 중 예상치 못한 장애물 감지")
            )
            self.get_logger().warn("데모 SAFETY_EVENT 발행 (carter1)")

    def _on_command(self, msg: String) -> None:
        try:
            body = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"명령 파싱 실패: {msg.data!r}")
            return
        self.get_logger().info(
            f"명령 수신: command={body.get('command')} robotId={body.get('robotId')}"
        )


def main() -> None:
    rclpy.init()
    node = MockRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
