"""로봇 2대 없이 백엔드만 테스트하는 ROS2 목 노드 (auto-dump-bot 패턴 그대로).

역할:
  - /robot/{waste,disinfect}/process_state 를 주기적으로 발행 → 백엔드 큐/브로드캐스트 검증
  - /robot/command 를 구독 → START / EMERGENCY_STOP 수신 로그 출력
  - 데모용으로 소독 로봇 safety_event 1회 발행

실행:
  ros2 run 대신 단독 실행도 가능:
    python3 mock_robot_node.py
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

ROBOT_IDS = ("waste", "disinfect")

WASTE_STEPS = [
    "대기", "전방 주행", "폐기물통 파지", "수거함 이동",
    "폐기물 투하", "수거통 원위치", "복귀",
]
DISINFECT_STEPS = [
    "대기", "복도 진입", "노즐 접촉", "소독 분사", "유턴 재분사", "복귀",
]
STEPS = {"waste": WASTE_STEPS, "disinfect": DISINFECT_STEPS}


class MockRobotNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_robot_node")

        self._state_pubs = {
            rid: self.create_publisher(String, f"/robot/{rid}/process_state", 10)
            for rid in ROBOT_IDS
        }
        self._safety_pubs = {
            rid: self.create_publisher(String, f"/robot/{rid}/safety_event", 10)
            for rid in ROBOT_IDS
        }
        self.create_subscription(String, "/robot/command", self._on_command, 10)

        self._idx = {rid: 0 for rid in ROBOT_IDS}
        self._ticks = 0
        self.create_timer(2.5, self._tick)  # 2.5초마다 상태 발행
        self.get_logger().info("mock_robot_node 시작")

    def _tick(self) -> None:
        self._ticks += 1
        for rid in ROBOT_IDS:
            steps = STEPS[rid]
            i = self._idx[rid]
            label = steps[i]
            # "RUNNING:라벨 중" 형식으로 발행 (백엔드 _split_colon이 파싱)
            payload = label if i == 0 else f"RUNNING:{label} 중"
            self._state_pubs[rid].publish(String(data=payload))
            self._idx[rid] = (i + 1) % len(steps)

        # 데모: 4틱째 소독 로봇 안전 이벤트
        if self._ticks == 4:
            self._safety_pubs["disinfect"].publish(
                String(data="ERR_COLLISION:노즐 접촉 중 예상치 못한 장애물 감지")
            )
            self.get_logger().warn("데모 SAFETY_EVENT 발행 (disinfect)")

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
