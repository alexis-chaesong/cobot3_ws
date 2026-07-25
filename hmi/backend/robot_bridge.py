"""ROS2(스레드) ↔ asyncio.Queue 브리지 — 로봇 2대 대응 확장.

핵심 원칙(auto-dump-bot 그대로):
  - 이 모듈은 큐에 데이터를 '적재만' 하고 WebSocket을 전혀 모른다.
  - 스레드 → asyncio 경계는 loop.call_soon_threadsafe 로만 넘는다.
  - 큐가 가득 차면 오래된 것부터 버린다(링버퍼).

토픽 네이밍:
  - 구독: /robot/{robot_id}/process_state, /robot/{robot_id}/safety_event
  - 발행: /robot/command (payload 안 robot_id로 대상 구분)
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import threading
import time
from typing import Optional

from config import CARTER_IDS, ROBOT_IDS, settings

logger = logging.getLogger(__name__)

# "state:message" 형태를 분리하고, 상태코드를 한글로 매핑
STATE_KO_MAP: dict[str, str] = {
    "IDLE": "대기",
    "RUNNING": "동작 중",
    "DONE": "완료",
}


def _split_colon(data: str) -> tuple[str, str]:
    state, _, message = data.partition(":")
    return state.strip(), message.strip()


class RobotBridgeManager:
    """ROS2 노드를 별도 스레드에서 spin 하며, 콜백 데이터를 asyncio 큐로 넘긴다."""

    def __init__(self) -> None:
        self.node = None
        self.command_pub = None
        self.goal_pub: dict = {}      # carter_id → PoseStamped 발행기 (/carterN/goal_pose)
        self._fastapi_loop: Optional[asyncio.AbstractEventLoop] = None
        self._output_queue: Optional[asyncio.Queue] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._executor = None

    # ── 시작/종료 ─────────────────────────────────────────────
    def start_bridge(
        self,
        fastapi_loop: asyncio.AbstractEventLoop,
        output_queue: asyncio.Queue,
    ) -> None:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

        self._fastapi_loop = fastapi_loop
        self._output_queue = output_queue

        if not rclpy.ok():
            rclpy.init()
        self.node = Node("hmi_robot_bridge")

        # 로봇마다 네임스페이스로 구독 (rid를 람다 기본인자로 고정)
        for robot_id in ROBOT_IDS:
            self.node.create_subscription(
                String,
                f"/robot/{robot_id}/process_state",
                lambda msg, rid=robot_id: self._process_state_callback(msg, rid),
                10,
            )
            self.node.create_subscription(
                String,
                f"/robot/{robot_id}/safety_event",
                lambda msg, rid=robot_id: self._safety_event_callback(msg, rid),
                10,
            )

        # 명령은 단일 토픽. payload 안에 robot_id 포함.
        self.command_pub = self.node.create_publisher(String, "/robot/command", 10)

        # ── 자유 클릭 내비게이션 (carter1/carter2 네임스페이스) ──
        #   구독: /carterN/amcl_pose (PoseWithCovarianceStamped) → ROBOT_POSE 브로드캐스트
        #   발행: /carterN/goal_pose (PoseStamped) — 운영자 수동 이동 전용.
        #   ★carter2 는 자동 픽업이 쓰는 /carter2/trash_can_nav_goal 과 ★다른 토픽★ 이라
        #     자유 클릭 goal 이 자동 픽업 goal 을 덮어쓰지 않는다(토픽 레벨 충돌 없음).
        for carter_id in CARTER_IDS:
            self.node.create_subscription(
                PoseWithCovarianceStamped,
                f"/{carter_id}/amcl_pose",
                lambda msg, cid=carter_id: self._amcl_pose_callback(msg, cid),
                10,
            )
            self.goal_pub[carter_id] = self.node.create_publisher(
                PoseStamped, f"/{carter_id}/goal_pose", 10
            )

        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()
        logger.info("RobotBridge 시작: state=%s, nav=%s", ROBOT_IDS, CARTER_IDS)

    def _spin_loop(self) -> None:
        import rclpy

        try:
            rclpy.spin(self.node)
        except Exception as exc:  # noqa: BLE001
            logger.warning("ROS2 spin 종료: %s", exc)

    def shutdown(self) -> None:
        import rclpy

        try:
            if self.node is not None:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:  # noqa: BLE001
            logger.warning("RobotBridge 종료 중 예외: %s", exc)

    # ── ROS 콜백 (스레드 컨텍스트) ────────────────────────────
    def _process_state_callback(self, msg, robot_id: str) -> None:
        state, message = _split_colon(msg.data)
        self._dispatch(
            {
                "type": "PROCESS_STATE",
                "robotId": robot_id,
                "payload": message or STATE_KO_MAP.get(state, state),
                "timestamp": time.time(),
            }
        )

    def _safety_event_callback(self, msg, robot_id: str) -> None:
        # 형식: "ERR_CODE:사람이 읽는 메시지"
        code, message = _split_colon(msg.data)
        self._dispatch(
            {
                "type": "SAFETY_EVENT",
                "robotId": robot_id,
                "error_code": code or "ERR_UNKNOWN",
                "error_msg": message,
                "timestamp": time.time(),
            }
        )

    def _amcl_pose_callback(self, msg, carter_id: str) -> None:
        """/carterN/amcl_pose (PoseWithCovarianceStamped) → ROBOT_POSE.
        quaternion 에서 yaw 만 뽑아 브로드캐스트(지도 아이콘 회전용)."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._dispatch(
            {
                "type": "ROBOT_POSE",
                "robotId": carter_id,
                "x": float(p.x),
                "y": float(p.y),
                "yaw": float(yaw),
                "timestamp": time.time(),
            }
        )

    # ── 명령 발행 ────────────────────────────────────────────
    def publish_command(
        self,
        command_type: str,
        robot_id: Optional[str] = None,
        payload: Optional[dict] = None,
    ) -> None:
        from std_msgs.msg import String

        if self.command_pub is None:
            logger.warning("command_pub 미초기화 — 명령 무시: %s", command_type)
            return
        body: dict = {
            "command": command_type,
            "robotId": robot_id,
            "timestamp": time.time(),
        }
        if payload is not None:
            body["payload"] = payload
        self.command_pub.publish(String(data=json.dumps(body, ensure_ascii=False)))

    def publish_nav_goal(
        self, robot_id: str, x: float, y: float, yaw: float = 0.0
    ) -> bool:
        """자유 클릭 이동 목표를 /{robot_id}/goal_pose 로 발행. robot_id 는 carter1/carter2.
        발행 성공 True, 발행기 없거나 미초기화면 False."""
        from geometry_msgs.msg import PoseStamped

        pub = self.goal_pub.get(robot_id)
        if pub is None:
            logger.warning("goal_pub 미초기화/미지원 robot_id — 무시: %s", robot_id)
            return False
        goal = PoseStamped()
        goal.header.frame_id = "map"
        if self.node is not None:
            goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.orientation.w = math.cos(float(yaw) / 2.0)
        pub.publish(goal)
        logger.info("nav goal → /%s/goal_pose (%.2f, %.2f, yaw=%.2f)", robot_id, x, y, yaw)
        return True

    # ── 스레드 → asyncio 경계 ────────────────────────────────
    def _dispatch(self, item: dict) -> None:
        """ROS 콜백(스레드)에서 호출. asyncio 루프로 안전하게 넘긴다."""
        if self._fastapi_loop is None:
            return
        self._fastapi_loop.call_soon_threadsafe(self._enqueue, item)

    def _enqueue(self, item: dict) -> None:
        """asyncio 루프 컨텍스트에서 실행. 큐가 가득 차면 오래된 것 폐기(링버퍼)."""
        if self._output_queue is None:
            return
        try:
            self._output_queue.put_nowait(item)
        except asyncio.QueueFull:
            try:
                self._output_queue.get_nowait()  # 가장 오래된 것 버리고
            except asyncio.QueueEmpty:
                pass
            try:
                self._output_queue.put_nowait(item)  # 새 것 적재
            except asyncio.QueueFull:
                pass


bridge_manager = RobotBridgeManager()

__all__ = ["bridge_manager", "RobotBridgeManager", "settings"]
