"""ROS2(스레드) ↔ asyncio.Queue 브리지 — [HMI v2] carter1/carter2 단일 식별체계.

핵심 원칙(auto-dump-bot 그대로):
  - 이 모듈은 큐에 데이터를 '적재만' 하고 WebSocket을 전혀 모른다.
  - 스레드 → asyncio 경계는 loop.call_soon_threadsafe 로만 넘는다.
  - 큐가 가득 차면 오래된 것부터 버린다(링버퍼).

[HMI v2 변경] 기존 hmi/backend 는 ROBOT_IDS(waste/disinfect, 역할고정)로 process_state/
safety_event 를 구독하고 CARTER_IDS(carter1/carter2)로 pose/tf/vision/goal 을 구독하는
이원 체계였다. 19_dual_task_select_yolo_integrated.py 는 역할고정이 없으므로(로봇이 런타임에
task 를 스스로 고름) 전부 CARTER_IDS 하나로 통일한다.

토픽 네이밍:
  - 구독: /{carter_id}/process_state, /{carter_id}/safety_event
    (19_ 이 publish_hmi_state() 로 직접 발행 — hmi_link.py/구식 미션노드 HMI 경유 안 함)
  - 발행: /robot/command (payload 안 robotId 로 대상 구분 — 19_ 이 이 리터럴 토픽을
    이미 직접 구독하도록 병합돼 있어 이름을 그대로 유지)
  - 발행(신규): /{carter_id}/task_select (String, "trash"|"spray") — 로봇에 작업 배정.
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import threading
import time
import uuid
from datetime import datetime, timezone
from typing import Optional

from config import CARTER_IDS, settings
from database import get_connection

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
        self.task_pub: dict = {}      # [HMI v2 신규] carter_id → String 발행기 (/carterN/task_select)
        self._current_task: dict[str, Optional[str]] = {cid: None for cid in CARTER_IDS}
        self._fastapi_loop: Optional[asyncio.AbstractEventLoop] = None
        self._output_queue: Optional[asyncio.Queue] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._executor = None
        # ── YOLO 비전 스트림(웹 VisionFeedPanel 용) : carter_id → 최신 annotated JPEG bytes ──
        self._latest_frames: dict[str, bytes] = {}
        self._frame_lock = threading.Lock()
        # ── TF 기반 pose 폴백/주 소스 : carter_id → 전용 tf2_ros.Buffer ──
        self._tf_buffers: dict = {}
        self._pose_timer = None

    # ── 시작/종료 ─────────────────────────────────────────────
    def start_bridge(
        self,
        fastapi_loop: asyncio.AbstractEventLoop,
        output_queue: asyncio.Queue,
    ) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
        from sensor_msgs.msg import CompressedImage
        import tf2_ros
        from tf2_msgs.msg import TFMessage

        self._fastapi_loop = fastapi_loop
        self._output_queue = output_queue

        if not rclpy.ok():
            rclpy.init()
        self.node = Node("hmi_v2_robot_bridge")

        # [HMI v2] /robot/ 접두 없이 19_ 자신의 토픽 컨벤션(/carterN/...)에 맞춤 — 19_ 이
        # publish_hmi_state() 로 이 토픽에 직접 발행한다(구식 미션노드 HMI 는 hmi_enable:=False).
        for carter_id in CARTER_IDS:
            self.node.create_subscription(
                String,
                f"/{carter_id}/process_state",
                lambda msg, cid=carter_id: self._process_state_callback(msg, cid),
                10,
            )
            self.node.create_subscription(
                String,
                f"/{carter_id}/safety_event",
                lambda msg, cid=carter_id: self._safety_event_callback(msg, cid),
                10,
            )
            # [HMI v2 신규] 작업 배정 발행기 — 19_ 의 g_task_select_mission 이 소비.
            self.task_pub[carter_id] = self.node.create_publisher(
                String, f"/{carter_id}/task_select", 10
            )

        # 명령은 단일 토픽. payload 안에 robotId 포함(19_ 이 이 리터럴 토픽을 직접 구독).
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

        # ── TF 기반 pose 폴백(지도 아이콘 "처음엔 안 보이다가 움직이면 보임" 버그 수정) ──
        #   amcl_pose 토픽은 update_min_d/a(0.25m/0.2rad) 이상 움직여야만 재발행되고 QoS 도
        #   volatile 라, 백엔드가 늦게 구독하면 그 로봇이 실제로 움직이기 전까진 아무 메시지도
        #   못 받는다. TF(map→base_link) 는 이 임계값과 무관하게 계속 흐르므로, 로봇별 전용
        #   버퍼에 /carterN/tf(+tf_static) 를 직접 먹여 주기적으로 조회 → 정지 상태에서도 즉시
        #   표시. 프레임 id 가 로봇마다 동일(map/odom/base_link, 접두 없음)해서 버퍼를 공유하면
        #   충돌하므로 로봇별로 분리한다.
        for carter_id in CARTER_IDS:
            buf = tf2_ros.Buffer()
            self._tf_buffers[carter_id] = buf
            self.node.create_subscription(
                TFMessage, f"/{carter_id}/tf",
                lambda msg, b=buf: self._tf_callback(msg, b, False), 100,
            )
            self.node.create_subscription(
                TFMessage, f"/{carter_id}/tf_static",
                lambda msg, b=buf: self._tf_callback(msg, b, True), 100,
            )
        self._pose_timer = self.node.create_timer(0.2, self._poll_tf_poses)

        # ── YOLO 비전 스트림 (웹 VisionFeedPanel) ──
        #   구독: /carterN/vision/annotated/compressed (multi_robot_yolo_viewer.py 가 발행하는
        #   bbox 그려진 JPEG). Isaac 카메라/뷰어는 sensor-data QoS(BEST_EFFORT) — 안 맞추면 안 들어옴.
        #   19_ 은 carter1+carter2 둘 다 YOLO 카메라가 있으므로(18_ 은 carter1 전용이었음) 두
        #   로봇 다 실제로 프레임이 들어온다.
        _vision_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                  history=HistoryPolicy.KEEP_LAST, depth=1)
        for carter_id in CARTER_IDS:
            self.node.create_subscription(
                CompressedImage,
                f"/{carter_id}/vision/annotated/compressed",
                lambda msg, cid=carter_id: self._vision_frame_callback(msg, cid),
                _vision_qos,
            )

        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()
        logger.info("RobotBridge(v2) 시작: carters=%s", CARTER_IDS)

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
    def _process_state_callback(self, msg, carter_id: str) -> None:
        state, message = _split_colon(msg.data)
        payload = message or STATE_KO_MAP.get(state, state)
        self._track_task_history(carter_id, payload)
        self._dispatch(
            {
                "type": "PROCESS_STATE",
                "robotId": carter_id,
                "payload": payload,
                # [HMI v2 신규] 프론트가 STEPS_BY_TASK 조회에 쓸 수 있도록 현재 배정된 task 를
                # 상태 브로드캐스트에 같이 실어보낸다(task_select 발행 시 캐시해둔 값).
                "task": self._current_task.get(carter_id),
                "timestamp": time.time(),
            }
        )

    def _track_task_history(self, robot_id: str, payload: str) -> None:
        """작업 큐(QueuePanel) 백엔드 소스 — tb_task_history 는 스키마+조회(/api/history)만 있고
        아무도 안 채우던 죽은 테이블이었다. process_state 가 "대기"↔"RUNNING:..." 를 오갈 때마다
        여기서 채운다 : 대기 진입 = 열린 작업 종료(DONE), 그 외 라벨 진입인데 열린 작업이 없으면
        새 작업 시작(RUNNING). 같은 작업의 하위 단계 라벨 전환(예: 노즐접촉→노즐장착)은 새 행을
        만들지 않고 하나로 묶는다(스키마에 라벨 컬럼이 없어 단계별 세분화는 안 함 — 필요하면 확장)."""
        now = datetime.now(timezone.utc).isoformat()
        conn = get_connection()
        try:
            if payload == "대기":
                conn.execute(
                    "UPDATE tb_task_history SET end_time=?, status='DONE' "
                    "WHERE robot_id=? AND end_time IS NULL",
                    (now, robot_id),
                )
            else:
                cur = conn.execute(
                    "SELECT task_id FROM tb_task_history WHERE robot_id=? AND end_time IS NULL",
                    (robot_id,),
                )
                if cur.fetchone() is None:
                    conn.execute(
                        "INSERT INTO tb_task_history (task_id, robot_id, start_time, end_time, status) "
                        "VALUES (?, ?, ?, NULL, 'RUNNING')",
                        (str(uuid.uuid4()), robot_id, now),
                    )
            conn.commit()
        except Exception as exc:  # noqa: BLE001
            logger.warning("tb_task_history 갱신 실패: %s", exc)
        finally:
            conn.close()

    def _safety_event_callback(self, msg, carter_id: str) -> None:
        # 형식: "ERR_CODE:사람이 읽는 메시지"
        code, message = _split_colon(msg.data)
        self._dispatch(
            {
                "type": "SAFETY_EVENT",
                "robotId": carter_id,
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

    def _tf_callback(self, msg, buf, is_static: bool) -> None:
        for tr in msg.transforms:
            try:
                if is_static:
                    buf.set_transform_static(tr, "hmi_bridge")
                else:
                    buf.set_transform(tr, "hmi_bridge")
            except Exception as exc:  # noqa: BLE001
                logger.debug("tf set_transform 실패: %s", exc)

    def _poll_tf_poses(self) -> None:
        """타이머(5Hz, ROS 스레드) — 로봇별 버퍼에서 map→base_link 를 조회해 ROBOT_POSE 로 발행.
        amcl_pose 콜백과 별개 경로라 두 소스가 같이 들어와도 무해(최신값이 이긴다)."""
        import rclpy

        for carter_id, buf in self._tf_buffers.items():
            try:
                tr = buf.lookup_transform("map", "base_link", rclpy.time.Time())
            except Exception:  # noqa: BLE001  (아직 tf 안 쌓였으면 정상적으로 실패)
                continue
            q = tr.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self._dispatch(
                {
                    "type": "ROBOT_POSE",
                    "robotId": carter_id,
                    "x": float(tr.transform.translation.x),
                    "y": float(tr.transform.translation.y),
                    "yaw": float(yaw),
                    "timestamp": time.time(),
                }
            )

    def _vision_frame_callback(self, msg, carter_id: str) -> None:
        """스레드 컨텍스트. WS 큐를 안 거치고 락으로 보호된 '최신 프레임 1장'만 갱신한다
        (MJPEG 스트리밍은 최신값만 필요 — 큐에 쌓아 지연시킬 이유가 없다)."""
        with self._frame_lock:
            self._latest_frames[carter_id] = bytes(msg.data)

    def get_latest_frame(self, carter_id: str) -> Optional[bytes]:
        """FastAPI(asyncio) 컨텍스트에서 호출. 아직 프레임이 없으면 None."""
        with self._frame_lock:
            return self._latest_frames.get(carter_id)

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

    def publish_task_select(self, carter_id: str, task: str) -> bool:
        """[HMI v2 신규] 로봇에 작업(trash|spray)을 배정 — /{carter_id}/task_select 로 발행.
        19_ 의 g_task_select_mission 이 이 메시지 수신 자체를 시작 트리거로 쓴다(별도 게이트 없음).
        발행 성공 True, 미등록 carter_id 면 False."""
        from std_msgs.msg import String

        pub = self.task_pub.get(carter_id)
        if pub is None:
            logger.warning("task_pub 미초기화/미지원 carter_id — 무시: %s", carter_id)
            return False
        pub.publish(String(data=task))
        self._current_task[carter_id] = task
        logger.info("task_select → /%s/task_select = %s", carter_id, task)
        return True

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
