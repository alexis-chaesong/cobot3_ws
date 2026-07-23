#!/usr/bin/env python3
"""
spray_waypoint_mission.py — Nav2 이동 ↔ 스윕 자동 핸드오프 오케스트레이터
============================================================================
역할 : 스윕 시작점(복도 입구 등)들을 순회하며,
  1) Nav2(NavigateToPose)로 시작점까지 이동  (거친 자율주행)
  2) 도착하면 /start_sweep(Bool=True) 발행    → Isaac 스크립트가 stop-and-go 소독 스윕 수행
  3) /sweep_done(Bool=True) 수신 대기          → 스윕 끝날 때까지 블록
  4) 다음 시작점으로

핸드오프 계약 (Isaac 10_carter_hospital_spray_nav.py DRIVE_MODE="handoff" 와 짝)
  mission → /start_sweep (Bool)  : 스윕 시작 트리거 (도착 후 1회)
  mission ← /sweep_done  (Bool)  : 스윕 완료 통지 (스크립트가 발행)
  → 둘이 /cmd_vel 을 "번갈아" 쓰므로(이동=Nav2, 스윕=스크립트) 충돌 없음.

파라미터 (병렬 배열; 길이 동일)
  sweep_x, sweep_y, sweep_yaw : float[]   스윕 시작 pose (map 프레임)
  sweep_enable                : bool[]    해당 지점에서 스윕할지 (False 면 통과만)
  sweep_timeout               : float     /sweep_done 최대 대기 [s]
  settle_time                 : float     도착 후 트리거까지 안정화 대기 [s]
  frame_id, action_name, server_timeout, loop
  return_home                 : bool      모든 소독 완료 후 초기 도킹 복귀 (기본 True)
  home_x, home_y, home_yaw    : float     복귀 지점 (기본 18.5,0,0 = 스폰/AMCL initial_pose)

사용
  # Isaac(10, handoff) Play + Nav2 bringup + 2D Pose Estimate 완료 후
  ros2 run commander spray_waypoint_mission --ros-args \
    -p sweep_x:="[3.0, 8.0]" -p sweep_y:="[0.0, 0.0]" -p sweep_yaw:="[0.0, 0.0]" \
    -p sweep_enable:="[true, true]" -p sweep_timeout:=120.0
============================================================================
"""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool


def yaw_to_quat(yaw):
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class SprayWaypointMission(Node):
    def __init__(self):
        super().__init__("spray_waypoint_mission")

        # ── 스윕 시작점 (map 프레임) : modified_hospital 우측 세로 복도 "양쪽 벽" 왕복 ──
        # 복도 free 범위 : x∈[17.3, 20.7](중심≈19), y 는 ~11 부터 좁아져 벽 y=20.33 까지.
        # WP1 = 시작(18.5, 8.0), yaw 90°(+Y) → +Y 로 10.5m 스윕(끝 y≈18.5, 벽까지 1.8m 여유).
        # WP2 = 끝(18.5, 18.5), yaw 270°(-Y) → Nav2 가 180° 회전 → -Y 로 스윕(반대편 벽).
        # 팔 겨냥이 로봇 몸체 기준(오른쪽)이라 180° 돌면 자동으로 반대편 벽을 향한다.
        # ★ 구 맵(긴 복도, y=28.9)용 좌표는 새 맵(y≤20.3)을 벗어나므로 위 값으로 교체함.
        # (RViz Publish Point → ros2 topic echo /clicked_point 으로 좌표 확인. yaw=복도 진행방향)
        # ※ FORWARD_DISTANCE(스윕 길이)는 Isaac 10_1 스크립트에서 10.5m 로 맞춤.
        self.declare_parameter("sweep_x", [18.5, 18.5])
        self.declare_parameter("sweep_y", [8.0, 18.5])
        self.declare_parameter("sweep_yaw", [1.5708, -1.5708])   # 90° → -90°(=270°)
        self.declare_parameter("sweep_enable", [True, True])
        self.declare_parameter("sweep_timeout", 120.0)   # /sweep_done 대기 상한 [s]
        self.declare_parameter("settle_time", 1.0)        # 도착 후 안정화 [s]
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("server_timeout", 10.0)
        self.declare_parameter("loop", False)

        # ── 멀티로봇 네임스페이스 ─────────────────────────────────────
        # 소독 로봇을 carter1/carter2 로 분리 운용할 때 접두. 빈 문자열이면 단일로봇
        # (기존 동작 그대로 : navigate_to_pose·/start_sweep·/sweep_done).
        # namespace="carter1" 이면 carter1/navigate_to_pose·/carter1/start_sweep·
        # /carter1/sweep_done 로 붙어, Isaac 통합 스크립트의 carter1 핸드오프 토픽과 짝이 된다.
        # (Isaac OmniGraph node_namespace=carter1 → /carter1/cmd_vel 을 Nav2 가 발행하는 것과 정합)
        self.declare_parameter("namespace", "")

        # 모든 소독 완료 후 "초기 도킹 스테이션(최초 스폰)"으로 복귀
        # 기본값 = Isaac CARTER_START_POSE / Nav2 amcl initial_pose 와 동일 (18.5, 0, yaw 0).
        # 스폰 위치를 바꾸면 home_* 도 같이 맞출 것. return_home:=false 면 복귀 생략.
        self.declare_parameter("return_home", True)
        self.declare_parameter("home_x", 18.5)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_yaw", 0.0)

        # namespace 접두 헬퍼 : ns 있으면 "/carter1/xxx", 없으면 "xxx"/"/xxx"(단일로봇)
        ns = str(self.get_parameter("namespace").value).strip("/")
        action_name = self.get_parameter("action_name").value
        if ns:
            action_name = f"{ns}/{action_name}"          # /carter1/navigate_to_pose
            start_topic = f"/{ns}/start_sweep"
            done_topic = f"/{ns}/sweep_done"
        else:
            start_topic = "/start_sweep"
            done_topic = "/sweep_done"
        self.get_logger().info(
            f"namespace='{ns or '(none)'}' → action='{action_name}', "
            f"pub='{start_topic}', sub='{done_topic}'")

        self._client = ActionClient(self, NavigateToPose, action_name)

        # 핸드오프 인터페이스
        self._start_pub = self.create_publisher(Bool, start_topic, 10)
        self._done = False
        self.create_subscription(Bool, done_topic, self._on_sweep_done, 10)

    def _on_sweep_done(self, msg):
        if bool(msg.data):
            self._done = True

    # ── 스윕 시작점 리스트 ────────────────────────────────────────
    def sweeps(self):
        xs = list(self.get_parameter("sweep_x").value)
        ys = list(self.get_parameter("sweep_y").value)
        yaws = list(self.get_parameter("sweep_yaw").value)
        ens = list(self.get_parameter("sweep_enable").value)
        n = min(len(xs), len(ys), len(yaws))
        if not (len(xs) == len(ys) == len(yaws)):
            self.get_logger().warn(f"sweep_x/y/yaw 길이 불일치 → 앞 {n}개만 사용")
        # sweep_enable 이 짧으면(또는 미지정) 나머지 지점은 True 로 간주 → 길이 안 맞춰도 됨
        return [(xs[i], ys[i], yaws[i], bool(ens[i]) if i < len(ens) else True)
                for i in range(n)]

    # ── Nav2 이동 ────────────────────────────────────────────────
    def navigate_to(self, x, y, yaw):
        """(x,y,yaw) 로 이동. 도착 성공(SUCCEEDED)이면 True."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.get_parameter("frame_id").value
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qz, qw = yaw_to_quat(float(yaw))
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("  goal 거부됨")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result()
        return res is not None and res.status == GoalStatus.STATUS_SUCCEEDED

    # ── 스윕 트리거 + 완료 대기 (핸드오프) ────────────────────────
    def run_sweep(self):
        """/start_sweep 발행 후 /sweep_done 올 때까지 블록. 완료 True, 타임아웃 False."""
        timeout = float(self.get_parameter("sweep_timeout").value)
        self._done = False
        self._start_pub.publish(Bool(data=True))     # 도착 후 1회만 트리거
        self.get_logger().info("  >>> [SWEEP] /start_sweep=True → 스크립트 스윕 대기")
        end = time.monotonic() + timeout
        while rclpy.ok() and not self._done and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._done:
            self.get_logger().info("  <<< [SWEEP] /sweep_done 수신 → 완료")
            return True
        self.get_logger().warn(f"  [SWEEP] {timeout:.0f}s 내 /sweep_done 없음 → 타임아웃")
        return False

    def _reset_sweep(self):
        """/start_sweep=False 를 여러 번 발행 → 스크립트가 진행 중 스윕을 취소하고 STANDBY 로."""
        for _ in range(5):
            self._start_pub.publish(Bool(data=False))
            rclpy.spin_once(self, timeout_sec=0.05)

    def _settle(self, seconds):
        end = time.monotonic() + float(seconds)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ── 미션 실행 ────────────────────────────────────────────────
    def run(self):
        timeout = float(self.get_parameter("server_timeout").value)
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("[FAIL] navigate_to_pose 서버 없음. Nav2 실행 확인.")
            return False

        settle = float(self.get_parameter("settle_time").value)
        loop = bool(self.get_parameter("loop").value)
        sweeps = self.sweeps()
        self.get_logger().info(f"미션 시작: 스윕 시작점 {len(sweeps)}개, loop={loop}")

        # 시작 시 이전 실행에서 갇힌 스윕이 있으면 취소(STANDBY 보장) → 이동 중 /cmd_vel 충돌 방지
        self._reset_sweep()

        while rclpy.ok():
            for i, (x, y, yaw, enable) in enumerate(sweeps):
                self.get_logger().info(
                    f"[{i+1}/{len(sweeps)}] → 이동 ({x:.2f},{y:.2f},{yaw:.2f}) sweep={enable}")
                if not self.navigate_to(x, y, yaw):
                    self.get_logger().warn("  도착 실패 → 이 지점 건너뜀")
                    continue
                self.get_logger().info("  도착.")
                if enable:
                    self._settle(settle)             # 정지 안정화 후 트리거
                    self.run_sweep()                 # /start_sweep → /sweep_done 대기
                    self._reset_sweep()              # 확실히 STANDBY 로 되돌림(잔류 트리거 제거)
            if not loop:
                break

        # ── 모든 소독 완료 → 초기 도킹 스테이션(홈)으로 복귀 ──
        if bool(self.get_parameter("return_home").value):
            hx = float(self.get_parameter("home_x").value)
            hy = float(self.get_parameter("home_y").value)
            hyaw = float(self.get_parameter("home_yaw").value)
            self._reset_sweep()      # 스윕 확실히 STANDBY → 복귀 주행은 Nav2 가(/cmd_vel 충돌 방지, 팔 stow)
            self.get_logger().info(f"[HOME] 소독 완료 → 초기 도킹 복귀 ({hx:.2f},{hy:.2f},{hyaw:.2f})")
            if self.navigate_to(hx, hy, hyaw):
                self.get_logger().info("[HOME] 초기 도킹 도착.")
            else:
                self.get_logger().warn("[HOME] 초기 도킹 복귀 실패(도달 불가/거부).")

        self.get_logger().info("[DONE] 미션 종료")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SprayWaypointMission()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 진행 중 스윕 취소(스크립트를 STANDBY 로) → 다음 실행 때 갇힌 스윕 방지
        try:
            if rclpy.ok():
                node._reset_sweep()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():                    # 이중 shutdown(RCLError) 가드
            rclpy.shutdown()


if __name__ == "__main__":
    main()
