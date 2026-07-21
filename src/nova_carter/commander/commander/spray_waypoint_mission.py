#!/usr/bin/env python3
"""
spray_waypoint_mission.py — waypoint 도착 시 분사 트리거 (작업 3 골격)
============================================================================
목적 : (임시) waypoint 리스트를 순회하며 Nav2(NavigateToPose)로 이동하고, 지정
waypoint 에 "도착"하면 분사를 시작/종료하는 트리거 로직의 뼈대(skeleton).

지금 범위
  - 실제 맵/waypoint 좌표 확정 전이므로 좌표는 하드코딩(파라미터로 교체 가능).
  - 분사 시작/종료는 stub. 단, 그냥 print 가 아니라 std_msgs/Bool 을 `/spray_active`
    로 발행하는 "실제 인터페이스"로 만들어, 추후 Isaac 팔 분사 노드(예: 9_carter_
    wall_spray 를 개조한 노드)가 이 토픽을 구독해 위아래+방사 모션을 켜/끄면 된다.

구조
  for wp in waypoints:
      navigate_to(wp)                 # Nav2 NavigateToPose
      if 도착 성공 and wp.spray:
          spray_start()               # /spray_active = True  (stub)
          _sleep(spray_duration)
          spray_stop()                # /spray_active = False (stub)

파라미터 (병렬 배열; 길이 동일해야 함)
  waypoints_x, waypoints_y, waypoints_yaw : float[]  (map 프레임)
  waypoints_spray : bool[]  (해당 지점에서 분사할지)
  spray_duration  : float   분사 유지 시간 [s]
  frame_id, action_name, server_timeout, loop

사용
  # Nav2 가 떠 있는 상태에서
  ros2 run commander spray_waypoint_mission
  # 좌표 교체 예:
  ros2 run commander spray_waypoint_mission --ros-args \
    -p waypoints_x:="[1.0, 3.0, 5.0]" -p waypoints_y:="[0.0, 0.0, 0.0]" \
    -p waypoints_yaw:="[0.0, 0.0, 1.57]" -p waypoints_spray:="[true, false, true]" \
    -p spray_duration:=4.0
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

        # ── 임시 하드코딩 waypoint (추후 실제 좌표로 교체) ──
        self.declare_parameter("waypoints_x", [1.0, 3.0, 5.0])
        self.declare_parameter("waypoints_y", [0.0, 0.0, 0.0])
        self.declare_parameter("waypoints_yaw", [0.0, 0.0, 0.0])
        self.declare_parameter("waypoints_spray", [True, False, True])
        self.declare_parameter("spray_duration", 4.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("server_timeout", 10.0)
        self.declare_parameter("loop", False)

        self._client = ActionClient(
            self, NavigateToPose, self.get_parameter("action_name").value)
        # 분사 트리거 인터페이스 (stub → 나중에 Isaac 분사 노드가 구독)
        self._spray_pub = self.create_publisher(Bool, "/spray_active", 10)

    # ── waypoint 리스트 구성 ─────────────────────────────────────
    def waypoints(self):
        xs = list(self.get_parameter("waypoints_x").value)
        ys = list(self.get_parameter("waypoints_y").value)
        yaws = list(self.get_parameter("waypoints_yaw").value)
        sprays = list(self.get_parameter("waypoints_spray").value)
        n = min(len(xs), len(ys), len(yaws), len(sprays))
        if not (len(xs) == len(ys) == len(yaws) == len(sprays)):
            self.get_logger().warn(
                f"waypoint 배열 길이 불일치 → 앞 {n}개만 사용")
        return [(xs[i], ys[i], yaws[i], bool(sprays[i])) for i in range(n)]

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

    # ── 분사 트리거 (stub, /spray_active 발행) ───────────────────
    def spray_start(self):
        # TODO: 실제 분사 연결 지점. 지금은 /spray_active=True 발행.
        #       Isaac 팔 분사 노드가 이 토픽 구독해 위아래+방사 모션 ON.
        self._spray_pub.publish(Bool(data=True))
        self.get_logger().info("  >>> [SPRAY START] /spray_active=True (stub)")

    def spray_stop(self):
        # TODO: 실제 분사 종료 연결 지점.
        self._spray_pub.publish(Bool(data=False))
        self.get_logger().info("  <<< [SPRAY STOP ] /spray_active=False (stub)")

    def _sleep(self, seconds):
        """분사 유지 (스핀하며 대기). 스텁이라 wall-clock 기준."""
        end = time.monotonic() + float(seconds)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    # ── 미션 실행 ────────────────────────────────────────────────
    def run(self):
        timeout = float(self.get_parameter("server_timeout").value)
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("[FAIL] navigate_to_pose 서버 없음. Nav2 실행 확인.")
            return False

        dur = float(self.get_parameter("spray_duration").value)
        loop = bool(self.get_parameter("loop").value)
        wps = self.waypoints()
        self.get_logger().info(f"미션 시작: waypoint {len(wps)}개, spray_duration={dur}s, loop={loop}")

        while rclpy.ok():
            for i, (x, y, yaw, spray) in enumerate(wps):
                self.get_logger().info(
                    f"[WP {i+1}/{len(wps)}] → ({x:.2f},{y:.2f},{yaw:.2f}) spray={spray}")
                arrived = self.navigate_to(x, y, yaw)
                if not arrived:
                    self.get_logger().warn(f"  도착 실패 → 분사 건너뜀")
                    continue
                self.get_logger().info("  도착.")
                if spray:
                    self.spray_start()          # 도착 콜백 → 분사 시작
                    self._sleep(dur)
                    self.spray_stop()
            if not loop:
                break
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
        # 안전 : 종료 시 분사 OFF
        try:
            node.spray_stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
