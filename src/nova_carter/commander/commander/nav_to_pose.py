#!/usr/bin/env python3
"""
nav_to_pose.py — Nav2 연결 확인용 NavigateToPose 액션 클라이언트
============================================================================
목적(작업 1) : 실제 맵/waypoint 확정 전에, Nova Carter 가 Nav2 의
`navigate_to_pose` 액션 서버와 정상 통신되는지 확인한다.
  - 액션 서버(bt_navigator) 연결 여부
  - dummy goal 수락(accept/reject) 여부
  - 피드백(남은 거리) 및 최종 결과(SUCCEEDED/ABORTED 등) 수신

이 스크립트는 "연결/응답 확인"이 목적이므로, goal 도달 성공까지는 요구하지
않는다(맵·위치추정이 없으면 ABORTED 나도 "서버는 살아있음"으로 판단 가능).

사용
  # Nav2 (carter_navigation.launch.py 등) 가 떠 있는 상태에서
  ros2 run commander nav_to_pose                 # 기본 dummy goal (1.0, 0.0, 0.0)
  ros2 run commander nav_to_pose --ros-args -p x:=2.0 -p y:=1.0 -p yaw:=1.57
  ros2 run commander nav_to_pose --ros-args -p frame_id:=map -p server_timeout:=5.0

주의 : goal 은 map 프레임 기준. use_sim_time 은 Isaac /clock 에 맞춰 true 권장.
============================================================================
"""
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw):
    """yaw(rad) → geometry_msgs 쿼터니언 (z, w)."""
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class NavToPoseChecker(Node):
    def __init__(self):
        super().__init__("nav_to_pose_checker")
        # 파라미터 (dummy goal / 서버 대기시간)
        self.declare_parameter("x", 1.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("server_timeout", 10.0)

        self._client = ActionClient(
            self, NavigateToPose, self.get_parameter("action_name").value
        )
        self._result_status = None

    # ── 1) 서버 연결 확인 ─────────────────────────────────────────
    def wait_for_server(self):
        timeout = float(self.get_parameter("server_timeout").value)
        name = self.get_parameter("action_name").value
        self.get_logger().info(f"[1/4] '{name}' 액션 서버 대기 (최대 {timeout:.0f}s)...")
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(
                f"[FAIL] 액션 서버 '{name}' 없음. Nav2(bt_navigator)가 떠있는지, "
                f"네임스페이스/토픽이 맞는지 확인하세요.")
            return False
        self.get_logger().info("[OK] 액션 서버 연결됨.")
        return True

    # ── 2) goal 구성 + 전송 ──────────────────────────────────────
    def build_goal(self):
        x = float(self.get_parameter("x").value)
        y = float(self.get_parameter("y").value)
        yaw = float(self.get_parameter("yaw").value)
        frame = self.get_parameter("frame_id").value

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        self.get_logger().info(
            f"[2/4] dummy goal 전송: frame={frame} x={x:.2f} y={y:.2f} yaw={yaw:.2f}")
        return goal

    def send_goal(self):
        goal = self.build_goal()
        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("[FAIL] goal 이 거부됨(rejected).")
            return False
        self.get_logger().info("[3/4] goal 수락됨(accepted). 결과 대기...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self._result_status = result.status if result is not None else None
        return True

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        try:
            dist = fb.distance_remaining
            self.get_logger().info(f"    피드백: 남은거리 {dist:.2f} m")
        except AttributeError:
            pass

    # ── 3) 결과 판정 ─────────────────────────────────────────────
    def report(self):
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        s = status_map.get(self._result_status, str(self._result_status))
        self.get_logger().info(f"[4/4] 최종 상태: {s}")
        if self._result_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[RESULT] Nav2 연결 + 주행 성공 ✅")
        else:
            self.get_logger().warn(
                "[RESULT] 서버 통신은 OK. 도달 실패(맵/위치추정/좌표 확인 필요) — "
                "'연결 확인' 목적은 통과.")


def main(args=None):
    rclpy.init(args=args)
    node = NavToPoseChecker()
    exit_code = 0
    try:
        if not node.wait_for_server():
            exit_code = 1
        elif not node.send_goal():
            exit_code = 2
        else:
            node.report()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
