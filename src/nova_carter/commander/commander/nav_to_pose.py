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
import math
import time
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

# ==========================================================
# 🧮 [연산 함수 1] 오일러 각도(Rad) -> 쿼터니언 변환 함수
# ==========================================================
def get_quaternion_from_euler(roll, pitch, yaw):
    """math 모듈만 사용하여 오일러 각도를 쿼터니언으로 변환"""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

# ==========================================================
# 🧮 [연산 함수 2] 쿼터니언 추출 -> 오일러 각도(Rad) 변환 함수
# ==========================================================
def get_euler_from_quaternion(x, y, z, w):
    """수신된 쿼터니언 값을 추출하여 오일러 각도(라디안)로 연산"""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

# ==========================================================
# 🖨️ [출력 함수] 최종 도착 위치 및 방향 출력기
# ==========================================================
def print_final_pose(pose_msg):
    """PoseStamped 메시지를 받아 위치와 오일러 각도를 예쁘게 출력"""
    if not pose_msg:
        return

    pos = pose_msg.pose.position
    ori = pose_msg.pose.orientation
    
    # 쿼터니언 -> 오일러 각도(라디안) 추출
    roll_rad, pitch_rad, yaw_rad = get_euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
    
    # 라디안 -> 디그리(도) 변환
    roll_deg = math.degrees(roll_rad)
    pitch_deg = math.degrees(pitch_rad)
    yaw_deg = math.degrees(yaw_rad)
    
    # 결과 출력
    print("-" * 50)
    print(f"📍 최종 위치: X = {pos.x:.3f} m, Y = {pos.y:.3f} m")
    print(f"🧭 최종 방향 (Radian): Roll = {roll_rad:.3f}, Pitch = {pitch_rad:.3f}, Yaw = {yaw_rad:.3f}")
    print(f"🧭 최종 방향 (Degree): {yaw_deg:.1f}°")
    print("-" * 50)


# ==========================================================
# 🚀 메인 주행 로직
# ==========================================================
def create_pose(navigator, x, y, yaw_deg):
    """x, y, yaw(도 단위) → PoseStamped 생성"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)

    yaw_rad = math.radians(yaw_deg)
    q = get_quaternion_from_euler(0, 0, yaw_rad)
    
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()
    
    # 1. 출발점 설정
    init_pose = create_pose(nav, 0.1338968276977539, 11.693913459777832, 0.007)
    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active()
    
    # 2. 목표 지점 설정
    goal_pose = create_pose(nav, -7.159238815307617, 15.961247444152832, 180.0)
        
    # 3. Task 실행
    nav.goToPose(goal_pose)
    
    last_pose = None

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            last_pose = feedback.current_pose
            print(f"남은 거리: {feedback.distance_remaining:.2f} m")
            
        time.sleep(1.0)

    # 4. 결과 처리
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('\n🎉 목적지 도착 완료!')
        
        print_final_pose(last_pose)
            
    elif result == TaskResult.CANCELED:
        print('주행 취소됨')
    elif result == TaskResult.FAILED:
        print('주행 실패')

    rclpy.shutdown()

if __name__ == '__main__':
    main()
