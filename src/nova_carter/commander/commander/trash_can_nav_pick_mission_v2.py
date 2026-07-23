"""
trash_can_nav_pick_mission_v2.py
--------------------------------------------------
4_v2_mobile_manipulator_trash_can_nav_pick_test.py 전용 커맨더.

원본 trash_can_nav_pick_mission.py 는 수정하지 않고, Nav2 가 목표 근처에서
TaskResult.FAILED 를 내는 경우(남은 거리 0.00 m 등)에도 /start_pick 을 내도록
보완한 v2.

실행 : ros2 run commander trash_can_nav_pick_mission_v2
--------------------------------------------------
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

CARTER_START_POSE = (16.66290495232035, -0.0029517927591273807, 0.0)  # x, y, yaw_deg

NAV_GOAL_TOPIC = "/trash_can_nav_goal"
START_PICK_TOPIC = "/start_pick"

# Nav2 xy_goal_tolerance / standoff 여유보다 약간 크게 — 0.00m FAILED 루프 방지
NEAR_GOAL_M = 0.75


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return [qx, qy, qz, qw]


def create_pose(navigator, x, y, yaw_deg):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    q = get_quaternion_from_euler(0, 0, math.radians(yaw_deg))
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


def publish_start_pick(pick_pub, nav, leg: int, reason: str):
    print(f"[NAV] ({leg}구간) {reason} → '{START_PICK_TOPIC}'=True 발행")
    for _ in range(15):
        pick_pub.publish(Bool(data=True))
        rclpy.spin_once(nav, timeout_sec=0.05)


def main():
    rclpy.init()
    nav = BasicNavigator()

    init_pose = create_pose(nav, *CARTER_START_POSE)
    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active()

    goal_holder = {"pose": None}

    def on_goal(msg):
        goal_holder["pose"] = msg

    nav.create_subscription(PoseStamped, NAV_GOAL_TOPIC, on_goal, 10)
    pick_pub = nav.create_publisher(Bool, START_PICK_TOPIC, 10)

    last_served_xy = None
    leg = 0
    while True:
        leg += 1
        goal_holder["pose"] = None
        print(f"[WAIT] ({leg}구간) '{NAV_GOAL_TOPIC}' 수신 대기 중...")
        while goal_holder["pose"] is None:
            rclpy.spin_once(nav, timeout_sec=0.5)

        goal_pose = goal_holder["pose"]
        gx = float(goal_pose.pose.position.x)
        gy = float(goal_pose.pose.position.y)

        if last_served_xy is not None:
            if math.hypot(gx - last_served_xy[0], gy - last_served_xy[1]) < 0.05:
                print(f"[NAV] ({leg}구간) 동일 목표 재수신(이미 처리함) → goToPose 생략, "
                      f"'{START_PICK_TOPIC}' 재발행")
                publish_start_pick(pick_pub, nav, leg, "동일 목표(이미 도착 처리)")
                continue

        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        print(f"[NAV] ({leg}구간) 목표 수신: x={gx:.3f}, y={gy:.3f} → 주행 시작")
        nav.goToPose(goal_pose)

        last_dist = None
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                last_dist = float(feedback.distance_remaining)
                print(f"남은 거리: {last_dist:.2f} m")

        result = nav.getResult()
        near = last_dist is not None and last_dist <= NEAR_GOAL_M

        if result == TaskResult.SUCCEEDED:
            publish_start_pick(pick_pub, nav, leg, "목적지 도착 완료(SUCCEEDED)")
            last_served_xy = (gx, gy)
        elif result == TaskResult.FAILED and near:
            publish_start_pick(
                pick_pub, nav, leg,
                f"FAILED 이지만 목표 근접(dist={last_dist:.2f}m≤{NEAR_GOAL_M}m) → 도착 간주",
            )
            last_served_xy = (gx, gy)
        elif result == TaskResult.CANCELED:
            print(f"[NAV] ({leg}구간) 주행 취소됨 → '{START_PICK_TOPIC}' 발행 안 함")
        else:
            dist_txt = f"{last_dist:.2f}m" if last_dist is not None else "n/a"
            print(f"[NAV] ({leg}구간) 주행 실패(result={result}, dist={dist_txt}) "
                  f"→ '{START_PICK_TOPIC}' 발행 안 함 (재시도 대기)")


if __name__ == "__main__":
    main()
