"""
trash_can_nav_pick_mission.py
--------------------------------------------------
4_mobile_manipulator_trash_can_nav_pick_test.py 의 짝 노드.

Isaac Sim 쪽 스크립트는 /clock 발행과 씬 관리만 하고, Nav2 주행(BasicNavigator의
blocking 호출)은 데드락을 피하기 위해 이 노드가 별도 프로세스로 담당한다
(Isaac Sim 스크립트 안에서 이 blocking 호출을 하면, 그동안 /clock 발행이 멈춰서
use_sim_time Nav2 노드들이 그 /clock 을 기다리며 같이 멈춰버린다).

흐름 : AMCL 초기위치 설정(CARTER_START_POSE, move_tash_can.usd 스폰 pose와 반드시
       일치해야 함) → Nav2 active 대기 → Isaac Sim 스크립트가 퍼블리시하는
       /trash_can_nav_goal 수신 → goToPose → 완료 시 /start_pick=True 발행.

실행 : ros2 run commander trash_can_nav_pick_mission
       (Nav2 bringup 이 먼저 떠 있어야 하고, 4_..._nav_pick_test.py 도 함께 실행 중이어야 함)
--------------------------------------------------
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# move_tash_can.usd 의 실제 스폰 pose(chassis_link) — modified_hospital.usd 의
# docking_station_02 마커(x=6.9807, y=0) 위치로 맞춤. 4_..._nav_pick_test.py 의
# CARTER_START_POSE 와 반드시 일치시킬 것. 어긋나면 AMCL 초기 추정 위치가 실제
# 위치와 달라져 이후 모든 위치추정/주행이 틀어진다.
#
# (참고: map 프레임이 stage 프레임과 x 부호가 반대일 거라 추정하고 한 번 반전 +
# yaw 180 보정을 시도했었으나, RViz 2D Pose Estimate 로 Isaac Sim 상 실제 위치에 맞게
# 직접 찍었을 때 주행이 정상 동작하는 것으로 확인되어 — 즉 map 프레임 = stage 프레임,
# 반전 불필요 — 원래대로 되돌림.)
CARTER_START_POSE = (16.66290495232035, -0.0029517927591273807, 0.0)  # x, y, yaw_deg

NAV_GOAL_TOPIC = "/trash_can_nav_goal"
START_PICK_TOPIC = "/start_pick"


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

    # Isaac Sim 스크립트가 여러 구간(소형 쓰레기통 파지 -> big_trash 덤프 -> 이후 추가될
    # 원위치 복귀/도킹 복귀)마다 같은 토픽에 새 목표를 퍼블리시하므로, 한 번 처리하고
    # 끝내지 않고 계속 반복해서 받는다. Isaac Sim 쪽은 /start_pick 받을 때까지 같은
    # 목표를 계속 재발행하므로, 주행 실패/취소돼도 다음 반복에서 같은 목표를 다시 받아
    # 자동으로 재시도된다.
    leg = 0
    while True:
        leg += 1
        goal_holder["pose"] = None
        print(f"[WAIT] ({leg}구간) '{NAV_GOAL_TOPIC}' 수신 대기 중...")
        while goal_holder["pose"] is None:
            rclpy.spin_once(nav, timeout_sec=0.5)

        goal_pose = goal_holder["pose"]
        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        print(f"[NAV] ({leg}구간) 목표 수신: x={goal_pose.pose.position.x:.3f}, "
              f"y={goal_pose.pose.position.y:.3f} → 주행 시작")
        nav.goToPose(goal_pose)

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                print(f"남은 거리: {feedback.distance_remaining:.2f} m")

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"[NAV] ({leg}구간) 목적지 도착 완료 → '{START_PICK_TOPIC}' 발행")
            for _ in range(10):
                pick_pub.publish(Bool(data=True))
                rclpy.spin_once(nav, timeout_sec=0.1)
        elif result == TaskResult.CANCELED:
            print(f"[NAV] ({leg}구간) 주행 취소됨 → '{START_PICK_TOPIC}' 발행 안 함")
        else:
            print(f"[NAV] ({leg}구간) 주행 실패 → '{START_PICK_TOPIC}' 발행 안 함")


if __name__ == "__main__":
    main()
