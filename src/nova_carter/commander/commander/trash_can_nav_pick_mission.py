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
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from commander.hmi_link import HmiLink

# move_tash_can.usd 의 실제 스폰 pose(chassis_link) — modified_hospital.usd 의
# docking_station_02 마커(x=6.9807, y=0) 위치로 맞춤. 4_..._nav_pick_test.py 의
# CARTER_START_POSE 와 반드시 일치시킬 것. 어긋나면 AMCL 초기 추정 위치가 실제
# 위치와 달라져 이후 모든 위치추정/주행이 틀어진다.
#
# (참고: map 프레임이 stage 프레임과 x 부호가 반대일 거라 추정하고 한 번 반전 +
# yaw 180 보정을 시도했었으나, RViz 2D Pose Estimate 로 Isaac Sim 상 실제 위치에 맞게
# 직접 찍었을 때 주행이 정상 동작하는 것으로 확인되어 — 즉 map 프레임 = stage 프레임,
# 반전 불필요 — 원래대로 되돌림.)
#
# [17_dual_task_select_tool_changer_integrated.py 대응] 이 노드는 원래 carter2(폐기물)
# 전용으로만 쓰여서 시작pose가 carter2 스폰 하나로 하드코딩돼 있었다 — 이제 carter1도 같은
# 제너릭 포워더를 namespace:=carter1 로 띄워 쓰므로, namespace 별 시작pose를 따로 둔다.
# 미등록 namespace 는 기존 기본값(carter2 좌표)로 안전하게 폴백(하위호환, 13_/16_/기존 배선 무영향).
CARTER_START_POSES = {
    "": (16.66290495232035, -0.0029517927591273807, 0.0),        # 기존 기본값(변경 없음, 13_/14_/16_)
    "carter2": (16.66290495232035, -0.0029517927591273807, 0.0),  # 13_/14_/16_ 도 공용 — yaw 못 바꿈,
                                                                    # 17_ 은 -p start_yaw_deg:=90.0 오버라이드로 대응
    "carter1": (18.5, 0.2317, 90.0),   # namespace='carter1' 은 17_ 전용이라 직접 수정 안전. 17_ C1_START_POSE 와 일치
}

NAV_GOAL_TOPIC = "/trash_can_nav_goal"
START_PICK_TOPIC = "/start_pick"


def _read_namespace():
    """BasicNavigator 는 생성 시점에 namespace 가 필요하므로(액션 서버 이름 결정),
    본 네비게이터를 만들기 전에 임시 노드로 'namespace' 파라미터만 먼저 읽는다.
    빈 문자열이면 단일로봇(기존 동작). 'carter2' 면 폐기물 로봇 멀티로봇 운용."""
    tmp = Node("trash_can_nav_pick_param_reader")
    tmp.declare_parameter("namespace", "")
    ns = str(tmp.get_parameter("namespace").value).strip("/")
    tmp.destroy_node()
    return ns


def get_quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)  # [태성] 표준 ZYX 공식(끝항 sin)
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

    ns = _read_namespace()
    # namespace 있으면 Nav2 액션/토픽이 /carter2/ 로 접두되므로 BasicNavigator 도 같은
    # namespace 로 생성해야 goToPose 가 /carter2/navigate_to_pose 등과 통신한다.
    # 조율 토픽(goal/pick)도 /carter2/ 로 붙여 Isaac 통합 스크립트 carter2 쪽과 짝을 맞춘다.
    if ns:
        nav = BasicNavigator(namespace=ns)
        goal_topic = f"/{ns}/trash_can_nav_goal"
        pick_topic = f"/{ns}/start_pick"
    else:
        nav = BasicNavigator()
        goal_topic = NAV_GOAL_TOPIC
        pick_topic = START_PICK_TOPIC
    print(f"[NS] namespace='{ns or '(none)'}' → goal_sub='{goal_topic}', pick_pub='{pick_topic}'")

    start_pose = CARTER_START_POSES.get(ns, CARTER_START_POSES[""])
    # [2026-07-26 17_ 후진진입 도킹 대응] CARTER_START_POSES 는 13_/14_/16_ 도 같이 쓰는 공용
    # 테이블이라(그쪽 carter2 는 여전히 x,y,yaw=기존값 스폰) 테이블 값 자체를 못 바꾼다 — 대신
    # 선택적 파라미터로 x/y/yaw 를 오버라이드. 안 주면(기본 sentinel) 테이블 값 그대로.
    nav.declare_parameter("start_x", float("nan"))
    nav.declare_parameter("start_y", float("nan"))
    nav.declare_parameter("start_yaw_deg", float("nan"))
    x_override = float(nav.get_parameter("start_x").value)
    y_override = float(nav.get_parameter("start_y").value)
    yaw_override = float(nav.get_parameter("start_yaw_deg").value)
    start_pose = (
        start_pose[0] if math.isnan(x_override) else x_override,
        start_pose[1] if math.isnan(y_override) else y_override,
        start_pose[2] if math.isnan(yaw_override) else yaw_override,
    )
    print(f"[NS] namespace='{ns or '(none)'}' → AMCL 초기pose={start_pose}")
    init_pose = create_pose(nav, *start_pose)
    nav.setInitialPose(init_pose)

    # ★ waitUntilNav2Active() 는 carter2/amcl lifecycle('active')을 get_state 로 폴링하는데,
    #   그 서비스가 안 잡히면 '무한 hang'(Publishing Initial Pose 에서 멈춤) → goal 루프 도달 못 함.
    #   carter1(spray)처럼 navigate_to_pose 액션서버만 '타임아웃 대기'로 바꿔 hang 을 없앤다.
    #   서버가 없어도 치명적 처리 안 함 : Isaac 이 goal 을 계속 재발행하므로 이후 goToPose 에서
    #   자연히 재시도된다(Nav2 가 늦게 떠도 자가복구).
    WAIT_NAV2_TIMEOUT = 30.0
    if nav.nav_to_pose_client.wait_for_server(timeout_sec=WAIT_NAV2_TIMEOUT):
        print("[NS] navigate_to_pose 액션서버 확인 → 진행")
    else:
        print(f"[NS][WARN] {WAIT_NAV2_TIMEOUT:.0f}s 내 navigate_to_pose 서버 없음 "
              "→ 그래도 진행(goal 수신 시 재시도).")

    goal_holder = {"pose": None}

    def on_goal(msg):
        goal_holder["pose"] = msg

    nav.create_subscription(PoseStamped, goal_topic, on_goal, 10)
    pick_pub = nav.create_publisher(Bool, pick_topic, 10)

    # ── HMI(웹) 연동 ─────────────────────────────────────────────
    # hmi_enable=True 면 진행 단계를 /robot/waste/process_state 로 발행하고
    # /robot/command(START/EMERGENCY_STOP) 를 받는다.
    # wait_for_hmi_start=True 면 START 를 받을 때까지 "대기"(웹에서 시작). 긴급정지는
    # Nav2 goal 취소 + cmd_vel 0 발행 후 재시작(START)까지 대기.
    nav.declare_parameter("hmi_enable", True)
    nav.declare_parameter("hmi_robot_id", "waste")
    nav.declare_parameter("wait_for_hmi_start", False)
    hmi = None
    if bool(nav.get_parameter("hmi_enable").value):
        cmd_vel_topic = f"/{ns}/cmd_vel" if ns else "/cmd_vel"

        def _hmi_estop():
            # 콜백 컨텍스트 → spin 금지. goal 취소(비동기) + 바퀴 0속도.
            gh = getattr(nav, "goal_handle", None)
            if gh is not None:
                try:
                    gh.cancel_goal_async()
                except Exception:      # noqa: BLE001
                    pass
            hmi.stop_wheels()

        hmi = HmiLink(nav, robot_id=str(nav.get_parameter("hmi_robot_id").value),
                      cmd_vel_topic=cmd_vel_topic, on_estop=_hmi_estop)
        hmi.publish_state("대기", force=True)
    wait_start = hmi is not None and bool(nav.get_parameter("wait_for_hmi_start").value)

    # ★긴급정지 항상 활성 (hmi_enable=False 여도)★
    #   19_ 은 이 노드를 hmi_enable:=False 로 띄운다(process_state 이중발행 방지) → 위 HmiLink 의
    #   estop→Nav2 goal 취소 훅이 통째로 꺼져, "Nav2 로 주행 중 웹 긴급정지를 눌러도 안 멈추던" 문제.
    #   해결 = HmiLink 유무와 무관하게 /robot/command 를 얇게 직접 구독해 EMERGENCY_STOP 시 진행 중
    #   Nav2 goal 을 취소한다(상태발행은 안 하므로 19_ 의 process_state 와 충돌 없음).
    #   estop_state["on"] 으로 대기게이트·goal수신을 막아 취소된 goal 즉시 재수락 방지, START 로 해제.
    estop_state = {"on": False}
    if hmi is None:
        _RID_ALIAS = {"disinfect": "carter1", "waste": "carter2"}   # 구 역할고정 alias 호환
        def _on_cmd_raw(msg):
            nonlocal last_serviced_xy, last_serviced_t
            try:
                d = json.loads(msg.data)
            except Exception:      # noqa: BLE001
                return
            rid = d.get("robotId")
            rid = _RID_ALIAS.get(rid, rid)
            if rid not in (None, ns):          # 내 로봇(namespace) 또는 null(전체)만 반응
                return
            cmd = d.get("command")
            if cmd == "EMERGENCY_STOP":
                estop_state["on"] = True
                gh = getattr(nav, "goal_handle", None)
                if gh is not None:
                    try:
                        gh.cancel_goal_async()
                    except Exception:      # noqa: BLE001
                        pass
                print(f"[ESTOP] EMERGENCY_STOP 수신 → 진행 중 Nav2 goal 취소 (ns={ns or '(none)'})")
            elif cmd == "START":
                estop_state["on"] = False
                print(f"[ESTOP] START 수신 → 긴급정지 해제 (ns={ns or '(none)'})")
            elif cmd in ("MANUAL_OVERRIDE", "DOCK_RETURN"):
                # [2026-07-27 버그 수정] 19_ 이 수동제어/도킹복귀로 새 goal_pose 를 발행해도, 이
                # 노드는 goToPose() 가 blocking 이라 "현재 진행 중인 leg" 가 끝나기 전엔 새 goal
                # 을 아예 확인하지 않는다 — 그래서 도킹복귀를 눌러도 로봇이 기존 목적지로 계속
                # 가거나(carter2 증상), 새 goal 이 직전 도착지점과 GOAL_DEDUP_TOL 이내면 "잔류
                # 재발행"으로 오판돼 조용히 버려져 19_ 쪽이 도착 신호를 영원히 못 받고 멈춘다
                # (carter1 증상). estop 과 동일하게 즉시 goal 을 취소하고, dedup 상태도 리셋해
                # 곧 들어올 새(도킹/수동) goal 이 무시되지 않게 한다. estop_state 는 안 건드림
                # (여긴 "재개까지 대기"가 아니라 "즉시 새 목적지로 전환"이 목적).
                gh = getattr(nav, "goal_handle", None)
                if gh is not None:
                    try:
                        gh.cancel_goal_async()
                    except Exception:      # noqa: BLE001
                        pass
                last_serviced_xy = None
                last_serviced_t = 0.0
                print(f"[OVERRIDE] {cmd} 수신 → 진행 중 Nav2 goal 취소, 새 목적지 즉시 수신 준비 (ns={ns or '(none)'})")
        nav.create_subscription(String, "/robot/command", _on_cmd_raw, 10)

    # 구간(leg) → 프론트 WASTE_STEPS 라벨 매핑 (PICK→DUMP→RETURN→DOCK)
    DRIVE_LABELS = ["전방 주행", "수거함 이동", "수거통 원위치", "복귀"]
    ARRIVE_LABELS = ["폐기물통 파지", "폐기물 투하", "수거통 원위치", "복귀"]

    # Isaac g_run_nav_leg 는 /start_pick 받을 때까지 standoff goal 을 매 스텝 재발행한다.
    # 미션이 도착·/start_pick 후 다음 구간 goal 대기로 넘어갈 때, 큐에 남은 '직전 standoff'
    # 잔류 메시지가 다시 들어오면 '제자리(0.2m) 도착'하는 허수 구간이 되어 단계 라벨이
    # 실제 물리단계보다 앞서간다(파지 중인데 '투하' 표시). → 직전 도착 goal 과 같은 좌표의
    # 재수신은 무시한다(연속되는 실제 phase goal 은 항상 멀리 떨어져 있어 오판 없음).
    last_serviced_xy = None      # 직전에 SUCCEEDED 로 도착 완료한 goal (x,y)
    last_serviced_t = 0.0        # 그 도착 시각(monotonic). 시간창 밖이면 dedup 해제
    GOAL_DEDUP_TOL = 1.0         # m. 이보다 가까우면 같은 지점으로 간주
    GOAL_DEDUP_WINDOW = 5.0      # s. 도착 직후 이 시간 안의 같은 좌표 재수신만 무시(잔류).
                                 #    지나면 새 사이클(Isaac 재시작 등)의 정상 goal 로 받아들임.

    def _gate():
        """게이트: '첫 START 이전' 또는 '긴급정지 이후'에만 멈추고 '대기'를 표시한다.
        구간(leg) 사이 정상 전환에서는 상태를 건드리지 않는다 → 파지/이동 모션 중에
        '대기'가 깜빡이던 문제 해결(폐기물 미션은 leg 마다 이 루프를 다시 돈다)."""
        # hmi_enable=False 라도 긴급정지 상태면 START(estop_state 해제)까지 대기 → 취소된 주행 재개 방지.
        while rclpy.ok() and estop_state["on"]:
            print("[ESTOP] 긴급정지 상태 — START(해제) 대기 중")
            rclpy.spin_once(nav, timeout_sec=0.1)
        if hmi is None:
            return
        need_wait = hmi.estop or (wait_start and not hmi.active)
        if need_wait:
            hmi.publish_state("대기")
            print("[HMI] START 대기 중... (웹 '시작' 버튼)")
            while rclpy.ok() and not hmi.active:
                rclpy.spin_once(nav, timeout_sec=0.1)
        elif not wait_start:
            hmi.active = True     # 자동 시작(하위호환)

    # Isaac Sim 스크립트가 여러 구간(소형 쓰레기통 파지 -> big_trash 덤프 -> 이후 추가될
    # 원위치 복귀/도킹 복귀)마다 같은 토픽에 새 목표를 퍼블리시하므로, 한 번 처리하고
    # 끝내지 않고 계속 반복해서 받는다. Isaac Sim 쪽은 /start_pick 받을 때까지 같은
    # 목표를 계속 재발행하므로, 주행 실패/취소돼도 다음 반복에서 같은 목표를 다시 받아
    # 자동으로 재시도된다.
    leg = 0
    while rclpy.ok():
        _gate()                            # 웹 시작/재시작 게이트 (긴급정지 후 여기서 대기)
        leg += 1
        idx = min(leg - 1, len(DRIVE_LABELS) - 1)
        goal_holder["pose"] = None
        print(f"[WAIT] ({leg}구간) '{goal_topic}' 수신 대기 중...")
        goal_pose = None
        while rclpy.ok():
            rclpy.spin_once(nav, timeout_sec=0.5)
            if (hmi is not None and hmi.estop) or estop_state["on"]:   # 대기 중 긴급정지 → 다시 게이트로
                break
            g = goal_holder["pose"]
            if g is None:
                continue
            # 직전 도착지점의 '잔류 재발행'(도착 직후 짧은 시간)만 무시 → 허수 구간 방지.
            # 시간창을 지나 들어오는 같은 좌표는 새 사이클의 정상 goal 이므로 받는다.
            if (last_serviced_xy is not None
                    and (time.monotonic() - last_serviced_t) < GOAL_DEDUP_WINDOW):
                dx = g.pose.position.x - last_serviced_xy[0]
                dy = g.pose.position.y - last_serviced_xy[1]
                if (dx * dx + dy * dy) ** 0.5 < GOAL_DEDUP_TOL:
                    goal_holder["pose"] = None
                    continue
            goal_pose = g                       # 새 구간 goal 확정
            break
        if goal_pose is None:
            leg -= 1                        # 이 구간 미진행(estop) → 카운트 롤백
            continue

        if hmi is not None:
            hmi.publish_state(DRIVE_LABELS[idx])
        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        print(f"[NAV] ({leg}구간) 목표 수신: x={goal_pose.pose.position.x:.3f}, "
              f"y={goal_pose.pose.position.y:.3f} → 주행 시작")
        nav.goToPose(goal_pose)

        # 긴급정지 콜백이 goal 을 취소하면 isTaskComplete 가 자연히 완료(CANCELED)된다.
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                print(f"남은 거리: {feedback.distance_remaining:.2f} m")

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            # 이 좌표의 잔류 재발행을 이후(시간창 내) 무시. 실패/취소 시엔 갱신 안 함(재시도 허용).
            last_serviced_xy = (goal_pose.pose.position.x, goal_pose.pose.position.y)
            last_serviced_t = time.monotonic()
            print(f"[NAV] ({leg}구간) 목적지 도착 완료 → '{pick_topic}' 발행")
            if hmi is not None:
                hmi.publish_state(ARRIVE_LABELS[idx])
            for _ in range(10):
                pick_pub.publish(Bool(data=True))
                rclpy.spin_once(nav, timeout_sec=0.1)
        elif result == TaskResult.CANCELED:
            print(f"[NAV] ({leg}구간) 주행 취소됨 → '{pick_topic}' 발행 안 함")
            leg -= 1                        # 취소된 구간 롤백(재시작 시 같은 라벨)
        else:
            print(f"[NAV] ({leg}구간) 주행 실패 → '{pick_topic}' 발행 안 함")


if __name__ == "__main__":
    main()
