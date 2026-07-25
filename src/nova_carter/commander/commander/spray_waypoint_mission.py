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

from commander.hmi_link import HmiLink


def yaw_to_quat(yaw):
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class _Estop(Exception):
    """긴급정지로 미션 진행을 중단시키는 내부 신호."""
    pass


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

        # ── HMI(웹) 연동 ─────────────────────────────────────────────
        # hmi_enable=True 면 진행 단계를 /robot/disinfect/process_state 로 발행하고
        # /robot/command(START/EMERGENCY_STOP) 를 받는다.
        # wait_for_hmi_start=True 면 START 를 받을 때까지 "대기"로 머문다(웹에서 시작).
        # False(기본)면 기존처럼 실행 즉시 미션 개시(단독 실행 하위호환).
        self.declare_parameter("hmi_enable", True)
        self.declare_parameter("hmi_robot_id", "disinfect")
        self.declare_parameter("wait_for_hmi_start", False)
        # 첫 START 직후 carter1 localize 가 덜 정착돼 첫 goal 이 실패하는 경우가 잦다.
        # 한 번 실패로 그 지점을 건너뛰지 않고 N회 재시도(그 사이 AMCL 수렴) → "통합 시작
        # 눌러도 안 움직임" 완화. 0 이면 재시도 없음(기존 동작).
        self.declare_parameter("nav_retries", 3)
        # 16번 dual-SG : 첫 웨이포인트가 노즐 거치대(툴체인지)면 True. 스윕 라벨을 한 칸 밀어
        # i==0="노즐 장착", i==1="소독 분사", i>=2="유턴 재분사" 로 발행(프론트 DISINFECT_STEPS 매칭).
        self.declare_parameter("dock_first", False)

        self._active_goal_handle = None       # 진행 중 Nav2 goal (긴급정지 취소용)
        self._hmi = None
        if bool(self.get_parameter("hmi_enable").value):
            cmd_vel_topic = f"/{ns}/cmd_vel" if ns else "/cmd_vel"
            self._hmi = HmiLink(
                self,
                robot_id=str(self.get_parameter("hmi_robot_id").value),
                cmd_vel_topic=cmd_vel_topic,
                on_estop=self._hmi_estop,
            )
            self._hmi.publish_state("대기", force=True)

    # ── HMI 헬퍼 ────────────────────────────────────────────────
    def _publish_state(self, label):
        if self._hmi is not None:
            self._hmi.publish_state(label)

    def _check_estop(self):
        """긴급정지가 눌렸으면 _Estop 예외로 현재 미션 패스를 중단시킨다."""
        if self._hmi is not None and self._hmi.estop:
            raise _Estop()

    def _hmi_estop(self):
        """긴급정지 콜백(spin 컨텍스트): Nav2 goal 취소 + 스윕 취소 + 바퀴 0속도.
        여기서는 절대 spin 하지 않는다(재진입 금지)."""
        if self._active_goal_handle is not None:
            try:
                self._active_goal_handle.cancel_goal_async()
            except Exception:      # noqa: BLE001
                pass
        # 진행 중 스윕 취소(스크립트를 STANDBY 로). spin 없이 발행만.
        for _ in range(5):
            self._start_pub.publish(Bool(data=False))
        self._hmi.stop_wheels()

    def _gate(self):
        """미션 패스 시작 전 게이트. '대기' 표시 후, wait_for_hmi_start 면 START 대기."""
        self._publish_state("대기")
        if self._hmi is None:
            return
        if not bool(self.get_parameter("wait_for_hmi_start").value):
            self._hmi.active = True         # 자동 시작(하위호환)
            return
        self.get_logger().info("[HMI] START 대기 중... (웹 '시작' 버튼)")
        while rclpy.ok() and not self._hmi.active:
            rclpy.spin_once(self, timeout_sec=0.1)

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
        # 긴급정지가 goal 전송 도중 들어왔으면 즉시 취소
        if self._hmi is not None and self._hmi.estop:
            try:
                handle.cancel_goal_async()
            except Exception:      # noqa: BLE001
                pass
            return False
        self._active_goal_handle = handle     # 긴급정지 콜백이 취소할 수 있도록 보관
        try:
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            res = result_future.result()
        finally:
            self._active_goal_handle = None
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
            if self._hmi is not None and self._hmi.estop:   # 긴급정지 → 스윕 대기 중단
                return False
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

    # ── 미션 한 패스 (긴급정지 시 _Estop 로 중단됨) ───────────────
    def _run_pass(self, sweeps, settle):
        # 소독 단계 → 프론트 DISINFECT_STEPS 라벨 매핑:
        #   이동="복도 진입", 도착후 안정화="노즐 접촉", 스윕 라벨=아래 dock_first 분기, 복귀="복귀"
        # ★dock_first(16번 dual-SG)★ : 첫 웨이포인트가 "노즐 거치대"라 거기서의 start_sweep 은
        #   실제로는 소독이 아니라 "노즐 장착(툴체인지)"이다 → i==0 라벨을 "노즐 장착"으로,
        #   실제 첫 소독(i==1)을 "소독 분사"로 한 칸씩 민다. dock_first=False(기본, 13번)면 종전대로.
        dock_first = bool(self.get_parameter("dock_first").value)
        for i, (x, y, yaw, enable) in enumerate(sweeps):
            self._check_estop()
            self.get_logger().info(
                f"[{i+1}/{len(sweeps)}] → 이동 ({x:.2f},{y:.2f},{yaw:.2f}) sweep={enable}")
            # 거치대(dock_first, i==0)로의 이동은 사실상 제자리(carter1 이 이미 거치대 근처 스폰)라
            # "복도 진입"으로 표시하면 어색하다 → "노즐 접촉"으로 발행(물리 순서와 UI 순서 일치).
            self._publish_state("노즐 접촉" if (dock_first and i == 0) else "복도 진입")
            # ★거치대 웨이포인트 주행 생략★ : dock_first 의 첫 웨이포인트(거치대)는 carter1 스폰/홈
            #   (home_x,home_y=18.5,0)과 같은 좌표라, navigate_to 하면 '제자리(0거리) goal' 이 되어
            #   Nav2 가 목표 허용오차를 못 맞추고 그 자리에서 빙빙 도는(회전 recovery) 현상이 난다.
            #   carter1 은 이미 거치대에 있으니 주행을 건너뛰고 바로 파지(툴체인지)로 넘어간다.
            #   (home=거치대라 loop/복귀 후에도 항상 거치대에서 시작하므로 안전.)
            if dock_first and i == 0:
                self.get_logger().info("  [DOCK] 거치대=홈 좌표 → 주행 생략(제자리 회전 방지) → 바로 파지")
                arrived = True                       # 주행 없이 바로 파지 단계로
            else:
                retries = max(1, int(self.get_parameter("nav_retries").value))
                arrived = False
                for attempt in range(retries):
                    self._check_estop()
                    if self.navigate_to(x, y, yaw):
                        arrived = True
                        break
                    self._check_estop()              # 취소로 인한 실패면 여기서 중단
                    self.get_logger().warn(
                        f"  도착 실패 (시도 {attempt + 1}/{retries})"
                        + (" — 재시도" if attempt + 1 < retries else ""))
            if not arrived:
                self.get_logger().warn("  재시도 후에도 도착 실패 → 이 지점 건너뜀")
                continue
            self.get_logger().info("  도착.")
            if enable:
                self._check_estop()
                # 거치대(dock_first,i==0)=노즐 접촉→노즐 장착. 벽면=도착 후 바로 스윕(노즐 이미 장착).
                if dock_first and i == 0:
                    self._publish_state("노즐 접촉")   # 위 move 라벨과 dedup(중복 발행 억제됨)
                    self._settle(settle)
                    self._check_estop()
                    sweep_label = "노즐 장착"
                elif dock_first:
                    # 벽면: 별도 "노즐 접촉" 라벨 없이 안정화(UI 는 "복도 진입" 유지) → 뒤로 점프 방지.
                    self._settle(settle)
                    self._check_estop()
                    sweep_label = "소독 분사" if i == 1 else "유턴 재분사"
                else:
                    # 13번 호환(dock_first=False): 종전대로 벽면 접촉 라벨 사용.
                    self._publish_state("노즐 접촉")
                    self._settle(settle)
                    self._check_estop()
                    sweep_label = "소독 분사" if i == 0 else "유턴 재분사"
                self._publish_state(sweep_label)
                self.run_sweep()                     # /start_sweep → /sweep_done 대기
                self._check_estop()
                self._reset_sweep()                  # 확실히 STANDBY 로 되돌림(잔류 트리거 제거)

        # ── 모든 소독 완료 → 초기 도킹 스테이션(홈)으로 복귀 ──
        if bool(self.get_parameter("return_home").value):
            hx = float(self.get_parameter("home_x").value)
            hy = float(self.get_parameter("home_y").value)
            hyaw = float(self.get_parameter("home_yaw").value)
            self._reset_sweep()      # 스윕 확실히 STANDBY → 복귀 주행은 Nav2 가(/cmd_vel 충돌 방지, 팔 stow)
            self._check_estop()
            self._publish_state("복귀")
            self.get_logger().info(f"[HOME] 소독 완료 → 초기 도킹 복귀 ({hx:.2f},{hy:.2f},{hyaw:.2f})")
            if self.navigate_to(hx, hy, hyaw):
                self.get_logger().info("[HOME] 초기 도킹 도착.")
            else:
                self._check_estop()
                self.get_logger().warn("[HOME] 초기 도킹 복귀 실패(도달 불가/거부).")

    # ── 미션 실행 ────────────────────────────────────────────────
    def run(self):
        timeout = float(self.get_parameter("server_timeout").value)
        if not self._client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("[FAIL] navigate_to_pose 서버 없음. Nav2 실행 확인.")
            return False

        settle = float(self.get_parameter("settle_time").value)
        loop = bool(self.get_parameter("loop").value)
        wait_start = self._hmi is not None and \
            bool(self.get_parameter("wait_for_hmi_start").value)
        sweeps = self.sweeps()
        self.get_logger().info(f"미션 시작: 스윕 시작점 {len(sweeps)}개, loop={loop}")

        # 시작 시 이전 실행에서 갇힌 스윕이 있으면 취소(STANDBY 보장) → 이동 중 /cmd_vel 충돌 방지
        self._reset_sweep()

        while rclpy.ok():
            self._gate()                     # '대기' 표시 + (wait_for_hmi_start면) START 대기
            try:
                self._run_pass(sweeps, settle)
            except _Estop:
                self.get_logger().warn("[HMI] 긴급정지 → 미션 중단, 대기 복귀")
                self._reset_sweep()
                self._publish_state("대기")
                if wait_start:               # 웹 START 를 다시 기다림
                    continue
                break
            self._publish_state("대기")
            if loop:
                continue
            # 단일 실행 완료. 웹 제어 모드면 대기로 두고 다음 START 를 기다린다.
            if wait_start:
                self._hmi.active = False
                continue
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
