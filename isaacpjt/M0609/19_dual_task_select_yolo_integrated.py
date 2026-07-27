"""
19_dual_task_select_yolo_integrated.py  ★17_(작업선택식) + 18_(YOLO 사람회피) 병합★
================================================================================
계보 : 17_dual_task_select_tool_changer_integrated.py 가 이 파일의 ★구조적 베이스★다(사용자 결정).
carter1/carter2 각자 /carterN/task_select(String, trash|spray) 로 독립적으로 작업을 선택하는
17_의 RobotCtx/g_task_select_mission 구조를 그대로 유지하고, 그 위에
18_dual_sg_tool_changer_yolo_integrated.py 의 기능(YOLO 사람회피, 새 병원맵, 웹 HMI 긴급정지)을
얹었다. 18_은 carter1 전담(소독)/carter2 전담(폐기물)의 역할고정판 + carter1 전용 YOLO 였지만,
이 파일에서는 ★두 로봇 다 어떤 작업을 하든(trash 든 spray 든) YOLO 사람회피가 적용된다★(사용자
결정 — 18_의 "c2 는 YOLO 미사용" 제한을 폐기하고 양쪽으로 확장).

★범위(17_ 과 동일, 사용자 결정 유지)★ : 이번 작업도 Isaac 스크립트만. ROS 미션노드 교체
(spray_waypoint_mission 폐기, trash_can_nav_pick_mission 류 제너릭 포워더로 통일)와 웹 HMI
(로봇별 작업선택 UI) 연동은 여전히 범위 밖 — 17_/16_ 파일은 그대로 두고 이 파일을 병행 추가한다.

★알려진 블로커(17_에서 승계, 코드는 안 고침)★ : trash_can_nav_pick_mission.py 의
CARTER_START_POSE 가 carter2 스폰 좌표로 하드코딩돼 있어, namespace:=carter1 인스턴스를 그대로
띄우면 AMCL 초기위치가 틀어진다 — 라이브에서 carter1 trash 작업을 검증하려면 이 노드를 먼저
네임스페이스별 시작pose를 받도록 고쳐야 함(다음 단계, 여전히 미해결).

★이번 병합에서 바뀐 것★
  · 맵 : 17_의 외부 워크스페이스 참조(CARTER_NAV_WS/.../modified_hospital.usd) → 18_처럼 레포
    상대경로 src/assets/map/modified_hospital_2.usd 로 교체(이미 라이브 Nav2/HMI 와 동기화된
    최신 맵 — 인수인계 문서 확인).
  · YOLO 사람회피(PersonGate) : c1 뿐 아니라 ★c1+c2 둘 다★ 우측(3시) RealSense 장착
    (/carterN/realsense/color/image_raw 발행 → 외부 뷰어가 YOLO → /carterN/person_alert 구독).
    person_near 체크는 (a) g_spray_sweep(스윕 중 정지→다음 웨이포인트로 "완료" 취급, 18_ 그대로)
    (b) g_run_nav_leg 의 최종접근 구간(g_drive_straight_open_loop/g_rotate_in_place, ★17_에는
    없던 신규 삽입 지점★ — 18_에는 대응물이 없다. Nav2 가 goal 을 향해 주행하는 구간은 costmap이
    이미 사람을 피하므로 게이팅 불필요, 스크립트가 직접 cmd_vel 을 모는 구간만 게이팅)로 "제자리
    정지"만 한다.
  · 웹 HMI 긴급정지(/robot/command 구독) : 17_엔 없던 개념을 신규 이식. 18_의 비대칭 설계
    (c1=계속 전진+cmd_vel 덮어쓰기, c2=제너레이터 동결)는 이 파일의 대칭 구조(양쪽 다 동일한
    g_task_select_mission)엔 위험하므로(제너레이터를 얼리면 trash_lock/spray_lock 을 쥔 채 영구
    락업 가능) 채택하지 않음 — 대신 person-gate 와 동일한 "제자리 정지" 메커니즘을 공유한다.
    단 g_spray_sweep 안에서는 의미가 다르므로 반환값을 구분(사용자 결정) : person-block=True(완료
    취급, 다음 웨이포인트로 자동 진행) / estop=False(취소). g_spray_mission_body 가 애초에 이
    반환값을 안 보고 무조건 거치대 복귀를 시도하는 구조라, estop 취소 후 복귀 이동도 (b)의 패치로
    똑같이 제자리에서 멈춘다 — 별도 재시도/취소 로직 없이 "멈춘 자리에서 START 시 이어서 재개"가
    성립한다.
  · 사람(YOLO) 배치(build_people, 18_ 이식) : 18_의 PEOPLE 좌표는 18_ 고유의 역할고정 경로
    (c1 스윕 진입로/c2 trash·big_trash leg) 기준이라 17_/19_의 다른 스폰·도킹 기하와 안 맞는다 →
    "특정 태스크 전용 leg" 대신 "두 로봇이 어느 작업을 고르든 지나가는 공용 구간"(홈↔공용
    분사웨이포인트 북쪽 복도, 공용 쓰레기통 진입로, big_trash 덤프 가로 leg)에 재배치했다.
    ⚠ 좌표는 추정치 — 라이브 GUI로 통행 가능(벽 파묻힘 없음) 재확인 필요.

★게이팅 대상에서 제외(17_/18_ 컨벤션 유지)★ : 노즐 툴체인지(g_tool_change_grasp/release)는
거치대에서 정지 상태로만 이뤄지는 IK 동작(주행 없음)이라 person-gate/estop 둘 다 게이팅 대상에서
제외했다(16_ 문서의 기존 판단과 동일 이유).

⚠ 라이브 검증 필요(오프라인 헤드리스만 확인) : 사람 배치 좌표 통행성, 스윕 중/최종접근 중 사람
감지 정지 동작, estop 발행 시 락 보유 로봇의 정지+다른 로봇 대기 유지, 카메라 2대(c1/c2) topic
분리 — 모두 GUI+Nav2+YOLO 뷰어 라이브 확인 필요(17_/18_ 파일들과 동일 컨벤션).

실행(총 6개 터미널 권장 — 18_ 대비 YOLO 뷰어를 두 로봇용으로) :
  1) 이 스크립트 : python.sh isaacpjt/M0609/19_dual_task_select_yolo_integrated.py
  2) Nav2(멀티, carter1/carter2 네임스페이스)
  3) trash_can_nav_pick_mission --ros-args -p namespace:=carter1  (★AMCL 시작pose 블로커 참고★)
  4) trash_can_nav_pick_mission --ros-args -p namespace:=carter2
  5) YOLO 뷰어(시스템 python3, ROS 소싱) : multi_robot_yolo_viewer.py --robots carter1,carter2
     (뷰어가 이미 멀티로봇 인자를 지원한다는 전제 — 실행 시점에 --help 로 확인 권장, 이 파일 범위 밖)
  6) 트리거 : ros2 topic pub /carter1/task_select std_msgs/msg/String "data: 'spray'" --once
              ros2 topic pub /carter2/task_select std_msgs/msg/String "data: 'trash'" --once
     (또는 SELFTEST_TASK_C1/SELFTEST_TASK_C2 env var 로 헤드리스 자동 트리거)
================================================================================
"""
import os
import time

from isaacsim import SimulationApp

_LIVESTREAM = os.environ.get("LIVESTREAM", "0") == "1"
simulation_app = SimulationApp({
    "headless": os.environ.get("ISAAC_HEADLESS", "0") == "1" or _LIVESTREAM
})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
if _LIVESTREAM:
    enable_extension("omni.kit.livestream.webrtc")
simulation_app.update()

import sys
from pathlib import Path

import numpy as np
import omni.usd
import omni.timeline
import omni.graph.core as og
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Sdf, Gf, Vt

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.sensors.camera import Camera          # ★YOLO★ 우측(3시) RealSense 컬러 캡처(c1+c2)

import json

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image as RosImage        # ★YOLO★ RealSense 컬러를 ROS 로 발행(뷰어가 YOLO)

_THIS_DIR = Path(__file__).resolve().parent                 # isaacpjt/M0609
_WS_ROOT = _THIS_DIR.parent.parent                           # 레포 루트(clone 위치 무관)

_C2_RMPFLOW_DIR = str(_WS_ROOT / "src" / "integration" / "integration" / "rmpflow")
if _C2_RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, _C2_RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

_TOOL_CHANGER_DIR = str(_WS_ROOT / "src" / "isaac_wiping_task")
if _TOOL_CHANGER_DIR not in sys.path:
    sys.path.insert(0, _TOOL_CHANGER_DIR)
import surface_gripper_utils  # noqa: E402
from tool_changer import ToolChangerController  # noqa: E402


# ════════════════════════════════════════════════════════════════════════════
#  A. 공통 경로/상수
# ════════════════════════════════════════════════════════════════════════════
# [19_ 병합] 17_의 외부 워크스페이스 참조 대신 18_처럼 레포 상대경로(태성님 갱신판, 이미 라이브
# Nav2/HMI 와 동기화됨)를 쓴다 — 인수인계 문서 확인상 modified_hospital_2.usd 가 최신.
HOSPITAL_USD = str(_WS_ROOT / "src" / "assets" / "map" / "modified_hospital_2.usd")
NOZZLE_USD = str(_WS_ROOT / "src" / "integration" / "integration" / "m0609_with_nozzle.usd")
MOVE_TRASH_USD = str(_WS_ROOT / "src" / "assets" / "scenes" / "move_tash_can.usd")
PEOPLE_USD = str(_WS_ROOT / "src" / "assets" / "props" / "people.usd")   # ★YOLO 병합★ 18_ 이식

PHYSICS_DT = 1.0 / 60.0
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]

# [2대 동시구동 둔감함 완화, 실측: ENABLE_C2=0 단독은 15_ 수준으로 쾌적 → 순수 2배 구조비용 확정]
# RENDER_EVERY↑ = 물리/제어(60Hz)는 그대로, 렌더+카메라/포인트클라우드 발행 주기만 늦춤(저위험).
# env var 로 재빌드 없이 실험 가능(RS_PUBLISH_EVERY 와 동일 패턴).
# [2026-07-27] 3으로 승격 시도했다가, 라이브에서 캐스터 보정 저하 + RS_ON=1 과 결합 시 악화 발견
# → 4는 더 심해짐(사용자 실측) → 2로 고정. RENDER_EVERY 값 자체가 캐스터/카메라 스톨과 얽히는
# 부작용이 있는 것으로 보여 3 이상은 당분간 권장하지 않음.
RENDER_EVERY = int(os.environ.get("RENDER_EVERY", "2"))
# [2026-07-27] RENDER_EVERY 는 2로 고정(캐스터 부작용) → 대신 "매 렌더당 비용" 자체를 줄이는 쪽으로.
# CAM_DEACTIVATE_UNUSED 로 11개는 이미 꺼져있어서, 실제로 매 렌더마다 무거운 카메라는 front_hawk
# (YOLO 뷰어 기본 구독 대상) 하나뿐 — 이 해상도를 낮추는 게 스텝 타이밍(캐스터와 얽힌 원인)은
# 안 건드리면서 "개별 렌더+읽기 작업의 소요시간"(픽셀수 비례, GPU 평균사용률과는 별개 축)을 줄이는
# 저위험 레버. 라이브 실측 : 640x400→480x300 만으로도 캐스터 보정 체감 개선 확인 → 320x240 으로
# 추가 축소. YOLO 자체가 입력을 imgsz=640 으로 리사이즈해서 쓰므로 소스 해상도를 낮춰도 감지 품질
# 손해는 제한적일 것으로 예상(라이브에서 감지율 저하 없는지 확인 필요).
CAM_RENDER_W = int(os.environ.get("CAM_RENDER_W", "320"))
CAM_RENDER_H = int(os.environ.get("CAM_RENDER_H", "240"))
# [19_ GPU 절감] Nova Carter 내장 카메라 12개(hawk 8+owl 4) 중 front_hawk 만 실사용(외부 YOLO
# 뷰어 기본값 front_stereo_camera/left) — 그 외(front_hawk 의 right 포함, right/left/back_hawk,
# owl)는 아무도 안 봄. keep_full_res 패턴에 안 걸리는 카메라는 CAM_RENDER_W/H 대신 이 극소
# 해상도로 낮춰 렌더 비용을 거의 0으로(OmniGraph 구조는 안 건드리는 저위험 접근, 사용자 결정).
CAM_KEEP_FULL_RES = ("front_hawk",)
CAM_MINIMIZE_W = 16
CAM_MINIMIZE_H = 16
# 16x16 으로 줄여도 render product 자체는 여전히 매 프레임 평가된다 — 아무도 안 보는 11개(owl 4 +
# hawk 3방향)를 통째로 SetActive(False) 하면(해당 OmniGraph 노드 1개만 비활성화, 그래프 나머지는
# 안 건드림) 렌더 패스 자체가 안 돎 → 해상도 축소보다 더 아낌. [2026-07-27] 헤드리스 스모크테스트
# (씬 빌드/ROS2 브리지 부작용 없음) + 라이브 검증(쾌적함 체감) 완료돼 기본값으로 승격.
# CAM_DEACTIVATE_UNUSED=0 으로 끄면 기존(16x16 축소만) 동작으로 되돌아감.
CAM_DEACTIVATE_UNUSED = os.environ.get("CAM_DEACTIVATE_UNUSED", "1") == "1"

NS_CARTER1 = "carter1"
NS_CARTER2 = "carter2"

C1_SCOPE = "/World/Carter1"
C2_SCOPE = "/World/Carter2"

MOUNT_OFFSET = Gf.Vec3d(-0.2317, 0.0, 0.5773)
DRIVE_MAX_ANG_SPEED = 2.5
DRIVE_MAX_ANG_ACCEL = 6.0
DRIVE_MAX_LIN_SPEED = 1.2


# ════════════════════════════════════════════════════════════════════════════
#  B. carter1 (소독+폐기물 겸용) 프림 경로
# ════════════════════════════════════════════════════════════════════════════
C1_CARTER_PRIM = f"{C1_SCOPE}/Nova_Carter_ROS"
C1_CHASSIS = f"{C1_CARTER_PRIM}/chassis_link"
C1_ARTICULATION_ROOT = C1_CHASSIS
C1_ARM = f"{C1_SCOPE}/m0609"
C1_ARM_ROOT = C1_ARM
C1_EE_LINK_NAME = "link_6"
C1_SURFACE_GRIPPER = f"{C1_ARM}/{C1_EE_LINK_NAME}/mop_surface_gripper"
C1_TRASH_CAN_PRIM = f"{C1_SCOPE}/small_trash_can_body"   # 공용 쓰레기통 아님 — 항상 비활성화
C1_EXTRA_PHYSICS = f"{C1_SCOPE}/PhysicsScene"
C1_GROUND_PLANE = f"{C1_SCOPE}/GroundPlane"

C1_START_POSE = dict(x=18.5, y=0.2317, z=0.08, yaw_deg=90.0)     # docking_station_1
# [2026-07-26 사용자 요청] 스폰 직후 챠시가 살짝 움찔거리는 현상 관찰 — 바닥/도킹스테이션 데칼
# 메시와 z 방향 여유가 거의 없어(특히 carter2 는 원래 z=0.0) 첫 물리 스텝에서 접촉보정으로 튀는
# 것으로 추정. 두 로봇 다 z 를 살짝(0.03~0.08m) 올려 여유를 둠.
# [2026-07-26 사용자 요청 2] 팔 베이스가 도킹스테이션 중심(원래 x,y=18.5,0.0)에 오도록 y 를
# +0.2317(=-MOUNT_OFFSET.x, yaw=90도에서 로컬 -X 오프셋이 그대로 월드 -Y 오프셋이 됨) 만큼
# 보정 — 팔 베이스 실측 (18.50,-0.23,0.66) 확인 후 역산. 이제 챠시가 (18.5,0.2317)에 스폰돼야
# 팔 베이스가 (18.5,0.0)에 정확히 옴(HOME1_XY 도 같이 이동하므로 도킹 복귀 지점도 일관되게 이동).
# [2026-07-26 사용자 요청] 스폰/docking_station 복귀 방향을 +Y 로 통일(기존 0도=+X). 노즐 거치대
# 파킹 각도(DOCK1_PARK_YAW, 후진 진입 최종 방향)는 이미 +Y 라 이제 셋 다 일치함. ★주의★: ROS 쪽
# trash_can_nav_pick_mission.py 의 CARTER_START_POSES["carter1"] yaw 도 반드시 같이 90.0 으로
# 맞춰야 함 — 안 맞추면 AMCL 초기pose 가 실제 스폰 방향과 달라져 위치추정이 처음부터 틀어짐.


# ════════════════════════════════════════════════════════════════════════════
#  C. carter2 (폐기물+소독 겸용) 프림 경로 — 공용 쓰레기통을 내장한 쪽
# ════════════════════════════════════════════════════════════════════════════
C2_SCOPE_PRIM = f"{C2_SCOPE}/Nova_Carter_ROS"
C2_ARTICULATION_ROOT = f"{C2_SCOPE}/Nova_Carter_ROS/chassis_link"
C2_ARM = f"{C2_SCOPE}/m0609"
C2_ARM_ROOT = C2_ARM
C2_EE_LINK_NAME = "link_6"
C2_SURFACE_GRIPPER = f"{C2_ARM}/{C2_EE_LINK_NAME}/mop_surface_gripper"
TRASH_CAN_PRIM = f"{C2_SCOPE}/small_trash_can_body"          # ★공용 쓰레기통(유일 인스턴스)★
C2_EXTRA_PHYSICS = f"{C2_SCOPE}/PhysicsScene"
C2_GROUND_PLANE = f"{C2_SCOPE}/GroundPlane"

C2_START_POSE = dict(x=16.66290495232035, y=-0.0029517927591273807 + 0.2317, z=0.08, yaw_deg=90.0)  # docking_station_02

ARM_URDF = str(_THIS_DIR / "rmpflow" / "m0609_isaac_sim.urdf")
NO_GRIPPER_URDF = str(_WS_ROOT / "src" / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf")


# ════════════════════════════════════════════════════════════════════════════
#  D. 로봇 공용 상수(팔 기하·트래시·분사·툴체인지) — 두 로봇이 동일 arm 모델·동일 쓰레기통·
#     동일 분사지점을 상대하므로 로봇별로 나눌 이유가 없다(ctx 로 개별화하는 건 dock/spray 좌표뿐).
# ════════════════════════════════════════════════════════════════════════════
IK_URDF_PATH = ARM_URDF
IK_DESCRIPTION_PATH = str(_THIS_DIR / "rmpflow" / "m0609_description.yaml")
EE_FRAME = "link_6"

TARGET_JOINTS_DEG = [-90.0, 101.0, 50.0, -94.0, 91.8, -1.1]
TUCK_J1_DEG = 10.0
DUMP_J1_DEG = TUCK_J1_DEG
DUMP_J6_ROTATE_DEG = -180.0
DUMP_RAMP_STEPS = 240
POST_DUMP_BACKUP_DISTANCE = 0.6
POST_RETURN_BACKUP_DISTANCE = 0.6

BIG_TRASH_POSITION_XY = np.array([4.0, 7.5])
BIG_TRASH_APPROACH_DIR = np.array([1.0, 0.0])
BIG_TRASH_STANDOFF_DISTANCE = 1.3
BIG_TRASH_FINAL_DISTANCE = 0.5

LIFT_OFFSET = np.array([0.0, 0.0, 0.75])
POSITION_TOLERANCE = 0.03
MAX_APPROACH_STEPS = 400
RETURN_PLACE_GROWING_TOLERANCE_MAX = 0.25
MAX_EE_LINEAR_SPEED = 0.10
MIN_INTERP_STEPS = 60
MAX_INTERP_STEPS = 600
SETTLE_STEPS = 60
GRASP_HOLD_STEPS = 60
JOINT_RAMP_STEPS = 300
CREEP_STEP_SIZE = 0.005
CREEP_MAX_STEPS = 60
CREEP_SETTLE_STEPS = 5
LATERAL_CORRECTION_MIN = 0.01
LATERAL_CORRECTION_MAX = 0.4

TRASH_SPAWN_FIXED = (14.0, 6.5)
TRASH_SPAWN_Z = 0.0786364536328308
TRASH_BBOX_CENTER_OFFSET_XY = np.array([0.15, 0.15])
OFFSET_TRASH_FROM_CHASSIS = np.array([-0.750368208, -0.758035257])

FINAL_APPROACH_DISTANCE = 0.4
FINAL_APPROACH_SPEED = 0.15
FINAL_APPROACH_RAMP_TIME = 0.5
FINAL_APPROACH_DRIFT_KP = 0.15
FINAL_APPROACH_DRIFT_KD = 0.4
FINAL_APPROACH_DRIFT_W_MAX = 0.15
FINAL_APPROACH_DRIFT_MAX_W_STEP = 0.01
FINAL_ROTATE_KP = 2.0
FINAL_ROTATE_KD = 0.5
FINAL_ROTATE_W_MAX = 1.0
FINAL_ROTATE_MAX_W_STEP = 0.05
FINAL_ROTATE_TOLERANCE_RAD = 0.02
FINAL_MOVE_SETTLE_STEPS = 30
FINAL_NUDGE_DISTANCE = 0.00
PRE_ROTATE_NUDGE_DISTANCE = 0.25

DOCK_APPROACH_SKIP_XY_RADIUS = 0.15    # 이 안이면 "이미 거치대 근처"로 보고 nav-leg 생략
DOCK_APPROACH_SKIP_YAW_TOL = np.radians(15.0)

# ── 노즐 거치대(로봇별 2개) ──
NOZZLE_SOURCE_PRIMPATH = "/World/m0609/nozzle_base_link"   # tool0_to_nozzle 조인트 밖(형제) — 안 딸려옴
NOZZLE_DOCK_HEIGHT = 0.65
DOCK_STANDOFF = 0.80        # 챠시 파킹지점-거치대 팔 접근거리. 접근방향은 -Y(사용자 결정).
                            # [2026-07-26] 원래 15_/16_ 검증값 0.35 → 전진진입으로 0.45/0.40/0.60(성공)/
                            # 1.00(실패, "노즐 상공 접근" IK 부터 안 풀림) 순으로 라이브 확인. 사용자가
                            # 팔 스펙 reach=0.9m 라고 알려줘서, 팔이 챠시 뒤쪽에 달린 걸 이용해 "후진
                            # 진입"(g_run_nav_leg reverse_entry=True)으로 전환 — 후진 진입 시 팔↔노즐
                            # 수평거리 = DOCK_STANDOFF-0.2317 이라 같은 팔 reach 로 훨씬 더 먼
                            # DOCK_STANDOFF 를 쓸 수 있음(계산상 이론 한계 ~1.06m).
                            # [2026-07-27 라이브 실패 대응] 0.85 로 시작했다가 실측: "노즐 상공 접근"·
                            # "노즐 하강" 둘 다 link_6 오차 |e|≈42.7~42.8mm 로 수렴(그립 범위 0.04m
                            # 를 살짝 초과해 파지 실패) — 무작위 오차가 아니라 두 단계에서 거의 동일한
                            # 부족분이라 IK 도달범위 부족으로 판단. 관측 부족분(~43mm)보다 여유있게
                            # 50mm 줄여 0.80 으로 재시도. ★라이브 재검증 필요★ — 이래도 실패하면
                            # (오차 폭으로 얼마나 더 줄여야 하는지 가늠 가능) 추가로 낮출 것.
# [사용자 요청] 거치대가 스폰 바로 옆(0.35m)이라 너무 가까워 보임 + Nav2 실주행 테스트를 위해서도
# "스폰↔거치대"가 제자리 회전만으로 끝나면 의미가 없다 — 그래서 거치대 자체는 스폰에서
# DOCK_HOME_DISTANCE 만큼 멀리 두고, 파킹지점(DOCKn_APPROACH_XY)만 거치대에서 DOCK_STANDOFF(검증된
# IK reach 거리, 불변) 만큼 못미친 곳에 둔다 — nav-leg 가 스폰→파킹지점 구간을 실제로 주행하게 됨.
DOCK_HOME_DISTANCE = 1.5    # 스폰-거치대 거리[m]. ⚠ 라이브 미검증(주변 벽/장애물과 안 부딪히는지 확인 필요).

# [2026-07-26 사용자 요청] 노즐이 허공에 매달려만 있어 실제 거치대 물리 형상(기둥+캔틸레버 암)을
# 추가한다. 로봇은 항상 노즐 북쪽(+Y, DOCKn_APPROACH_XY 방향)에서 접근해 파킹하므로(도킹 구간이
# g_run_nav_leg 한 번의 회전→직진→회전으로 끝나고 그 이후 챠시가 더 움직이지 않음 — 정밀보정 루프는
# 제거됨, 아래 참고), 기둥을 노즐 남쪽(-Y)으로 오프셋해 세우면 챠시 파킹 구역과 기하학적으로 겹치지
# 않는다. 손잡이(NOZZLE_DOCKn_XY, NOZZLE_DOCK_HEIGHT) 바로 위 TC_EE_OFFSET(0.15m) 수직 통로는
# 캔틸레버 암이 절대 덮지 않음(암은 손잡이와 같은 높이에서 옆으로만 붙는다) — 팔이 위에서 곧장 하강해
# 파지하는 경로를 막지 않기 위함.
NOZZLE_STAND_POLE_OFFSET = 0.20      # 기둥을 손잡이에서 남쪽(-Y)으로 띄우는 거리[m]
NOZZLE_STAND_POLE_RADIUS = 0.03
NOZZLE_STAND_ARM_RADIUS = 0.02       # 기둥→손잡이 캔틸레버 암 반지름[m]
NOZZLE_STAND_BASE_RADIUS = 0.12      # 바닥 받침 원판 반지름[m]
NOZZLE_STAND_BASE_HEIGHT = 0.03
# 단계적 롤아웃: 처음엔 형상만 보이고 충돌은 꺼서 배치를 라이브로 먼저 눈으로 확인 — 배치 확정되면
# True 로 바꿔 실제 충돌 활성화(형상을 다시 만들 필요 없이 이 플래그만 바꾸면 됨).
NOZZLE_STAND_COLLISION_ENABLED = False

NOZZLE_DOCK1_SCOPE = "/World/NozzleDock1"
NOZZLE_TOOL_PATH_C1 = f"{NOZZLE_DOCK1_SCOPE}/nozzle_tool"
NOZZLE_TCP_PATH_C1 = f"{NOZZLE_TOOL_PATH_C1}/nozzle_tcp"
NOZZLE_HOLD_JOINT_PATH_C1 = f"{NOZZLE_DOCK1_SCOPE}/hold_joint"
NOZZLE_HOLD_ANCHOR_PATH_C1 = f"{NOZZLE_DOCK1_SCOPE}/hold_anchor"
NOZZLE_DOCK1_XY = np.array([C1_START_POSE["x"], C1_START_POSE["y"] - DOCK_HOME_DISTANCE])
DOCK1_APPROACH_XY = NOZZLE_DOCK1_XY + np.array([0.0, DOCK_STANDOFF])
DOCK1_APPROACH_YAW = -np.pi / 2.0     # 남쪽(-Y)을 보고 거치대 접근(Nav2 이동 방향 기준)
DOCK1_PARK_YAW = DOCK1_APPROACH_YAW + np.pi   # [2026-07-26] 후진 진입 최종 파킹 방향(북쪽) — 아래 참고

NOZZLE_DOCK2_SCOPE = "/World/NozzleDock2"
NOZZLE_TOOL_PATH_C2 = f"{NOZZLE_DOCK2_SCOPE}/nozzle_tool"
NOZZLE_TCP_PATH_C2 = f"{NOZZLE_TOOL_PATH_C2}/nozzle_tcp"
NOZZLE_HOLD_JOINT_PATH_C2 = f"{NOZZLE_DOCK2_SCOPE}/hold_joint"
NOZZLE_HOLD_ANCHOR_PATH_C2 = f"{NOZZLE_DOCK2_SCOPE}/hold_anchor"
NOZZLE_DOCK2_XY = np.array([C2_START_POSE["x"], C2_START_POSE["y"] - DOCK_HOME_DISTANCE])
DOCK2_APPROACH_XY = NOZZLE_DOCK2_XY + np.array([0.0, DOCK_STANDOFF])
DOCK2_APPROACH_YAW = -np.pi / 2.0
DOCK2_PARK_YAW = DOCK2_APPROACH_YAW + np.pi

# [2026-07-26 사용자 제안] 팔(MOUNT_OFFSET x=-0.2317, 챠시 원점보다 뒤쪽 장착)이 노즐 거치대에 "후진
# 진입"하면, 전진 진입 대비 팔↔노즐 수평거리가 2*0.2317m 만큼 짧아져 같은 팔 도달거리(스펙 0.9m)로도
# DOCK_STANDOFF 를 훨씬 크게 잡을 수 있다(전진: DOCK_STANDOFF+0.2317, 후진: DOCK_STANDOFF-0.2317).
# g_run_nav_leg(reverse_entry=True) 로 목표 반대방향을 보고 후진으로 진입 — 도착 시 이미 거치대 반대
# 방향(DOCKn_PARK_YAW)을 보고 있으므로 파지/반납 직후 별도 후진+180도 회전(g_backup_and_turn_away)도
# 불필요해짐(이미 그 방향을 보고 있어 바로 다음 leg 로 갈 수 있음).

# [사용자 요청] 노즐 거치대 파킹지점(DOCKn_APPROACH_XY)이 원래 스폰지점(docking_station_1/02,
# C1/C2_START_POSE)과 달라져(y 로 DOCK_HOME_DISTANCE-DOCK_STANDOFF 만큼 벌어짐) 작업 완료 후 로봇이
# 원래 도킹스테이션이 아닌 곳에 최종 파킹되는 문제 발견 — trash/spray 작업 완료 시 마지막에 원래
# docking_station_1/02 로 복귀하는 nav-leg 를 추가한다(HOME1/2_XY/YAW).
HOME1_XY = np.array([C1_START_POSE["x"], C1_START_POSE["y"]])
HOME1_YAW = np.radians(C1_START_POSE["yaw_deg"])
HOME2_XY = np.array([C2_START_POSE["x"], C2_START_POSE["y"]])
HOME2_YAW = np.radians(C2_START_POSE["yaw_deg"])

# Surface Gripper 튜닝(15_/16_ 검증값 — grip_travel 을 IK 접근오차보다 넉넉히, clearance 는 최소).
# [2026-07-27 재보정] MAX_GRIP_DISTANCE=0.04(4cm) 는 D6 조인트 컴플라이언스(GRIP_TRAVEL=1.5cm)
# 보다 훨씬 넉넉해서, 트래시 크립(게이팅 없음, 매 스텝 즉시 grip 시도)이 물체에서 최대 4cm 떨어진
# 상태로도 바로 잡혀버려 눈에 띄는 틈이 남는 원인이 됐다(사용자 라이브 관찰). 노즐 쪽은 이미
# TC_CREEP_GRIP_ATTEMPT_RADIUS=GRIP_TRAVEL(1.5cm)로 게이팅해뒀으니, 이 물리적 그립범위 자체도
# GRIP_TRAVEL 과 맞춰 — "조인트가 실제로 컴플라이언스로 흡수할 수 있는 거리 안에서만 잡힌다"로
# 통일(기하 추정 불필요, 하드 물리 제약이라 안전). 노즐은 기존 게이팅과 값이 같아져 동작 변화 없음,
# 트래시는 게이팅이 없던 채로 그립 자체가 더 가까워야만 성공하게 돼 틈이 줄어들 것으로 기대.
MAX_GRIP_DISTANCE = 0.015
GRIP_DRIVE_STIFFNESS = 5000.0
GRIP_DRIVE_DAMPING = 100.0
CLEARANCE_OFFSET = 0.0005
GRIP_TRAVEL = 0.015

# 툴체인지 접근/파지 상수(15_/16_ 검증값 — 수직 매달기).
TC_APPROACH_ORIENTATION = np.array([0.0, 1.0, 0.0, 0.0])   # 로컬 Z가 world -Z(팁 아래로)
TC_EE_OFFSET = np.array([0.0, 0.0, 0.15])
TC_GRASP_CLEARANCE = np.zeros(3)
TC_FINGERTIP_OFFSET_FROM_TOOL0 = np.zeros(3)                # 맨몸 팔 : fingertip(link_6)≈tool0
TC_GRASP_SETTLE_STEPS = 30
TC_REDOCK_SETTLE_STEPS = 20
TC_JOINT_RAMP_STEPS = 200
TC_IK_RETRY_COUNT = 2   # [2026-07-26] IK 실패 시 그 자리(챠시 이동 없음)에서 재시도 횟수(최대 3회 시도)
TC_TARGET_BIAS_COMPENSATION = np.array([0.006, -0.0003, 0.0])   # URDF-USD 형상 불일치 고정 바이어스
# [2026-07-27 재보정 추가] IK 1회 솔브(_tc_solve_and_ramp)만으로는 챠시 접근 오차(Nav2 핸드오프
# 오차)가 그대로 그립 실패로 이어지는 구조적 약점이 있었다(38mm 오차 실측, g_tool_change_grasp
# 참고). g_trash_mission 의 검증된 크립(creep) 패턴을 그대로 이식 — 실측 handle_position 을 향해
# RMPflow 로 짧게 끊어 다가가며(매 스텝 settle) 재그립 시도. 트래시와 동일 해상도(5mm)로 시작,
# 최대거리(200mm)는 트래시(300mm)보다 좁게(관측된 부족분 38mm 대비 5배 여유).
TC_CREEP_STEP_SIZE = 0.005
TC_CREEP_MAX_STEPS = 40
TC_CREEP_SETTLE_STEPS = 5
# [2026-07-27 라이브 피드백] 크립 첫 스텝부터 매번 grip 을 시도하면, MAX_GRIP_DISTANCE(0.04m) 범위에
# "막 들어온" 먼 지점(가장자리)에서 바로 잡혀버려 노즐이 손끝 중심이 아니라 팔 가장자리에 붙은
# 것처럼 보이는 문제 발견 — target(handle_position)에 이 반경 안으로 들어올 때까지는 grip 을
# 시도하지 않고 계속 다가가기만 한다. GRIP_TRAVEL(D6 조인트 컴플라이언스 범위)과 동일값으로
# 시작 — 그 범위 안에서 잡히면 자연스러운 자세로 보정될 여지가 있다.
TC_CREEP_GRIP_ATTEMPT_RADIUS = 0.015

WALL_X = 0.575
AIM_Y = 0.0
Z_LOW = 0.12
Z_HIGH = 0.80
J1_INDEX = 0
J1_OFFSET = -np.pi / 2
J5_INDEX = 4
J5_FLICK = -0.5

S_CRUISE = 2.0
S_ACCEL = 3.0
S_HOLD_STEPS = 6
STOW_Q = np.array([0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0])
MAX_JOINT_STEP = 0.06
NAV_STOW_Q_DEG = [0.0, 90.0, -90.0, 0.0, 0.0, 0.0]
SPRAY_ENTRY_RAMP_STEPS = 220

STROKES_PER_WIPE = 2
MOVE_DISTANCE = 0.25
FORWARD_DISTANCE = 9.5     # [2026-07-26 사용자 요청] 스윕이 벽까지 도달해 챠시가 끼는 문제 발견
                           # (Nav2 backup/spin 복구도 실패, 로봇 전복까지 간 적 있음) — 분사
                           # 사이클 약 2번 분(~1.0m, 사이클당 progress 증가폭 약 0.46~0.5m 실측
                           # 기준) 줄여서 벽에서 여유를 둠. 그래도 벽에 닿으면 더 줄일 것.
FORWARD_SPEED = 0.35
FORWARD_ACCEL = 0.50
KP_YAW = 2.5
KP_LAT = 2.0
KD_YAW = 0.4
W_MAX = 0.8

SPRAY_FX_ON = True
SPRAY_MAX = 700
SPRAY_RATE = 12
SPRAY_SPEED = 2.5
SPRAY_CONE_DEG = 120.0
SPRAY_LIFETIME = 0.5
SPRAY_GRAVITY = -2.0
SPRAY_SIZE = 0.012
SPRAY_OPACITY = 0.6
SPRAY_COLOR = (0.55, 0.85, 1.0)

# 분사 벽 웨이포인트(공용, spray_lock 로 상호배제) — spray_waypoint_mission.py 기존 검증 좌표 재사용.
SPRAY_WP1_XY = np.array([18.8, 8.0])
SPRAY_WP1_YAW = np.radians(90.0)

# ── ROS 토픽(로봇별) ──
C1_TASK_SELECT = f"/{NS_CARTER1}/task_select"
C1_NAV_GOAL = f"/{NS_CARTER1}/trash_can_nav_goal"
C1_START_PICK = f"/{NS_CARTER1}/start_pick"
C1_CMD_VEL = f"/{NS_CARTER1}/cmd_vel"
C1_HMI_STATE = f"/{NS_CARTER1}/process_state"   # [HMI v2] 19_ 이 직접 발행(hmi_link.py 미경유)

C2_TASK_SELECT = f"/{NS_CARTER2}/task_select"
C2_NAV_GOAL = f"/{NS_CARTER2}/trash_can_nav_goal"
C2_START_PICK = f"/{NS_CARTER2}/start_pick"
C2_CMD_VEL = f"/{NS_CARTER2}/cmd_vel"
C2_HMI_STATE = f"/{NS_CARTER2}/process_state"   # [HMI v2]


# ════════════════════════════════════════════════════════════════════════════
#  D-2. ★YOLO 병합★ 사람(people.usd) 에셋 + 회피 게이트 상수 (18_ 이식, c1+c2 양쪽으로 확장)
# ════════════════════════════════════════════════════════════════════════════
# people.usd = Isaac People 캐릭터들(원격 payload) + Plane/Environment/Light. 통째 참조 금지 →
# 아래 PEOPLE 목록의 (캐릭터 프림명, x, y, yaw_deg) 로 "캐릭터 프림만" 개별 참조해 배치한다.
PEOPLE_SCOPE = "/World/People"       # 사람들을 모아두는 스코프(환경과 분리)
# people.usd 안의 캐릭터 프림 이름들(개별 참조 대상). 18_에서 pxr 검사로 확인됨.
PEOPLE_CHARACTER_PRIMS = [
    "/World/F_Business_02",
    "/World/M_Medical_01",
    "/World/female_adult_business_02",
]
# [19_ 재배치] 18_의 PEOPLE 좌표는 18_ 고유의 역할고정 경로(c1 스윕 진입로/c2 trash·big_trash leg)
# 기준이라 17_/19_ 의 다른 스폰·도킹 기하(C1/C2_START_POSE, yaw=90 등)와 안 맞는다 — "특정 태스크
# 전용 leg" 대신 "두 로봇이 어느 작업을 고르든 지나가는 공용 구간"에 재배치. ⚠ 좌표 추정치, 라이브
# GUI 통행성(벽 파묻힘 없음) 재확인 필요(idx = PEOPLE_CHARACTER_PRIMS 순환 인덱스, name, x, y, yaw_deg).
PEOPLE = [
    # 홈(y≈0) ↔ 공용 분사 웨이포인트(SPRAY_WP1_XY=18.8,8.0) 사이 북쪽 복도 — spray 작업 시
    # 두 로봇 다 지나감.
    ("person_spray_corridor", 18.5, 4.5, -90.0),
    # 공용 쓰레기통(TRASH_SPAWN_FIXED=14.0,6.5) 진입로 — trash 작업 시 두 로봇 다 지나감.
    ("person_trash_approach", 15.5, 3.0, -135.0),
    # 공용 big_trash 덤프(BIG_TRASH_POSITION_XY=4.0,7.5)로 가는 긴 가로 leg(18_ person_c2_leg2a
    # 좌표 재사용 — 이 구간은 17_/19_에서도 기하학적으로 안 바뀜).
    ("person_dump_leg", 10.0, 7.5, 100.0),
]
PERSON_COLLIDER_RADIUS = 0.30        # 사람 자리에 겹치는 '보이지 않는' 물리 collision 실린더 반경[m]
PERSON_COLLIDER_HEIGHT = 1.8         # 실린더 높이[m] (사람 키 정도)
PERSON_COLLIDER_VISIBLE = False      # True 면 실린더도 렌더(디버그용). 기본 False = 물리 전용(안 보임).

# ── 회피(안전 게이트) : 뷰어가 발행하는 /carterN/person_alert(Bool) 를 구독 ──
# True = "그 로봇 전방/우측 카메라에 사람이 위험 근접" → 스크립트 구동 cmd_vel 구간에서 정지·홀드.
# [19_ 변경] 18_은 c1 전용이었으나, 19_은 두 로봇 다 어떤 작업을 하든 회피가 적용돼야 하므로
# c1+c2 둘 다 실제로 사용한다(사용자 결정).
C1_PERSON_ALERT = f"/{NS_CARTER1}/person_alert"
C2_PERSON_ALERT = f"/{NS_CARTER2}/person_alert"
PERSON_GATE_ON = True                # False 면 게이트 무시(회피 로직 끔, 디버그).
# alert 가 끊겨도(뷰어 죽음 등) 무한정지 안 하도록, 마지막 True 수신 후 이 시간 지나면 자동 해제[s].
PERSON_ALERT_TIMEOUT = 1.5


# ════════════════════════════════════════════════════════════════════════════
#  D-3. ★YOLO 병합★ 우측(3시) RealSense 카메라 = 소독범위 사람 감지 센서 (18_ 이식, c1+c2 공통)
# ════════════════════════════════════════════════════════════════════════════
# 진행방향=12시(로봇 forward=chassis +X). 분사는 로봇 '오른쪽'(J1_OFFSET=-90°) = 3시 = chassis -Y.
# 그 소독범위(우측)를 보도록 카메라를 chassis 우측에 달고 뷰(-Z)를 chassis -Y 로 향하게 한다.
# 두 로봇 다 형상이 동일(같은 move_tash_can.usd)이라 오프셋/해상도/초점 값은 공통 사용.
# [2026-07-27 실측 확정] RS_ON=0 으로 끊김이 사라지는 것 실측 확인 — 카메라 cam.get_rgb() 의
# GPU→CPU 동기 읽기(렌더 완료 대기 스톨)가 프레임드랍의 원인이었다(nvidia-smi 상 GPU/CPU 평균
# 사용률은 여유로웠던 것과 모순 없음 — 스톨은 "대기"라 평균 사용률엔 잘 안 잡힘).
# 해결 = 필요 이상으로 자주 읽지 않기. PHYSICS_DT=1/60s 이고 기존 RS_PUBLISH_EVERY=8 은
# 60/8=7.5Hz 로 읽었는데, YOLO 뷰어 자체가 --rate 4(4Hz)로만 처리하므로 절반 가까이 뷰어가
# 버리는 낭비 읽기였다 — 뷰어 소비속도에 정확히 맞춰 15(=60/15=4Hz)로 낮춤(env var 로 계속 조정 가능).
#   예) RS_ON=0 isaac_python 19_...py                → 카메라 완전 비활성(끊김 완전 재현 확인용)
#       RS_PUBLISH_EVERY=30 isaac_python 19_...py    → 더 낮춰야 하면(2Hz)
RS_ON = os.environ.get("RS_ON", "1") == "1"        # False 면 카메라/발행 비활성(양쪽 공통)
RS_OFFSET = Gf.Vec3d(0.0, -0.30, 0.35)              # chassis 기준 우측(-Y)·살짝 위.
RS_RESOLUTION = (320, 240)      # ★YOLO용★ 저해상도(근접 사람 감지엔 충분, render product 부담↓)
RS_FOCAL = 14.0                                    # mm. ↓=광각(가까운 사람 잘 봄) / ↑=협각
RS_PUBLISH_EVERY = int(os.environ.get("RS_PUBLISH_EVERY", "30"))  # 메인 루프 N스텝마다 1프레임 발행
# [2026-07-27 시도·롤백] RS_PUBLISH_EVERY(15)가 RENDER_EVERY(3)의 배수라 읽기 스텝이 항상 렌더
# 트리거 스텝과 겹친다는 점에 착안, 렌더와 안 겹치게 읽기를 몇 스텝 늦추는 RS_READ_DELAY_STEPS 를
# 시도했으나 라이브 실측 결과 캐스터 보정이 오히려 더 나빠져 롤백함(가설 반증 또는 역효과가 더 큼,
# 원인 미확정) — 코드는 원래(렌더 스텝과 겹쳐도 그냥 읽음) 방식 그대로.

C2_CHASSIS = C2_ARTICULATION_ROOT   # camera 부착 기준(C1_CHASSIS 와 대칭)
C1_RS_PRIM = f"{C1_CHASSIS}/realsense_3oclock"
C2_RS_PRIM = f"{C2_CHASSIS}/realsense_3oclock"
C1_RS_TOPIC = f"/{NS_CARTER1}/realsense/color/image_raw"
C2_RS_TOPIC = f"/{NS_CARTER2}/realsense/color/image_raw"
C1_RS_FRAME_ID = "carter1_realsense"
C2_RS_FRAME_ID = "carter2_realsense"
# ── ★프레임 드랍 시 YOLO 관련 노브★ ──
#   · 전방 YOLO = 기존 hawk 재사용(새 Camera/render product 추가 금지 — GPU 부담↑)
#   · RS_PUBLISH_EVERY↑ / RS_RESOLUTION↓ (우측 RS 만 추가분, 두 로봇 공통 적용)
#   · 뷰어: --device cpu + --alternate 류(로봇/카메라 교차 추론) → CPU 부담 분산.
#   · [19_ GPU 절감] hawk 8개(4방향×L/R) + owl 4개 중 실사용은 front_hawk 뿐 — set_camera_
#     resolution() 호출 시 CAM_KEEP_FULL_RES/CAM_MINIMIZE_W/H 로 미사용 카메라를 16x16까지
#     낮춰 렌더 비용을 거의 0으로(F섹션 build_carter1/2 참고). GPU 여유 더 필요하면
#     CAM_MINIMIZE_W/H 를 더 낮추거나 RENDER_EVERY 를 3 이상으로.


# ════════════════════════════════════════════════════════════════════════════
#  E. namespace 유틸 + 수학 유틸 (13_/15_/16_ 과 완전히 동일, 변경 없음)
# ════════════════════════════════════════════════════════════════════════════
def set_carter_namespace(root_path, ns):
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    n = 0
    n_cam = 0
    for prim in Usd.PrimRange(root):
        nm = prim.GetName()
        if nm == "node_namespace":
            a = prim.GetAttribute("inputs:value")
            if a and a.IsValid():
                a.Set(ns); n += 1
        elif nm == "camera_namespace":
            a = prim.GetAttribute("inputs:value")
            if a and a.IsValid():
                cur = (a.Get() or "").strip("/")
                a.Set(f"/{ns}/{cur}" if cur else f"/{ns}")
                n_cam += 1
    print(f"[NS] {root_path} → ns='{ns}' (node_namespace {n}개, camera_namespace {n_cam}개[카메라])")
    if n == 0:
        print(f"[NS][WARN] {root_path} 하위 node_namespace 노드 못 찾음 — 토픽 접두 실패 위험")
    if n_cam == 0:
        print(f"[NS][WARN] {root_path} 하위 camera_namespace 노드 못 찾음 — 카메라 토픽 분리 실패 위험")
    return n


def set_camera_resolution(root_path, width, height, keep_full_res=(), minimize_w=None, minimize_h=None,
                          deactivate_unused=False):
    """[19_ GPU 절감] Nova Carter 내장 카메라 render product 12개(hawk 4방향×L/R=8 + owl 4) 중
    실제로 구독되는 건 front_hawk 의 left 뿐(외부 YOLO 뷰어 기본값 front_stereo_camera/left/
    image_raw) — 나머지(front_hawk 의 right, right/left/back_hawk 전부, owl 전부)는 아무도
    구독 안 하는데 계속 렌더링되고 있었다(인수인계서 15-8 GPU 병목 로그 참고).
    keep_full_res 에 담긴 문자열이 prim 경로에 포함되면(예: "front_hawk") width/height 그대로
    유지, 그 외는 minimize_w/h(지정 시)로 대폭 축소 — OmniGraph 구조는 안 건드리고 해상도만
    낮추는 저위험 접근(사용자 결정). keep_full_res 를 안 주면 기존과 동일하게 전부 width/height
    로 축소(하위호환).
    [실험적] deactivate_unused=True 면, 축소 대상(미사용) render product 노드를 해상도 조정에
    더해 prim.SetActive(False) 로 통째 비활성화한다 — 16x16 이어도 여전히 매 프레임 평가되는
    render product 자체를 꺼서 더 아낄 수 있는지 실험(CAM_DEACTIVATE_UNUSED 로 옵트인, 기본 False
    면 기존 동작과 완전히 동일). PrimRange 순회 중 SetActive 호출은 순회 자체에 영향을 줄 수 있어
    먼저 대상 목록을 다 모은 뒤에 일괄 적용한다."""
    if not width or not height:
        return 0
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    full_prims = []; min_prims = []
    for prim in Usd.PrimRange(root):
        nm = prim.GetName()
        is_cam_rp = ("camera_render_product" in nm) or (nm == "isaac_create_render_product")
        if not is_cam_rp:
            continue
        wa = prim.GetAttribute("inputs:width"); ha = prim.GetAttribute("inputs:height")
        if not (wa and wa.IsValid() and ha and ha.IsValid()):
            continue
        path_str = str(prim.GetPath()).lower()
        if keep_full_res and any(pat.lower() in path_str for pat in keep_full_res):
            full_prims.append((prim, wa, ha))
        elif minimize_w and minimize_h:
            min_prims.append((prim, wa, ha))
        else:
            full_prims.append((prim, wa, ha))
    for prim, wa, ha in full_prims:
        wa.Set(int(width)); ha.Set(int(height))
    for prim, wa, ha in min_prims:
        wa.Set(int(minimize_w)); ha.Set(int(minimize_h))
        if deactivate_unused:
            prim.SetActive(False)
    n_full = len(full_prims); n_min = len(min_prims)
    if minimize_w and minimize_h:
        tag = " (+비활성화)" if deactivate_unused else ""
        print(f"[GPU] {root_path} 카메라 render product : 유지 {n_full}개 → {width}x{height}, "
              f"미사용 축소 {n_min}개 → {minimize_w}x{minimize_h}{tag}")
    else:
        print(f"[GPU] {root_path} 카메라 render product {n_full}개 → {width}x{height} 로 축소")
    return n_full + n_min


def matrix_to_quat_wxyz(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2; w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S; y = (R[0, 2] - R[2, 0]) / S; z = (R[1, 0] - R[0, 1]) / S
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S; x = 0.25 * S; y = (R[0, 1] + R[1, 0]) / S; z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S; x = (R[0, 1] + R[1, 0]) / S; y = 0.25 * S; z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S; x = (R[0, 2] + R[2, 0]) / S; y = (R[1, 2] + R[2, 1]) / S; z = 0.25 * S
    q = np.array([w, x, y, z]); return q / np.linalg.norm(q)


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def spray_orientation_quat():
    R = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
    return matrix_to_quat_wxyz(R)


def local_to_world(base_pos, base_quat, p_local):
    return base_pos + quat_to_matrix(base_quat) @ np.asarray(p_local)


def aim_orientation_world(base_quat):
    """spray_orientation_quat() 을 "챠시 로컬 기준 조준방향"으로 재해석해 현재 base_quat 로 합성한
    진짜 월드 orientation 반환(15_/16_ 이식, 웨이포인트 도착 yaw 오차를 흡수)."""
    R_world = quat_to_matrix(base_quat) @ quat_to_matrix(spray_orientation_quat())
    return matrix_to_quat_wxyz(R_world)


def aim_link6_world_from_offset(base_pos, base_quat, ori_quat, z, tip_offset_link6_frame):
    world_offset = quat_to_matrix(ori_quat) @ np.asarray(tip_offset_link6_frame)
    return local_to_world(base_pos, base_quat, [WALL_X, AIM_Y, z]) - world_offset


def read_world_pose(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = Gf.Transform(m)
    tr = t.GetTranslation(); q = t.GetRotation().GetQuat()
    pos = np.array([tr[0], tr[1], tr[2]])
    quat = np.array([q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]])
    return pos, quat


def relative_pose(parent_path, child_path):
    p_pos, p_quat = read_world_pose(parent_path)
    c_pos, c_quat = read_world_pose(child_path)
    R_p = quat_to_matrix(p_quat)
    rel_pos = R_p.T @ (c_pos - p_pos)
    rel_R = R_p.T @ quat_to_matrix(c_quat)
    return rel_pos, rel_R


def wrap_pi(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def rotate_2d(v, theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def get_prim_world_position(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    matrix = omni.usd.get_world_transform_matrix(prim)
    t = matrix.ExtractTranslation()
    return np.array([t[0], t[1], t[2]])


def get_prim_world_bbox_center(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return (np.array(r.GetMin()) + np.array(r.GetMax())) / 2.0


def get_world_orientation_wxyz(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    q = omni.usd.get_world_transform_matrix(prim).ExtractRotationQuat()
    im = q.GetImaginary()
    return np.array([q.GetReal(), im[0], im[1], im[2]])


def rotate_vector_by_quat(q_wxyz, v):
    w = q_wxyz[0]; qv = np.asarray(q_wxyz[1:4], dtype=float); v = np.asarray(v, dtype=float)
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def get_chassis_yaw(prim_path):
    w, x, y, z = get_world_orientation_wxyz(prim_path)
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _smoothstep(a):
    return a * a * (3.0 - 2.0 * a)


def _ramp_steps_for_distance(distance, max_speed):
    raw = distance / max_speed / PHYSICS_DT
    return int(np.clip(round(raw), MIN_INTERP_STEPS, MAX_INTERP_STEPS))


def boost_drive_limits(carter_prim_path):
    stage = omni.usd.get_context().get_stage()
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(carter_prim_path)):
        ttype = prim.GetAttribute("node:type")
        if ttype and ttype.IsValid() and "DifferentialController" in str(ttype.Get() or ""):
            for name, val in (("inputs:maxAngularSpeed", DRIVE_MAX_ANG_SPEED),
                              ("inputs:maxAngularAcceleration", DRIVE_MAX_ANG_ACCEL),
                              ("inputs:maxLinearSpeed", DRIVE_MAX_LIN_SPEED)):
                a = prim.GetAttribute(name)
                if a and a.IsValid():
                    a.Set(float(val)); n += 1
    print(f"[DRIVE] {carter_prim_path} 차동구동 클램프 상향 attr {n}")


def setup_ros2_camera_publisher(camera_prim_path, topic_name, frame_id, resolution, graph_path):
    """isaacsim.ros2.bridge 의 ROS2 Camera Helper OmniGraph를 코드로 구성해
    카메라 이미지를 sensor_msgs/Image 로 백그라운드에서 발행합니다 (get_rgb 스톨 방지)."""
    import omni.graph.core as og
    keys = og.Controller.Keys
    (graph, _, _, _) = og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CameraHelperRGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "CameraHelperRGB.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "CameraHelperRGB.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("CreateRenderProduct.inputs:cameraPrim", camera_prim_path),
                ("CreateRenderProduct.inputs:width", resolution[0]),
                ("CreateRenderProduct.inputs:height", resolution[1]),
                ("CameraHelperRGB.inputs:topicName", topic_name),
                ("CameraHelperRGB.inputs:frameId", frame_id),
                ("CameraHelperRGB.inputs:type", "rgb"),
            ],
        },
    )
    print(f"[RS] OmniGraph Camera Helper 생성 완료: {graph_path} -> {topic_name}")
    return graph


def tune_arm_drives(root_path):
    stage = omni.usd.get_context().get_stage()
    n = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        for dt in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, dt)
            if drive:
                drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                drive.GetDampingAttr().Set(DRIVE_DAMPING)
                drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                drive.GetTargetPositionAttr().Set(0.0)
                n += 1
    print(f"[PHYSICS] {root_path} arm drive tuned: {n}")


class PersonGate:
    """★YOLO 병합★(18_ 이식) 사람 회피 안전 게이트. 뷰어(multi_robot_yolo_viewer.py)가
    /carterN/person_alert(Bool) 로 '전방/우측 사람 위험근접'을 알린다. blocked() 가 True 인 동안
    호출부(g_run_nav_leg 의 최종접근 cmd_vel 구간, g_spray_sweep)가 정지·홀드시킨다(Nav2 가
    조향하는 주행 leg 는 costmap 이 알아서 우회하므로 게이트 대상 아님).
    [19_ 변경] 18_은 c1 전용이었으나 두 로봇 모두에 인스턴스화한다(사용자 결정).
    · 타임아웃 : 마지막 True 수신 후 PERSON_ALERT_TIMEOUT[s] 지나면 자동 해제(뷰어 끊김 시 무한정지 방지)."""
    def __init__(self, ros_node, topic, world, label):
        self.world = world; self.label = label
        self._alert = False
        self._last_true_t = -1e9
        self._last_log = None
        ros_node.create_subscription(Bool, topic, self._cb, 10)
        print(f"[GATE] {label} person_alert 구독: {topic} (PERSON_GATE_ON={PERSON_GATE_ON})")

    def _cb(self, msg):
        self._alert = bool(msg.data)
        if self._alert:
            self._last_true_t = float(self.world.current_time)

    def blocked(self):
        if not PERSON_GATE_ON:
            return False
        b = self._alert and (float(self.world.current_time) - self._last_true_t) <= PERSON_ALERT_TIMEOUT
        if b != self._last_log:
            print(f"[GATE] {self.label} {'⛔사람 근접 → 정지·대기' if b else '✅해제 → 작업 재개'}")
            self._last_log = b
        return b


# ════════════════════════════════════════════════════════════════════════════
#  F. 씬 구성
# ════════════════════════════════════════════════════════════════════════════
def build_env():
    stage = omni.usd.get_context().get_stage()
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    world_prim.GetReferences().AddReference(HOSPITAL_USD)
    for _ in range(60):
        simulation_app.update()
    print(f"[LOAD] hospital env = {HOSPITAL_USD}")

    # ★조명 기본값★ : hospital 자체 조명이 약해 시작 시 씬이 어둡다 → DomeLight 로 기본 조명 보강.
    #   실제 씬 라이트라 뷰포트 "Lighting Mode" 설정과 무관하게 항상 밝게 나온다(15_ 선례 동일 패턴).
    light_path = "/World/DefaultDomeLight"
    if not stage.GetPrimAtPath(light_path).IsValid():
        dome = UsdLux.DomeLight.Define(stage, light_path)
        dome.CreateIntensityAttr(500.0)
        print(f"[LIGHT] 기본 조명 보강: {light_path} (intensity 500)")


def _place_xform(prim_path, x, y, z, yaw_deg):
    stage = omni.usd.get_context().get_stage()
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    xf.ClearXformOpOrder()
    m = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), float(yaw_deg)))
    m.SetTranslateOnly(Gf.Vec3d(x, y, z))
    xf.AddTransformOp().Set(m)


def build_carter1():
    """carter1 = move_tash_can.usd(Nova+Surface Gripper 팔+쓰레기통) 전체를 /World/Carter1 로
    참조. 쓰레기통은 공용 인스턴스가 아니므로 비활성화, Nova 를 carter1 홈(18.5,0)으로 재배치."""
    stage = omni.usd.get_context().get_stage()
    scope = stage.DefinePrim(C1_SCOPE, "Xform")
    UsdGeom.Xformable(scope).ClearXformOpOrder()
    scope.GetReferences().AddReference(Sdf.Reference(assetPath=MOVE_TRASH_USD, primPath="/World"))
    for _ in range(80):
        simulation_app.update()

    if not stage.GetPrimAtPath(C1_ARTICULATION_ROOT).IsValid():
        print(f"[FATAL] {C1_ARTICULATION_ROOT} 없음 — carter1 로드 실패"); return False

    extra = stage.GetPrimAtPath(C1_EXTRA_PHYSICS)
    if extra.IsValid():
        extra.SetActive(False)
        print(f"[SCENE] {C1_EXTRA_PHYSICS} 비활성화(중복 PhysicsScene)")

    trash = stage.GetPrimAtPath(C1_TRASH_CAN_PRIM)
    if trash.IsValid():
        trash.SetActive(False)
        print(f"[SCENE] {C1_TRASH_CAN_PRIM} 비활성화(쓰레기통은 carter2쪽 1개만 공용 사용)")

    _place_xform(C1_CARTER_PRIM, C1_START_POSE["x"], C1_START_POSE["y"],
                 C1_START_POSE["z"], C1_START_POSE["yaw_deg"])
    simulation_app.update()

    chassis = stage.GetPrimAtPath(C1_ARTICULATION_ROOT)
    chassis_m = UsdGeom.Xformable(chassis).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    _wp = Gf.Transform(chassis_m).GetTranslation()
    print(f"[SPAWN] c1 chassis world = ({_wp[0]:.3f}, {_wp[1]:.3f}, {_wp[2]:.3f}) "
          f"(목표 {C1_START_POSE['x']:.2f},{C1_START_POSE['y']:.2f})")

    arm1 = stage.GetPrimAtPath(C1_ARM_ROOT)
    if arm1.IsValid():
        arm1_m = Gf.Matrix4d().SetTranslate(MOUNT_OFFSET) * chassis_m
        xf1 = UsdGeom.Xformable(arm1)
        xf1.ClearXformOpOrder()
        xf1.AddTransformOp().Set(arm1_m)
        _awp = Gf.Transform(UsdGeom.Xformable(stage.GetPrimAtPath(f"{C1_ARM_ROOT}/base_link"))
                            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())).GetTranslation()
        print(f"[SCENE] c1 팔 사전배치(Play 전 가시화) → base_link ({_awp[0]:.2f},{_awp[1]:.2f},{_awp[2]:.2f})")
    else:
        print(f"[SCENE][WARN] {C1_ARM_ROOT} 없음 — 팔 사전배치 스킵")

    set_carter_namespace(C1_CARTER_PRIM, NS_CARTER1)
    set_camera_resolution(C1_CARTER_PRIM, CAM_RENDER_W, CAM_RENDER_H,
                          keep_full_res=CAM_KEEP_FULL_RES, minimize_w=CAM_MINIMIZE_W, minimize_h=CAM_MINIMIZE_H,
                          deactivate_unused=CAM_DEACTIVATE_UNUSED)
    print("[SCENE] carter1 (Nova + Surface Gripper 팔, 쓰레기통 비활성) 완료")
    return True


def build_carter2():
    """carter2 = move_tash_can.usd 전체를 /World/Carter2 로 참조. 이쪽 쓰레기통(TRASH_CAN_PRIM)이
    ★두 로봇 공용 유일 인스턴스★ — 비활성화하지 않는다."""
    stage = omni.usd.get_context().get_stage()
    scope = stage.DefinePrim(C2_SCOPE, "Xform")
    UsdGeom.Xformable(scope).ClearXformOpOrder()
    scope.GetReferences().AddReference(Sdf.Reference(assetPath=MOVE_TRASH_USD, primPath="/World"))
    for _ in range(80):
        simulation_app.update()

    if not stage.GetPrimAtPath(C2_ARTICULATION_ROOT).IsValid():
        print(f"[FATAL] {C2_ARTICULATION_ROOT} 없음 — carter2 로드 실패"); return False

    extra = stage.GetPrimAtPath(C2_EXTRA_PHYSICS)
    if extra.IsValid():
        extra.SetActive(False)
        print(f"[SCENE] {C2_EXTRA_PHYSICS} 비활성화(중복 PhysicsScene)")

    trash = stage.GetPrimAtPath(TRASH_CAN_PRIM)
    if trash.IsValid():
        ta = trash.GetAttribute("xformOp:translate")
        if ta and ta.IsValid():
            _origin = np.asarray(TRASH_SPAWN_FIXED) - TRASH_BBOX_CENTER_OFFSET_XY
            ta.Set(Gf.Vec3d(float(_origin[0]), float(_origin[1]), TRASH_SPAWN_Z))
            print(f"[SPAWN] 공용 trash 중심 → {TRASH_SPAWN_FIXED} (원점 {_origin.tolist()})")

    # [2026-07-26] build_carter1() 과 달리 여기엔 _place_xform 호출이 원래 없었음 — carter2 는 참조된
    # USD 에셋의 기본 자세(yaw=0/+X)를 그대로 써왔고, C2_START_POSE["yaw_deg"] 는 값만 있고 실제로
    # 적용된 적이 없었다(우연히 기본값이 0이라 안 드러남). +Y 스폰 방향으로 바꾸며 발견 — carter1 과
    # 동일하게 명시적으로 배치한다.
    _place_xform(f"{C2_SCOPE}/Nova_Carter_ROS", C2_START_POSE["x"], C2_START_POSE["y"],
                 C2_START_POSE["z"], C2_START_POSE["yaw_deg"])
    simulation_app.update()

    chassis2 = stage.GetPrimAtPath(C2_ARTICULATION_ROOT)
    chassis2_m = UsdGeom.Xformable(chassis2).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    _wp = Gf.Transform(chassis2_m).GetTranslation()
    print(f"[SPAWN] c2 chassis world = ({_wp[0]:.3f}, {_wp[1]:.3f}, {_wp[2]:.3f}) "
          f"(목표 {C2_START_POSE['x']:.2f},{C2_START_POSE['y']:.2f})")

    arm2 = stage.GetPrimAtPath(C2_ARM_ROOT)
    if arm2.IsValid():
        arm2_m = Gf.Matrix4d().SetTranslate(MOUNT_OFFSET) * chassis2_m
        xf2 = UsdGeom.Xformable(arm2)
        xf2.ClearXformOpOrder()
        xf2.AddTransformOp().Set(arm2_m)
        _awp = Gf.Transform(UsdGeom.Xformable(stage.GetPrimAtPath(f"{C2_ARM_ROOT}/base_link"))
                            .ComputeLocalToWorldTransform(Usd.TimeCode.Default())).GetTranslation()
        print(f"[SCENE] c2 팔 사전배치(Play 전 가시화) → base_link ({_awp[0]:.2f},{_awp[1]:.2f},{_awp[2]:.2f})")
    else:
        print(f"[SCENE][WARN] {C2_ARM_ROOT} 없음 — 팔 사전배치 스킵")

    set_carter_namespace(C2_SCOPE_PRIM, NS_CARTER2)
    set_camera_resolution(C2_SCOPE_PRIM, CAM_RENDER_W, CAM_RENDER_H,
                          keep_full_res=CAM_KEEP_FULL_RES, minimize_w=CAM_MINIMIZE_W, minimize_h=CAM_MINIMIZE_H,
                          deactivate_unused=CAM_DEACTIVATE_UNUSED)
    print("[SCENE] carter2 (Nova + Surface Gripper 팔 + 공용 쓰레기통) 완료")
    return True


def build_people():
    """★YOLO 병합★(18_ 이식, 로직 무변경) PEOPLE 목록대로 사람을 배치한다.
    · people.usd 를 통째 참조하면 Plane/Environment/Light/Render 까지 딸려와 병원 씬을 오염 →
      '캐릭터 프림만' primPath 타겟으로 개별 참조.
    · 캐릭터 메시는 collision 이 없다 :
        - 렌더 메시 → Nova Carter RTX front_3d_lidar 가 장애물로 인식(→ /scan → costmap → Nav2 우회)
                       + 우측 RealSense 카메라(YOLO)가 'person' 으로 검출.
        - 그 위에 '보이지 않는' static collision 실린더를 겹쳐 물리적 관통 방지(안전)."""
    if not Path(PEOPLE_USD).is_file():
        print(f"[PEOPLE][WARN] {PEOPLE_USD} 없음 — 사람 배치 스킵"); return
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, PEOPLE_SCOPE)
    n = 0
    for i, (name, x, y, yaw_deg) in enumerate(PEOPLE):
        char_src = PEOPLE_CHARACTER_PRIMS[i % len(PEOPLE_CHARACTER_PRIMS)]
        person_path = f"{PEOPLE_SCOPE}/{name}"
        prim = stage.DefinePrim(person_path, "Xform")
        prim.GetReferences().AddReference(Sdf.Reference(assetPath=PEOPLE_USD, primPath=char_src))
        _place_xform(person_path, x, y, 0.0, yaw_deg)
        col_path = f"{person_path}/safety_collider"
        cyl = UsdGeom.Cylinder.Define(stage, col_path)
        cyl.CreateRadiusAttr(float(PERSON_COLLIDER_RADIUS))
        cyl.CreateHeightAttr(float(PERSON_COLLIDER_HEIGHT))
        cyl.CreateAxisAttr(UsdGeom.Tokens.z)
        UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, PERSON_COLLIDER_HEIGHT / 2.0))
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        if not PERSON_COLLIDER_VISIBLE:
            UsdGeom.Imageable(cyl.GetPrim()).MakeInvisible()
        n += 1
        print(f"[PEOPLE] {name} @ ({x},{y},yaw{yaw_deg}) src={char_src}")
    for _ in range(60):
        simulation_app.update()
    print(f"[PEOPLE] 사람 {n}명 배치 완료 (렌더메시→라이다/YOLO, 보이지않는 실린더→물리충돌)")


def build_realsense(chassis_path, cam_prim_path, offset, focal, label):
    """★YOLO 병합★(18_ build_c1_realsense() 일반화) chassis_path/cam_prim_path/offset 을 인자로
    받아 두 로봇에 동일하게 호출한다. carter 의 3시(오른쪽=분사 방향)에 컬러 카메라를 장착 —
    카메라 뷰(local -Z)를 chassis -Y(로봇 우측=소독범위)로 향하게 배치해 그쪽 사람을 감지한다.
    반환: 카메라 prim 경로(str) or None."""
    if not RS_ON:
        print(f"[RS][{label}] RS_ON=False → 3시 카메라 비활성")
        return None
    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(chassis_path).IsValid():
        print(f"[RS][{label}][WARN] chassis({chassis_path}) 없음 — RealSense 스킵"); return None
    cam = UsdGeom.Camera.Define(stage, cam_prim_path)
    R = np.array([[-1.0, 0.0, 0.0],
                  [ 0.0, 0.0, 1.0],
                  [ 0.0, 1.0, 0.0]])
    q = matrix_to_quat_wxyz(R)
    m = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Quatd(float(q[0]), float(q[1]), float(q[2]), float(q[3]))))
    m.SetTranslateOnly(offset)
    xf = UsdGeom.Xformable(cam.GetPrim()); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(m)
    cam.CreateFocalLengthAttr(float(focal))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
    print(f"[RS][{label}] 3시 카메라 배치 {cam_prim_path} (offset {tuple(offset)}, 우측(-Y) 향, f={focal}mm)")
    return cam_prim_path


def rgb_to_ros_image(rgb, stamp=None, frame_id="carter_realsense"):
    """HxWx3 uint8 RGB → sensor_msgs/Image(rgb8). 뷰어가 구독해 YOLO.
    [19_ 변경] frame_id 를 인자화(18_은 C1_RS_FRAME_ID 기본값 하드코딩) — 호출부가 로봇별로 전달."""
    msg = RosImage()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = int(rgb.shape[0]), int(rgb.shape[1])
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()
    return msg


_RS_CUDA_GET_RGB_OK = {}      # 카메라 label별 device="cuda" 지원 여부 캐시(1회 확인 후 재사용)
_RS_TIME_DEBUG = os.environ.get("RS_TIME_DEBUG", "0") == "1"


def publish_realsense_frame(cam, pub, frame_id, t):
    """★YOLO 병합★ 18_ 메인루프 인라인 로직을 헬퍼화(c1/c2 재사용, 중복 제거). cam/pub 이 None
    이면(카메라 비활성/발행자 미준비) 무해하게 통과.
    [2026-07-27 실험] get_rgb() 는 device 인자로 'cuda' 를 받으면 GPU→CPU 복사 없이 GPU 상주
    (Warp) 배열을 반환할 수 있다(Camera.get_rgb 시그니처 확인) — ROS2 로 내보내려면 결국 CPU
    numpy 가 필요해서 최종적으로 .numpy() 호출은 하지만, 이 경로가 device 미지정 기본값보다
    스톨을 줄이는지 확인하기 위한 실험. 실패하면 device 미지정으로 자동 폴백(카메라별 1회만
    시도 후 결과 캐시 — 매 프레임 재시도 안 함). RS_TIME_DEBUG=1 이면 get_rgb() 소요시간 로그."""
    if cam is None or pub is None:
        return
    try:
        use_cuda = _RS_CUDA_GET_RGB_OK.get(frame_id, True)
        t0 = time.perf_counter() if _RS_TIME_DEBUG else None
        if use_cuda:
            try:
                rgb = cam.get_rgb(device="cuda")
                if hasattr(rgb, "numpy"):
                    rgb = rgb.numpy()
                _RS_CUDA_GET_RGB_OK[frame_id] = True
            except Exception as e:
                print(f"[RS][{frame_id}][WARN] get_rgb(device='cuda') 실패({e}) — 기본 device 로 폴백")
                _RS_CUDA_GET_RGB_OK[frame_id] = False
                rgb = cam.get_rgb()
        else:
            rgb = cam.get_rgb()
        if _RS_TIME_DEBUG:
            dt_ms = (time.perf_counter() - t0) * 1000.0
            print(f"[RS][{frame_id}][TIME] get_rgb(cuda={_RS_CUDA_GET_RGB_OK.get(frame_id)}) "
                  f"{dt_ms:.2f}ms")
        if rgb is None:
            return
        arr = np.asarray(rgb)
        if arr.ndim == 3 and arr.shape[2] >= 3 and arr.size:
            arr = np.ascontiguousarray(arr[:, :, :3])
            if arr.dtype != np.uint8:
                arr = np.clip(arr, 0, 255).astype(np.uint8)
            img = rgb_to_ros_image(arr, frame_id=frame_id)
            img.header.stamp.sec = int(t)
            img.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
            pub.publish(img)
    except Exception:
        pass


def build_nozzle_dock(scope, tool_path, hold_joint_path, hold_anchor_path, dock_xy, dock_height, label):
    """[15_/16_ 일반화] 로봇별 노즐 거치대 하나를 세운다 — 인자로 경로/좌표를 받아 로봇별로 두 번
    호출한다. "수직 매달기"(TC_APPROACH_ORIENTATION=(0,1,0,0) → 로컬 Z가 world -Z)로 세우고, 임시
    hold_joint(정적 anchor prim에 FixedJoint)로 고정 — 파지 성공 직후 release_hold_joint() 로 해제."""
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, scope)
    tool_prim = stage.DefinePrim(tool_path, "Xform")
    tool_prim.GetReferences().AddReference(
        Sdf.Reference(assetPath=NOZZLE_USD, primPath=NOZZLE_SOURCE_PRIMPATH))
    for _ in range(20):
        simulation_app.update()
    if not stage.GetPrimAtPath(tool_path).IsValid():
        print(f"[FATAL] {tool_path} 로드 실패"); return False

    dock_quat = TC_APPROACH_ORIENTATION
    rot = Gf.Rotation(Gf.Quatd(float(dock_quat[0]), Gf.Vec3d(*dock_quat[1:])))
    m = Gf.Matrix4d().SetRotate(rot).SetTranslateOnly(
        Gf.Vec3d(float(dock_xy[0]), float(dock_xy[1]), float(dock_height)))
    xf = UsdGeom.Xformable(tool_prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)
    print(f"[SPAWN] 노즐 거치대[{label}] = ({dock_xy[0]:.3f},{dock_xy[1]:.3f},{dock_height:.3f}) 수직 매달기")

    anchor_prim = stage.DefinePrim(hold_anchor_path, "Xform")
    anchor_xf = UsdGeom.Xformable(anchor_prim)
    anchor_xf.ClearXformOpOrder()
    anchor_xf.AddTransformOp().Set(m)

    hold_joint = UsdPhysics.FixedJoint.Define(stage, hold_joint_path)
    hold_joint.CreateBody0Rel().SetTargets([hold_anchor_path])
    hold_joint.CreateBody1Rel().SetTargets([tool_path])
    hold_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    hold_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    hold_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    hold_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    hold_joint.CreateExcludeFromArticulationAttr().Set(True)
    hold_joint.CreateJointEnabledAttr().Set(True)
    print(f"[SCENE] 임시 거치 조인트[{label}] authoring = {hold_joint_path} (anchor={hold_anchor_path})")

    # [2026-07-26] 실제 거치대 물리 형상(기둥+받침+캔틸레버 암) — 18_ build_people()의
    # UsdGeom.Cylinder+CollisionAPI 정적 충돌체 패턴 재사용. 기둥은 손잡이(dock_xy)에서 남쪽(-Y)으로
    # NOZZLE_STAND_POLE_OFFSET 만큼 띄워 세운다(로봇 파킹 구역과 기하학적으로 안 겹치도록).
    pole_xy = np.array([dock_xy[0], dock_xy[1] - NOZZLE_STAND_POLE_OFFSET])

    base = UsdGeom.Cylinder.Define(stage, f"{scope}/stand_base")
    base.CreateRadiusAttr(float(NOZZLE_STAND_BASE_RADIUS))
    base.CreateHeightAttr(float(NOZZLE_STAND_BASE_HEIGHT))
    base.CreateAxisAttr(UsdGeom.Tokens.z)
    UsdGeom.Xformable(base).AddTranslateOp().Set(
        Gf.Vec3d(float(pole_xy[0]), float(pole_xy[1]), NOZZLE_STAND_BASE_HEIGHT / 2.0))

    pole = UsdGeom.Cylinder.Define(stage, f"{scope}/stand_pole")
    pole.CreateRadiusAttr(float(NOZZLE_STAND_POLE_RADIUS))
    pole.CreateHeightAttr(float(dock_height))
    pole.CreateAxisAttr(UsdGeom.Tokens.z)
    UsdGeom.Xformable(pole).AddTranslateOp().Set(
        Gf.Vec3d(float(pole_xy[0]), float(pole_xy[1]), dock_height / 2.0))

    arm_mid_xy = (pole_xy + np.asarray(dock_xy)) / 2.0
    arm = UsdGeom.Cylinder.Define(stage, f"{scope}/stand_arm")
    arm.CreateRadiusAttr(float(NOZZLE_STAND_ARM_RADIUS))
    arm.CreateHeightAttr(float(NOZZLE_STAND_POLE_OFFSET))
    arm.CreateAxisAttr(UsdGeom.Tokens.y)   # 오프셋이 순수 -Y 방향이라 회전 없이 Y축 실린더로 정렬됨
    UsdGeom.Xformable(arm).AddTranslateOp().Set(
        Gf.Vec3d(float(arm_mid_xy[0]), float(arm_mid_xy[1]), float(dock_height)))

    if NOZZLE_STAND_COLLISION_ENABLED:
        for geom in (base, pole, arm):
            UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
    print(f"[SPAWN] 노즐 거치대 형상[{label}] 기둥=({pole_xy[0]:.3f},{pole_xy[1]:.3f}) "
          f"충돌={'ON' if NOZZLE_STAND_COLLISION_ENABLED else 'OFF(시각only)'}")
    return True


def release_hold_joint(stage, hold_joint_path):
    prim = stage.GetPrimAtPath(hold_joint_path)
    UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(False)
    print(f"[INFO] 임시 거치 조인트 비활성화 = {hold_joint_path}")


def engage_hold_joint(stage, hold_joint_path):
    prim = stage.GetPrimAtPath(hold_joint_path)
    UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(True)
    print(f"[INFO] 임시 거치 조인트 재활성화 = {hold_joint_path}")


def setup_nozzle_surface_gripper(stage, arm_root, ee_link_name, surface_gripper_path, label):
    """[15_/16_ 일반화] 로봇 그리퍼(move_tash_can 내장 mop_surface_gripper)의 기존 D6 조인트 속성만
    검증값(CLEARANCE_OFFSET/GRIP_TRAVEL 등)으로 재기록(새로 만들면 attachmentPoints 관계 깨질 위험)."""
    joint_path = f"{arm_root}/{ee_link_name}/mop_surface_gripper_joints/mop_attachment_joint"
    gripper_prim = stage.GetPrimAtPath(surface_gripper_path)
    joint_prim = stage.GetPrimAtPath(joint_path)
    if gripper_prim.IsValid() and joint_prim.IsValid():
        gripper_prim.GetAttribute("isaac:maxGripDistance").Set(float(MAX_GRIP_DISTANCE))
        joint_prim.GetAttribute("isaac:clearanceOffset").Set(float(CLEARANCE_OFFSET))
        joint_prim.GetAttribute("limit:transZ:physics:high").Set(float(GRIP_TRAVEL))
        joint_prim.GetAttribute("drive:transZ:physics:stiffness").Set(float(GRIP_DRIVE_STIFFNESS))
        joint_prim.GetAttribute("drive:transZ:physics:damping").Set(float(GRIP_DRIVE_DAMPING))
        print(f"[GRIPPER][{label}] {surface_gripper_path} 기존 조인트 재튜닝 "
              f"(clearance={CLEARANCE_OFFSET}, travel={GRIP_TRAVEL}, maxGripDistance={MAX_GRIP_DISTANCE})")
        return surface_gripper_path
    print(f"[GRIPPER][{label}][WARN] {surface_gripper_path} 또는 {joint_path} 없음 — 새로 authoring")
    return surface_gripper_utils.setup_mop_surface_gripper(
        stage, fingertip_prim_path=f"{arm_root}/{ee_link_name}",
        gripper_prim_path=surface_gripper_path,
        max_grip_distance=MAX_GRIP_DISTANCE, grip_drive_stiffness=GRIP_DRIVE_STIFFNESS,
        grip_drive_damping=GRIP_DRIVE_DAMPING, clearance_offset=CLEARANCE_OFFSET, grip_travel=GRIP_TRAVEL)


def build_robot_manipulator(my_world, arm_root, articulation_root, ee_link_name, label):
    """carter1/carter2 공용 SingleManipulator 등록(world.reset 전 호출) → (robot, ee_path, tool0_path)."""
    stage = omni.usd.get_context().get_stage()
    tune_arm_drives(arm_root)

    def find_by_name(root, name):
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if prim.GetName() == name:
                return str(prim.GetPath())
        return None

    ee_path = find_by_name(arm_root, ee_link_name)
    tool0_path = find_by_name(arm_root, "tool0")
    if ee_path is None or tool0_path is None:
        raise RuntimeError(f"{label} link_6/tool0 못 찾음 (ee={ee_path}, tool0={tool0_path})")

    robot = my_world.scene.add(SingleManipulator(
        prim_path=articulation_root, name=f"{label}_m0609",
        end_effector_prim_path=ee_path, gripper=None))
    return robot, ee_path, tool0_path


def build_robot_rmpflow_gripper_toolchanger(stage, arm_root, ground_plane, ee_path,
                                            surface_gripper_path, dock_xy, dock_height,
                                            tool_path, robot_articulation, label):
    """[15_ build_common_control 일반화] RMPflow(트래시 Cartesian 이동용) + Surface Gripper +
    ToolChangerController — 두 로봇 다 이제 트래시/분사를 둘 다 하므로 둘 다 전부 갖춘다
    (16_에서 carter1은 RMPflow가 없었으나, 여기선 필요)."""
    rmpflow = RMPFlowController(name=f"{label}_cspace", robot_articulation=robot_articulation,
                                urdf_path=NO_GRIPPER_URDF)
    sync_base_prim = stage.GetPrimAtPath(f"{arm_root}/base_link")
    _m = omni.usd.get_world_transform_matrix(sync_base_prim)
    _t = _m.ExtractTranslation(); _q = _m.ExtractRotationQuat(); _im = _q.GetImaginary()
    rmpflow.rmp_flow.set_robot_base_pose(
        robot_position=np.array([_t[0], _t[1], _t[2]]),
        robot_orientation=np.array([_q.GetReal(), _im[0], _im[1], _im[2]]))
    try:
        rmpflow.rmp_flow.add_ground_plane(prim_path=ground_plane, z_position=0.0)
        print(f"[RMP][{label}] ground plane 등록: {ground_plane}")
    except TypeError:
        try:
            rmpflow.rmp_flow.add_ground_plane()
            print(f"[RMP][{label}] ground plane 등록(무인자)")
        except Exception as e:
            print(f"[RMP][{label}][WARN] ground plane 등록 실패({e}) → 스킵")
    except Exception as e:
        print(f"[RMP][{label}][WARN] ground plane 등록 실패({e}) → 스킵")

    gripper = SurfaceGripper(end_effector_prim_path=ee_path, surface_gripper_path=surface_gripper_path)
    gripper.initialize()

    dock_quat = TC_APPROACH_ORIENTATION
    tool_changer = ToolChangerController(
        rg2_fingertip_prim_path=ee_path,
        mop_handle_prim_path=tool_path,
        stand_position=np.array([dock_xy[0], dock_xy[1], dock_height]),
        stand_orientation=dock_quat,
        approach_orientation=dock_quat,
        fingertip_offset_from_ik_frame=TC_FINGERTIP_OFFSET_FROM_TOOL0,
        rg2_gripper=None,
        surface_gripper_prim_path=surface_gripper_path,
        auto_create_surface_gripper=False,
    )
    tool_changer.initialize()
    return rmpflow, gripper, tool_changer


# ════════════════════════════════════════════════════════════════════════════
#  G. RobotCtx — 로봇 비종속 제너레이터가 공유하는 핸들 묶음
# ════════════════════════════════════════════════════════════════════════════
class RobotCtx:
    """15_ C2Ctx + 16_ C1Ctx 를 하나로 통합. 두 로봇 다 트래시/분사를 둘 다 할 수 있어야 하므로
    RMPflow(트래시 Cartesian)와 ToolChangerController+Joint IK(툴체인지/분사조준)를 모두 갖는다.
    dock/spray 좌표를 필드로 들고 있어 아래 제너레이터들은 모듈 전역 C1_*/C2_* 를 참조하지 않고
    전부 ctx.* 만 참조한다(단일 카피로 두 로봇 다 구동)."""
    def __init__(self, name, world, robot, rmpflow, dof_names, tool0_path, ee_path, gripper,
                 arm_root, articulation_root, ros_node, nav_goal_pub, cmd_pub, pick_state,
                 task_select_state, dock_xy, dock_height, dock_approach_xy, dock_approach_yaw,
                 tool_path, tcp_path, hold_joint_path, hold_anchor_path,
                 spray_wp_xy, spray_wp_yaw, home_xy, home_yaw, dock_park_yaw):
        self.name = name
        self.world = world; self.robot = robot; self.rmpflow = rmpflow
        self.dof_names = dof_names; self.tool0_path = tool0_path; self.ee_path = ee_path
        self.gripper = gripper
        self.arm_root = arm_root; self.articulation_root = articulation_root
        self.ros_node = ros_node; self.nav_goal_pub = nav_goal_pub; self.cmd_pub = cmd_pub
        self.pick_state = pick_state; self.task_select_state = task_select_state
        self.dock_xy = dock_xy; self.dock_height = dock_height
        self.dock_approach_xy = dock_approach_xy; self.dock_approach_yaw = dock_approach_yaw
        self.dock_park_yaw = dock_park_yaw   # [2026-07-26] 후진 진입 최종 파킹 방향(=dock_approach_yaw+pi)
        self.tool_path = tool_path; self.tcp_path = tcp_path
        self.hold_joint_path = hold_joint_path; self.hold_anchor_path = hold_anchor_path
        self.spray_wp_xy = spray_wp_xy; self.spray_wp_yaw = spray_wp_yaw
        self.home_xy = home_xy; self.home_yaw = home_yaw
        self.stage = omni.usd.get_context().get_stage()
        self.status = "시작 대기"
        self.tool_changer = None            # ToolChangerController, main() 이 주입
        self.holding_nozzle = False
        self.holding_trash = False          # [HMI v2] g_trash_mission 파지~반납 사이 True — 도킹복귀
                                             # 시 쓰레기통 원위치 반납 판단에 사용(g_return_dock_and_reset)
        self.nozzle_tip_offset = None       # link_6 기준 nozzle_tcp 상대위치(3축) — 파지 직후 실측
        self.spray_fx = None                # [HMI v2] g_spray_sweep 이 생성한 SprayFX 참조 — 수동제어/
                                             # 도킹복귀/긴급정지로 스윕 제너레이터가 중간에 버려져도
                                             # (return 경로를 못 타 자체 clear() 가 안 불림) 밖에서 정리 가능하게
        # [19_ 병합] __init__ 시그니처는 안 건드리고(사용자 결정) 기본값만 두고, main() 이 RobotCtx
        # 생성 직후 tool_changer 와 동일한 패턴으로 별도 대입한다.
        self.person_gate = None    # PersonGate 인스턴스(YOLO 사람회피) — main() 이 생성 후 주입
        self.estop_flags = None    # {"carter1":bool,"carter2":bool} dict 참조 — main() 이 생성 후 주입
        # [HMI v2 병합] /carterN/process_state 직접 발행용. main() 이 퍼블리셔 생성 후 주입
        # (person_gate/estop_flags 와 동일한 "생성 후 별도 대입" 패턴).
        self.hmi_state_pub = None  # ros_node.create_publisher(String, f"/{name}/process_state", 10)
        self._last_hmi_label = None


def publish_hmi_state(ctx, label):
    """[HMI v2 병합] hmi_link.py 의 HmiLink.publish_state() 와 동일한 와이어 포맷으로
    /carterN/process_state 를 직접 발행한다(19_ 은 이미 /robot/command 를 직접 구독하는
    패턴이 있으므로, 같은 방식으로 상태도 직접 발행 — trash_can_nav_pick_mission.py 의 구식
    HMI 발행(hmi_enable:=False 로 꺼둠, 트래시 전용 고정 라벨이라 task_select 모델에 안 맞음)을
    대체한다). label="대기" 면 payload "대기", 그 외엔 "RUNNING:{label} 중". 동일 라벨 연속
    발행은 dedup 해 브로드캐스트 노이즈를 줄인다(ctx.status 콘솔로그와는 별개 채널)."""
    if ctx.hmi_state_pub is None or label == ctx._last_hmi_label:
        return
    ctx._last_hmi_label = label
    data = "대기" if label == "대기" else f"RUNNING:{label} 중"
    ctx.hmi_state_pub.publish(String(data=data))


def sync_rmpflow_base_pose(ctx):
    base_prim = ctx.stage.GetPrimAtPath(f"{ctx.arm_root}/base_link")
    m = omni.usd.get_world_transform_matrix(base_prim)
    tr = m.ExtractTranslation(); q = m.ExtractRotationQuat(); im = q.GetImaginary()
    pos = np.array([tr[0], tr[1], tr[2]])
    ori = np.array([q.GetReal(), im[0], im[1], im[2]])
    ctx.rmpflow.rmp_flow.set_robot_base_pose(robot_position=pos, robot_orientation=ori)


def _estop_hold(ctx):
    """★긴급정지 시 팔 즉시 정지★ : estop 동안 새 관절/EE 목표를 적용하지 않고(=마지막 명령 자세를
    articulation PD 가 그대로 유지) 스텝 진행을 멈춘 채 대기한다. 해제되면 반환 → 호출한 팔 램프가
    멈췄던 자리에서 이어서 진행(램프 step 을 소비하지 않으므로 재개 시 자세 점프 없음).
    ctx.estop_flags 가 None(HMI 미연결)이면 즉시 통과."""
    while ctx.estop_flags is not None and ctx.estop_flags.get(ctx.name, False):
        yield


def g_ramp_to_joint_positions(ctx, target_joints_deg, ramp_steps):
    start = ctx.robot.get_joint_positions().copy()
    target = start.copy()
    arm_idx = [ctx.dof_names.index(n) for n in ARM_JOINT_NAMES if n in ctx.dof_names]
    for idx, rad in zip(arm_idx, np.radians(target_joints_deg)):
        target[idx] = rad
    for step in range(ramp_steps):
        yield
        yield from _estop_hold(ctx)          # ★긴급정지 시 팔 정지·대기★
        alpha = (step + 1) / ramp_steps
        wp = start + _smoothstep(alpha) * (target - start)
        ctx.robot.apply_action(ArticulationAction(joint_positions=wp))


def g_ramp_ee_target(ctx, target_position, target_orientation, ramp_steps):
    start = get_prim_world_position(ctx.tool0_path)
    for step in range(ramp_steps):
        yield
        yield from _estop_hold(ctx)          # ★긴급정지 시 팔 정지·대기★
        alpha = (step + 1) / ramp_steps
        wp = start + _smoothstep(alpha) * (target_position - start)
        ctx.robot.apply_action(ctx.rmpflow.forward(
            target_end_effector_position=wp, target_end_effector_orientation=target_orientation))


def g_move_to_pose(ctx, target_position, target_orientation, label, max_linear_speed=MAX_EE_LINEAR_SPEED,
                   growing_tolerance_max=None, growing_tolerance_tau=None, position_tolerance=None,
                   max_approach_steps=None):
    start = get_prim_world_position(ctx.tool0_path)
    distance = float(np.linalg.norm(target_position - start))
    ramp_steps = _ramp_steps_for_distance(distance, max_linear_speed)
    print(f"[INFO][{ctx.name}] {label} 이동 (dist={distance:.3f}m)")
    yield from g_ramp_ee_target(ctx, target_position, target_orientation, ramp_steps)
    steps_budget = max_approach_steps if max_approach_steps is not None else MAX_APPROACH_STEPS
    tau = growing_tolerance_tau if growing_tolerance_tau else steps_budget / 4.0
    base_tol = position_tolerance if position_tolerance is not None else POSITION_TOLERANCE
    for step in range(steps_budget):
        yield
        yield from _estop_hold(ctx)          # ★긴급정지 시 팔 정지·대기★
        ee = get_prim_world_position(ctx.tool0_path)
        ctx.robot.apply_action(ctx.rmpflow.forward(
            target_end_effector_position=target_position, target_end_effector_orientation=target_orientation))
        if growing_tolerance_max is not None:
            cur_tol = growing_tolerance_max * (1.0 - np.exp(-step / tau))
        else:
            cur_tol = base_tol
        if float(np.linalg.norm(ee - target_position)) < cur_tol:
            print(f"[INFO][{ctx.name}] {label} 도달 (step={step}, tol={cur_tol:.3f}m)"); return
    final_dist = float(np.linalg.norm(get_prim_world_position(ctx.tool0_path) - target_position))
    print(f"[WARN][{ctx.name}] {label} {steps_budget} step 내 미수렴 (잔여오차={final_dist*1000:.1f}mm)")


def g_hold_pose(ctx, target_position, target_orientation, steps):
    for _ in range(steps):
        yield
        ctx.robot.apply_action(ctx.rmpflow.forward(
            target_end_effector_position=target_position, target_end_effector_orientation=target_orientation))


def g_rotate_in_place(ctx, target_yaw, kp, kd, w_max, max_w_step, tol, chassis_path, max_steps=600,
                      force_direction=None):
    """[19_ 병합] Nav2 가 관여 안 하는 순수 스크립트 주행 구간이라, 사람감지(YOLO)/긴급정지(HMI) 를
    여기서 직접 게이팅한다 — 둘 다 '제자리 정지'로 동일 처리(전진 없이 이 스텝만 건너뜀, 상태는
    그대로 유지해 재개 시 이어서 진행). ctx.person_gate/ctx.estop_flags 가 None 이면(카메라
    비활성/HMI 미연결) 무해하게 통과.
    [2026-07-27] force_direction="cw"/"ccw" — 기본(None)은 wrap_pi 로 최단경로를 따라가는데,
    target_yaw 가 현재 yaw 와 정확히 180도 차이(예: 후진+180도 회전)면 wrap_pi(diff) 의 부호가
    +π/-π 경계에서 부동소수점 오차로 실행마다 뒤바뀌어 회전방향이 무작위로 보임(사용자 관찰) —
    force_direction 지정 시 그 방향이 되도록 오차를 2π 만큼 밀어서 부호를 강제한다(목표 각도
    자체는 그대로, 최단경로가 아닐 수 있음)."""
    prev_yaw = get_chassis_yaw(chassis_path); w_applied = 0.0; settled = 0
    for _ in range(max_steps):
        yield
        if (ctx.person_gate is not None and ctx.person_gate.blocked()) or \
           (ctx.estop_flags is not None and ctx.estop_flags.get(ctx.name, False)):
            ctx.cmd_pub.publish(Twist())
            continue
        yaw = get_chassis_yaw(chassis_path)
        yaw_err = wrap_pi(target_yaw - yaw)
        if force_direction == "cw" and yaw_err > 0:
            yaw_err -= 2.0 * np.pi
        elif force_direction == "ccw" and yaw_err < 0:
            yaw_err += 2.0 * np.pi
        yaw_rate = wrap_pi(yaw - prev_yaw) / PHYSICS_DT
        prev_yaw = yaw
        if abs(yaw_err) < tol:
            settled += 1
            if settled >= 5:
                break
        else:
            settled = 0
        w_t = float(np.clip(kp * yaw_err - kd * yaw_rate, -w_max, w_max))
        w_applied += float(np.clip(w_t - w_applied, -max_w_step, max_w_step))
        tw = Twist(); tw.angular.z = w_applied; ctx.cmd_pub.publish(tw)
    ctx.cmd_pub.publish(Twist())
    for _ in range(FINAL_MOVE_SETTLE_STEPS):
        yield


def g_drive_straight_open_loop(ctx, distance, chassis_path, speed=FINAL_APPROACH_SPEED,
                               ramp_time=FINAL_APPROACH_RAMP_TIME,
                               drift_kp=FINAL_APPROACH_DRIFT_KP, drift_kd=FINAL_APPROACH_DRIFT_KD,
                               drift_w_max=FINAL_APPROACH_DRIFT_W_MAX,
                               drift_max_w_step=FINAL_APPROACH_DRIFT_MAX_W_STEP,
                               reverse=False):
    sign = -1.0 if reverse else 1.0
    start_xy = get_prim_world_position(chassis_path)[:2]
    target_yaw = get_chassis_yaw(chassis_path)
    forward = np.array([np.cos(target_yaw), np.sin(target_yaw)])
    max_steps = int(round(distance / speed / PHYSICS_DT)) + 100
    ramp_steps = max(1, int(round(ramp_time / PHYSICS_DT)))
    prev_yaw = target_yaw; w_applied = 0.0
    for step in range(max_steps):
        yield
        if (ctx.person_gate is not None and ctx.person_gate.blocked()) or \
           (ctx.estop_flags is not None and ctx.estop_flags.get(ctx.name, False)):
            ctx.cmd_pub.publish(Twist())
            continue
        pos_xy = get_prim_world_position(chassis_path)[:2]
        progress = float(np.dot(pos_xy - start_xy, forward)) * sign
        alpha = min(1.0, (step + 1) / ramp_steps)
        cur_speed = sign * speed * _smoothstep(alpha)
        yaw = get_chassis_yaw(chassis_path)
        yaw_err = wrap_pi(yaw - target_yaw)
        yaw_rate = wrap_pi(yaw - prev_yaw) / PHYSICS_DT
        prev_yaw = yaw
        w_t = float(np.clip(-(drift_kp * yaw_err + drift_kd * yaw_rate), -drift_w_max, drift_w_max))
        w_applied += float(np.clip(w_t - w_applied, -drift_max_w_step, drift_max_w_step))
        tw = Twist(); tw.linear.x = float(cur_speed); tw.angular.z = w_applied
        ctx.cmd_pub.publish(tw)
        if progress >= distance:
            break
    ctx.cmd_pub.publish(Twist())
    for _ in range(FINAL_MOVE_SETTLE_STEPS):
        yield


def g_stow_arm_for_nav(ctx, ramp_steps=JOINT_RAMP_STEPS):
    """폐기물통을 쥐고 있지 않은 Nav2 주행 구간 진입 전, 팔을 NAV_STOW_Q_DEG(무게중심 낮춤+챠시
    중심 정렬, 라이다 가림 없음) 자세로 접는다. ★폐기물통 파지 중 구간에는 호출 금지★."""
    yield from g_ramp_to_joint_positions(ctx, NAV_STOW_Q_DEG, ramp_steps)


def g_run_nav_leg(ctx, standoff_xy, standoff_yaw, chassis_goal_xy, chassis_goal_yaw, label,
                  reverse_entry=False):
    """Nav2 목표(standoff) 발행 → /start_pick 대기 → 실제 위치 기준 회전→직진→회전.
    [2026-07-26] reverse_entry=True 면 목표를 향해 도는 대신 목표 반대방향(=chassis_goal_yaw)을 보고
    후진으로 진입한다 — 팔이 챠시 원점보다 뒤쪽(MOUNT_OFFSET x<0)에 달려있어, 노즐 거치대처럼 팔이
    닿아야 하는 지점에 후진 진입하면 그만큼 팔-목표 거리가 짧아져 더 먼 DOCK_STANDOFF 를 쓸 수 있다
    (사용자 제안). 도착 시 이미 최종 방향(chassis_goal_yaw=목표 반대방향)을 보고 있으므로 이후 별도
    회전 없이 바로 멀어질 수 있다는 게 부가 이점."""
    for _ in range(30):
        yield
    ctx.pick_state["start"] = False

    goal = PoseStamped(); goal.header.frame_id = "map"
    goal.pose.position.x = float(standoff_xy[0]); goal.pose.position.y = float(standoff_xy[1])
    goal.pose.orientation.z = float(np.sin(standoff_yaw / 2.0))
    goal.pose.orientation.w = float(np.cos(standoff_yaw / 2.0))

    print(f"[NAV:{label}][{ctx.name}] standoff={standoff_xy.tolist()} yaw={np.degrees(standoff_yaw):.1f} 발행")
    ctx.status = f"{label}: Nav2 이동 대기(nav_goal 발행중, start_pick 기다림)"
    while not ctx.pick_state["start"]:
        yield
        # ★긴급정지(어떤 상황에서든 정지)★ : 이 구간은 Nav2 가 로봇을 몰아(스크립트 cmd_vel 아님)
        #   여기서 직접 못 멈춘다 → (1)standoff goal 재발행을 멈추고 (2)cmd_vel 0 을 발행해 즉시
        #   정지시키며 (3)미션 릴레이(trash_can_nav_pick_mission)가 /robot/command 로 진행 중 Nav2
        #   goal 을 취소하게 둔다. estop 해제 시 goal 재발행 재개 → Nav2 재주행. (person 은 이 구간에선
        #   costmap 이 알아서 우회하므로 게이팅 안 함 — 설계상 estop 만.)
        if ctx.estop_flags is not None and ctx.estop_flags.get(ctx.name, False):
            ctx.cmd_pub.publish(Twist())
            if not simulation_app.is_running():
                return
            continue
        st = float(ctx.world.current_time)
        goal.header.stamp.sec = int(st)
        goal.header.stamp.nanosec = int(round((st - int(st)) * 1e9))
        ctx.nav_goal_pub.publish(goal)
        if not simulation_app.is_running():
            return

    print(f"[NAV:{label}][{ctx.name}] /start_pick 수신 → 최종 접근")
    ctx.status = f"{label}: 최종 접근(회전→직진→회전)"
    cur = get_prim_world_position(ctx.articulation_root)[:2]
    to_goal = chassis_goal_xy - cur
    entry_distance = float(np.linalg.norm(to_goal))
    entry_yaw = float(np.arctan2(to_goal[1], to_goal[0]))
    if reverse_entry:
        entry_yaw = wrap_pi(entry_yaw + np.pi)
    yield from g_rotate_in_place(ctx, entry_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, ctx.articulation_root)
    # [2026-07-26] PRE_ROTATE_NUDGE_DISTANCE 는 전진 진입 기준으로 "약간 초과주행 후 최종 회전으로
    # 보정"하는 여유값이다 — reverse_entry 에서 그대로 더하면 초과주행 방향이 거치대 쪽이 되어(뒤로
    # 더 파고듦) DOCK_STANDOFF 로 확보한 여유를 까먹는다. 후진 진입은 이 여유 없이 정확히
    # entry_distance 만큼만 이동.
    drive_distance = entry_distance if reverse_entry else entry_distance + PRE_ROTATE_NUDGE_DISTANCE
    yield from g_drive_straight_open_loop(ctx, drive_distance, ctx.articulation_root,
                                          reverse=reverse_entry)
    yield from g_rotate_in_place(ctx, chassis_goal_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, ctx.articulation_root)
    if FINAL_NUDGE_DISTANCE > 0.0:
        yield from g_drive_straight_open_loop(ctx, FINAL_NUDGE_DISTANCE, ctx.articulation_root,
                                              reverse=reverse_entry)


def create_plug_at_world_pos(trash_can_path, world_pos, plug_name="grip_plug"):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(trash_can_path)
    desired = Gf.Vec3d(float(world_pos[0]), float(world_pos[1]), float(world_pos[2]))
    local = omni.usd.get_world_transform_matrix(prim).GetInverse().Transform(desired)
    plug_path = f"{trash_can_path}/{plug_name}"
    if stage.GetPrimAtPath(plug_path).IsValid():
        stage.RemovePrim(plug_path)
    px = UsdGeom.Xform.Define(stage, plug_path)
    px.AddTranslateOp().Set(local)
    px.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return plug_path


def g_dump_into_big_trash(ctx):
    cur = ctx.robot.get_joint_positions()
    dump_deg = []
    for name in ARM_JOINT_NAMES:
        idx = ctx.dof_names.index(name)
        if name == "joint_1":
            dump_deg.append(DUMP_J1_DEG)
        elif name == "joint_6":
            dump_deg.append(float(np.degrees(cur[idx])) + DUMP_J6_ROTATE_DEG)
        else:
            dump_deg.append(float(np.degrees(cur[idx])))
    yield from g_ramp_to_joint_positions(ctx, dump_deg, DUMP_RAMP_STEPS)
    print(f"[INFO][{ctx.name}] big_trash 덤프 완료 (j1={DUMP_J1_DEG}, j6+={DUMP_J6_ROTATE_DEG})")

    print(f"[INFO][{ctx.name}] 쓰레기통 털기 (위아래 흔들기)")
    shake_steps = 20
    shake_amp = 15.0
    for _ in range(2):
        shake_up = list(dump_deg)
        shake_up[4] += shake_amp  # joint_5 위로
        yield from g_ramp_to_joint_positions(ctx, shake_up, shake_steps)
        shake_down = list(dump_deg)
        shake_down[4] -= shake_amp  # joint_5 아래로
        yield from g_ramp_to_joint_positions(ctx, shake_down, shake_steps)
    
    yield from g_ramp_to_joint_positions(ctx, dump_deg, shake_steps)


def g_restore_upright_after_dump(ctx):
    cur = ctx.robot.get_joint_positions()
    restore_deg = []
    for name in ARM_JOINT_NAMES:
        idx = ctx.dof_names.index(name)
        if name == "joint_6":
            restore_deg.append(float(np.degrees(cur[idx])) - DUMP_J6_ROTATE_DEG)
        else:
            restore_deg.append(float(np.degrees(cur[idx])))
    yield from g_ramp_to_joint_positions(ctx, restore_deg, DUMP_RAMP_STEPS)
    print(f"[INFO][{ctx.name}] 쓰레기통 다시 위로 복귀 (j6-={DUMP_J6_ROTATE_DEG})")


def _pick_closest_entry(trash_origin_xy, from_xy):
    best = None
    for k in range(4):
        theta = k * (np.pi / 2.0)
        goal_xy = trash_origin_xy - rotate_2d(OFFSET_TRASH_FROM_CHASSIS, theta)
        d = float(np.linalg.norm(goal_xy - from_xy))
        if best is None or d < best[2]:
            best = (goal_xy, theta, d)
    return best[0], best[1]


# ════════════════════════════════════════════════════════════════════════════
#  H. 툴체인지 제너레이터(15_/16_ 이식, ctx 만 참조)
# ════════════════════════════════════════════════════════════════════════════
def _tc_solve_and_ramp(ctx, ik, target_pos, target_ori, warm_start, ramp_steps, label,
                       bias_compensation=None, retries=TC_IK_RETRY_COUNT):
    """거치대는 고정 위치라 반응형 IK(RMPflow) 대신 "IK 1회 풀어 관절각 확정 → 램프"
    (15_ 실측: 반응형 IK 는 오래 돌수록 오히려 발산). position_tolerance=0.001 +
    TC_TARGET_BIAS_COMPENSATION(URDF-USD 형상 불일치 고정 바이어스 보정, TC_APPROACH_ORIENTATION
    접근에만 유효). 반환 = (관절각 rad 6dof|None, ok).
    [2026-07-26] 챠시 위치 보정을 없앤 대신, IK 가 안 풀리는 경우 그 자리(챠시 이동 없음)에서
    warm_start 를 바꿔가며 최대 retries 회 재시도(사용자 요청) — 같은 입력을 그대로 반복하면 결정론적
    솔버라 100% 재실패하므로, warm_start 유무를 번갈아 써서 실제로 다른 해를 탐색하게 한다."""
    bias = bias_compensation if bias_compensation is not None else TC_TARGET_BIAS_COMPENSATION
    corrected_target = np.asarray(target_pos) - bias
    q, ok = None, False
    for attempt in range(retries + 1):
        attempt_warm_start = warm_start if attempt % 2 == 0 else None
        q, ok = ik.compute_inverse_kinematics(
            EE_FRAME, corrected_target, target_ori,
            warm_start=attempt_warm_start, position_tolerance=0.001, orientation_tolerance=0.02)
        if ok:
            break
        if attempt < retries:
            print(f"[TOOLCHANGE][{ctx.name}][WARN] {label} IK 실패(시도 {attempt + 1}/{retries + 1}) — 재시도")
    if not ok:
        print(f"[TOOLCHANGE][{ctx.name}][WARN] {label} IK 실패")
        return None, False
    q6 = np.asarray(q[:6])
    yield from g_ramp_to_joint_positions(ctx, np.degrees(q6), ramp_steps)
    actual_pos = get_prim_world_position(ctx.ee_path)
    actual_err = actual_pos - np.asarray(target_pos)
    print(f"[TOOLCHANGE][{ctx.name}] {label} 관절이동 완료 q={np.round(q6, 3)} "
          f"(link_6 오차 |e|={np.linalg.norm(actual_err) * 1000:.2f}mm)")
    return q6, True


def g_creep_to(ctx, target_pos, target_ori, step_size=TC_CREEP_STEP_SIZE, settle_steps=TC_CREEP_SETTLE_STEPS,
               max_steps=200, tol=0.003):
    """[2026-07-27 재보정 추가] 현재 손끝 위치에서 target_pos 까지 RMPflow 로 작은 스텝씩(그립 시도
    없이) 이동만 한다 — 노즐 크립 재시도에서 "안전 고도로 들어올림"/"고도 유지한 채 수평이동" 구간에
    공용으로 쓴다(수직 하강+그립 재시도는 호출부가 별도로 처리)."""
    current = get_prim_world_position(ctx.ee_path)
    for _ in range(max_steps):
        to_target = np.asarray(target_pos) - current
        dist = float(np.linalg.norm(to_target))
        if dist <= tol:
            return
        current = current + to_target / dist * min(step_size, dist)
        for _ in range(settle_steps):
            yield
            ctx.robot.apply_action(ctx.rmpflow.forward(
                target_end_effector_position=current, target_end_effector_orientation=target_ori))


def g_tool_change_grasp(ctx):
    """자기 전용 거치대(ctx.tool_path/dock_xy)에서 노즐 파지. 성공 시 ctx.holding_nozzle=True,
    ctx.nozzle_tip_offset 갱신. 반환값 = grasp_ok(bool).
    [2026-07-27] IK 1회 솔브("노즐 하강") 직후 그립이 바로 안 되면, g_trash_mission 의 크립(creep)
    패턴을 이식해 RMPflow 로 실측 handle_position 을 향해 조금씩 다가가며 재시도한다(TC_CREEP_*).
    챠시 접근(Nav2 핸드오프) 오차가 보정 없이 그대로 IK 목표 오차로 전달되던 구조적 약점을 흡수."""
    tc = ctx.tool_changer
    handle_position, handle_orientation = tc.approach_tool_stand()
    base_pos, base_quat = read_world_pose(f"{ctx.arm_root}/base_link")
    ik = mg.LulaKinematicsSolver(robot_description_path=IK_DESCRIPTION_PATH, urdf_path=IK_URDF_PATH)
    ik.set_robot_base_pose(base_pos, base_quat)

    q_above, ok_above = yield from _tc_solve_and_ramp(
        ctx, ik, handle_position + TC_EE_OFFSET, handle_orientation, None, TC_JOINT_RAMP_STEPS, "노즐 상공 접근")
    if not ok_above:
        return False
    q_grasp, ok_grasp = yield from _tc_solve_and_ramp(
        ctx, ik, handle_position + TC_GRASP_CLEARANCE, handle_orientation, q_above, TC_JOINT_RAMP_STEPS, "노즐 하강")
    if not ok_grasp:
        yield from g_ramp_to_joint_positions(ctx, np.degrees(q_above), TC_JOINT_RAMP_STEPS)
        return False

    tc.grasp_mop()
    for _ in range(TC_GRASP_SETTLE_STEPS):
        yield
    if not tc.surface_gripper.is_closed():
        # [2026-07-27 재보정 추가] IK 1회 솔브가 챠시 접근오차(Nav2 핸드오프 오차)를 그대로 물려받아
        # 그립범위(MAX_GRIP_DISTANCE=0.04m)를 놓친 경우 — g_trash_mission 의 크립(creep) 패턴을 그대로
        # 이식해 실측 handle_position 을 향해 RMPflow 로 조금씩 다가가며 매 스텝 재그립 시도.
        # [2026-07-27 라이브 사고 대응] "노즐 하강" 직후(=이미 손잡이 높이) 상태 그대로 옆으로
        # 크립하면 hold_joint 로 아직 고정돼 있는 노즐 본체와 부딪혀 챠시까지 흔들리는 사고가
        # 실제로 발생함 — 반드시 "상공 접근"과 같은 안전 고도로 먼저 들어올린 뒤(1), 그 고도를
        # 유지한 채 XY 만 목표 바로 위로 맞추고(2), 마지막에 그 자리서 수직으로만 하강(3)한다.
        print(f"[TOOLCHANGE][{ctx.name}][WARN] 노즐 하강 직후 파지 실패 — 크립 재시도 시작 "
              f"(최대 {TC_CREEP_MAX_STEPS}스텝×{TC_CREEP_STEP_SIZE * 1000:.0f}mm)")
        handle_position_arr = np.asarray(handle_position)
        lift_target = handle_position_arr + TC_EE_OFFSET

        print(f"[TOOLCHANGE][{ctx.name}] 크립 1/3: 안전 고도로 들어올림")
        yield from g_creep_to(ctx, lift_target, handle_orientation)

        print(f"[TOOLCHANGE][{ctx.name}] 크립 2/3: 고도 유지한 채 목표 바로 위로 수평이동")
        above_target = np.array([handle_position_arr[0], handle_position_arr[1], lift_target[2]])
        yield from g_creep_to(ctx, above_target, handle_orientation)

        print(f"[TOOLCHANGE][{ctx.name}] 크립 3/3: 목표 위에서 수직 하강하며 재그립")
        creep_start = get_prim_world_position(ctx.ee_path)
        to_handle = handle_position_arr - creep_start
        remaining = float(np.linalg.norm(to_handle))
        creep_dir = to_handle / remaining if remaining > 1e-6 else np.array([0.0, 0.0, -1.0])
        current_target = creep_start.copy()
        gripped_ok = False
        for creep_step in range(TC_CREEP_MAX_STEPS):
            current_target = current_target + creep_dir * TC_CREEP_STEP_SIZE
            for _ in range(TC_CREEP_SETTLE_STEPS):
                yield
                ctx.robot.apply_action(ctx.rmpflow.forward(
                    target_end_effector_position=current_target, target_end_effector_orientation=handle_orientation))
            # [2026-07-27] target 에 TC_CREEP_GRIP_ATTEMPT_RADIUS 안으로 들어오기 전엔 grip 을 시도하지
            # 않는다 — 그립범위(0.04m) 가장자리에서 바로 잡혀 노즐이 삐딱하게 붙는 것 방지.
            remaining_now = float(np.linalg.norm(handle_position_arr - current_target))
            if remaining_now > TC_CREEP_GRIP_ATTEMPT_RADIUS:
                continue
            tc.surface_gripper.close()
            if tc.surface_gripper.is_closed():
                gripped_ok = True
                traveled_mm = float(np.linalg.norm(current_target - creep_start)) * 1000
                print(f"[TOOLCHANGE][{ctx.name}] 크립 파지 성공 (creep {creep_step + 1}/{TC_CREEP_MAX_STEPS}, "
                      f"누적 이동={traveled_mm:.1f}mm, 목표까지 잔여={remaining_now * 1000:.1f}mm)")
                break
        if not gripped_ok:
            print(f"[TOOLCHANGE][{ctx.name}][FAIL] 노즐 파지 실패 "
                  f"(크립 {TC_CREEP_MAX_STEPS}회 소진, {TC_CREEP_MAX_STEPS * TC_CREEP_STEP_SIZE * 1000:.0f}mm "
                  f"이내 그립 안 됨)")
            yield from g_ramp_to_joint_positions(ctx, np.degrees(q_above), TC_JOINT_RAMP_STEPS)
            return False

    release_hold_joint(ctx.stage, ctx.hold_joint_path)
    for _ in range(10):
        yield

    rel_pos, _ = relative_pose(ctx.ee_path, ctx.tcp_path)
    ctx.nozzle_tip_offset = rel_pos
    base_offset = get_prim_world_position(ctx.tool_path) - get_prim_world_position(ctx.ee_path)
    tc.fingertip_offset_from_ik_frame = base_offset
    ctx.holding_nozzle = True
    print(f"[TOOLCHANGE][{ctx.name}] 노즐 파지 성공. link_6 기준 tcp 오프셋={np.round(rel_pos, 4)} "
          f"(|rel|={np.linalg.norm(rel_pos):.4f})")
    publish_hmi_state(ctx, "노즐 장착")  # [HMI v2] SPRAY_STEPS
    yield from g_ramp_to_joint_positions(ctx, np.degrees(q_above), TC_JOINT_RAMP_STEPS)
    return True


def g_tool_change_release(ctx):
    """노즐을 자기 전용 거치대에 반납. 성공 시 ctx.holding_nozzle=False, hold_joint 재체결,
    fingertip_offset 리셋. 반환값 = release_ok(bool)."""
    tc = ctx.tool_changer
    yield from g_ramp_to_joint_positions(ctx, np.degrees(STOW_Q), SPRAY_ENTRY_RAMP_STEPS)
    stand_position, stand_orientation = tc.stand_return_target()
    base_pos, base_quat = read_world_pose(f"{ctx.arm_root}/base_link")
    ik = mg.LulaKinematicsSolver(robot_description_path=IK_DESCRIPTION_PATH, urdf_path=IK_URDF_PATH)
    ik.set_robot_base_pose(base_pos, base_quat)

    q_above, ok_above = yield from _tc_solve_and_ramp(
        ctx, ik, stand_position + TC_EE_OFFSET, stand_orientation, None, TC_JOINT_RAMP_STEPS, "거치대 상공 복귀")
    if not ok_above:
        return False
    q_down, ok_down = yield from _tc_solve_and_ramp(
        ctx, ik, stand_position + TC_GRASP_CLEARANCE, stand_orientation, q_above, TC_JOINT_RAMP_STEPS, "거치대 하강")
    if not ok_down:
        yield from g_ramp_to_joint_positions(ctx, np.degrees(q_above), TC_JOINT_RAMP_STEPS)
        return False

    tc.release_mop_to_stand()
    for _ in range(15):
        yield
    release_ok = not tc.surface_gripper.is_closed()
    engage_hold_joint(ctx.stage, ctx.hold_joint_path)
    for _ in range(TC_REDOCK_SETTLE_STEPS):
        yield
    ctx.holding_nozzle = False
    ctx.nozzle_tip_offset = None
    tc.fingertip_offset_from_ik_frame = TC_FINGERTIP_OFFSET_FROM_TOOL0.copy()
    print(f"[TOOLCHANGE][{ctx.name}] 노즐 반납 {'성공' if release_ok else '실패'} + 재도킹 완료")
    yield from g_ramp_to_joint_positions(ctx, np.degrees(q_above), TC_JOINT_RAMP_STEPS)
    return release_ok



# ════════════════════════════════════════════════════════════════════════════
#  I. 분사 스윕(WIPE/MOVE) 제너레이터(15_/16_ 이식, ctx 만 참조)
# ════════════════════════════════════════════════════════════════════════════
def _basis_from_z(d):
    d = d / (np.linalg.norm(d) + 1e-9)
    a = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    x = np.cross(a, d); x /= (np.linalg.norm(x) + 1e-9)
    y = np.cross(d, x)
    return np.column_stack([x, y, d])


def _cone_dirs(direction, half_angle_rad, n):
    cos_a = np.cos(half_angle_rad)
    z = np.random.uniform(cos_a, 1.0, n)
    phi = np.random.uniform(0.0, 2.0 * np.pi, n)
    s = np.sqrt(np.clip(1.0 - z * z, 0.0, 1.0))
    local = np.stack([s * np.cos(phi), s * np.sin(phi), z], axis=1)
    return local @ _basis_from_z(direction).T


class SprayFX:
    def __init__(self, stage, prim_path="/World/spray_fx"):
        self.N = int(SPRAY_MAX)
        self.pos = np.zeros((self.N, 3), dtype=np.float32)
        self.vel = np.zeros((self.N, 3), dtype=np.float32)
        self.age = np.full(self.N, SPRAY_LIFETIME + 1.0, dtype=np.float32)
        self._cursor = 0
        pts = UsdGeom.Points.Define(stage, prim_path)
        pts.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(self.pos))
        pts.CreateWidthsAttr(Vt.FloatArray.FromNumpy(np.zeros(self.N, dtype=np.float32)))
        pts.CreateDisplayColorAttr([Gf.Vec3f(*SPRAY_COLOR)])
        pts.CreateDisplayOpacityAttr([float(SPRAY_OPACITY)])
        self._pts_attr = pts.GetPointsAttr()
        self._w_attr = pts.GetWidthsAttr()

    def _emit(self, origin, direction, n):
        idx = (self._cursor + np.arange(n)) % self.N
        self._cursor = int((self._cursor + n) % self.N)
        self.pos[idx] = origin
        self.vel[idx] = (SPRAY_SPEED * _cone_dirs(direction, np.radians(SPRAY_CONE_DEG), n)).astype(np.float32)
        self.age[idx] = 0.0

    def update(self, spraying, origin, direction, dt):
        if spraying and origin is not None:
            self._emit(np.asarray(origin, dtype=np.float32),
                       np.asarray(direction, dtype=np.float32), int(SPRAY_RATE))
        alive = self.age < SPRAY_LIFETIME
        self.vel[alive, 2] += SPRAY_GRAVITY * dt
        self.pos[alive] += self.vel[alive] * dt
        self.age[alive] += dt
        frac = np.clip(self.age / SPRAY_LIFETIME, 0.0, 1.0)
        widths = np.where(self.age < SPRAY_LIFETIME, SPRAY_SIZE * (0.6 + 0.8 * frac), 0.0)
        self._pts_attr.Set(Vt.Vec3fArray.FromNumpy(self.pos))
        self._w_attr.Set(Vt.FloatArray.FromNumpy(widths.astype(np.float32)))

    def clear(self):
        self.age[:] = SPRAY_LIFETIME + 1.0
        self._w_attr.Set(Vt.FloatArray.FromNumpy(np.zeros(self.N, dtype=np.float32)))


class Sweeper:
    def __init__(self, lo, hi, cruise, accel, dt, hold_steps=6):
        self.lo, self.hi = lo, hi
        self.cruise, self.accel, self.dt = cruise, accel, dt
        self.hold_steps = hold_steps
        self.reset_bottom()

    def reset_bottom(self):
        self.s = self.lo; self.dir = 1.0; self.v = 0.0; self.hold = 0; self.strokes = 0

    def step(self):
        if self.hold > 0:
            self.hold -= 1
            if self.hold == 0:
                self.dir *= -1.0
            return self.s
        end = self.lo if self.dir < 0 else self.hi
        dr = abs(end - self.s)
        v_decel = np.sqrt(max(0.0, 2.0 * self.accel * dr))
        self.v = min(self.cruise, self.v + self.accel * self.dt, v_decel)
        self.s += self.dir * self.v * self.dt
        if dr <= 2e-3:
            self.s = end; self.v = 0.0; self.hold = self.hold_steps
            self.strokes += 1
        return self.s


def g_spray_sweep(ctx, forward_distance=FORWARD_DISTANCE, max_steps=None):
    """WIPE(정지+상하 스윕)↔MOVE(팔 고정+전진, heading-hold) 반복. 사전조건: ctx.holding_nozzle=True,
    ctx.nozzle_tip_offset 실측 완료. 반환 = True(목표 거리 도달, 또는 사람회피로 '완료' 취급) /
    False(조기종료·조준IK실패·grip풀림·★긴급정지(ESTOP)★=취소)."""
    if not ctx.holding_nozzle or ctx.nozzle_tip_offset is None:
        print(f"[SPRAY][{ctx.name}][FAIL] 노즐 파지 상태에서만 호출 가능(holding=False) — 중단")
        return False

    stage = ctx.stage
    base_pos, base_quat = read_world_pose(f"{ctx.arm_root}/base_link")
    ori = aim_orientation_world(base_quat)
    ik = mg.LulaKinematicsSolver(robot_description_path=IK_DESCRIPTION_PATH, urdf_path=IK_URDF_PATH)
    ik.set_robot_base_pose(base_pos, base_quat)
    q_low, ok_lo = ik.compute_inverse_kinematics(
        EE_FRAME, aim_link6_world_from_offset(base_pos, base_quat, ori, Z_LOW, ctx.nozzle_tip_offset), ori,
        position_tolerance=0.005, orientation_tolerance=0.05)
    q_high, ok_hi = ik.compute_inverse_kinematics(
        EE_FRAME, aim_link6_world_from_offset(base_pos, base_quat, ori, Z_HIGH, ctx.nozzle_tip_offset), ori,
        warm_start=(q_low if ok_lo else None),
        position_tolerance=0.005, orientation_tolerance=0.05)
    print(f"[SPRAY][{ctx.name}] 조준 IK q_low ok={ok_lo} q_high ok={ok_hi}")
    if not (ok_lo and ok_hi):
        print(f"[SPRAY][{ctx.name}][FAIL] 조준 IK 실패 (low={ok_lo}, high={ok_hi}) — 이 웨이포인트 스윕 불가")
        return False
    q_low = np.asarray(q_low[:6]); q_high = np.asarray(q_high[:6])
    q_mid = 0.5 * (q_low + q_high); q_half = 0.5 * (q_high - q_low)

    def q_of_s(s):
        q = q_mid + q_half * s
        q[J5_INDEX] += J5_FLICK * s
        q[J1_INDEX] += J1_OFFSET
        return q

    arm_idx = np.array([ctx.dof_names.index(n) for n in ARM_JOINT_NAMES])
    q_low_deg = np.degrees(q_of_s(-1.0))
    print(f"[SPRAY][{ctx.name}] 스윕 자세로 진입(부드럽게, {SPRAY_ENTRY_RAMP_STEPS}스텝)")
    yield from g_ramp_to_joint_positions(ctx, q_low_deg, SPRAY_ENTRY_RAMP_STEPS)

    q_applied = q_of_s(-1.0).copy()
    sweeper = Sweeper(-1.0, 1.0, S_CRUISE, S_ACCEL, PHYSICS_DT, S_HOLD_STEPS)
    q_hold = q_of_s(-1.0)
    spray_fx = SprayFX(stage) if SPRAY_FX_ON else None
    ctx.spray_fx = spray_fx  # [HMI v2] 밖(g_return_dock_and_reset 등)에서도 정리할 수 있게 노출

    def apply(q_target):
        nonlocal q_applied
        q_applied = q_applied + np.clip(q_target - q_applied, -MAX_JOINT_STEP, MAX_JOINT_STEP)
        ctx.robot.apply_action(ArticulationAction(joint_positions=q_applied, joint_indices=arm_idx))

    def publish_cmd(vx, wz, state):
        dv = FORWARD_ACCEL * PHYSICS_DT
        v = state["vx"] + float(np.clip(float(vx) - state["vx"], -dv, dv))
        state["vx"] = v
        tw = Twist(); tw.linear.x = v; tw.angular.z = float(wz); ctx.cmd_pub.publish(tw)

    phase = "WIPE"; heading_ready = False
    global_start = None; forward0 = None; target_yaw = 0.0; prev_yaw = None
    progress = 0.0; move_start_prog = 0.0; cycle = 0
    drive_state = {"vx": 0.0}
    step_i = 0

    while True:
        step_i += 1

        # ★YOLO+HMI 병합★ ESTOP 과 사람회피(person-block) 는 의미가 다르므로 반환값을 구분한다
        #   (사용자 결정) : ESTOP=사용자가 멈추라고 한 것=취소(False, sweep_done 미발행 취급) /
        #   사람회피=자동 우회 목적=완료 취급(True, 다음 웨이포인트로 자동 진행). g_spray_mission_body
        #   가 이 반환값을 안 보고 무조건 거치대 복귀를 시도하는 구조라(17_ 원본 그대로), ESTOP 취소
        #   후의 복귀 이동도 g_run_nav_leg → g_drive_straight_open_loop/g_rotate_in_place 의 estop
        #   체크로 똑같이 제자리에서 멈춘다 — 별도 재시도/취소 로직 없이 "멈춘 자리에서 START 시
        #   이어서 재개"가 성립한다.
        if ctx.estop_flags is not None and ctx.estop_flags.get(ctx.name, False):
            publish_cmd(0.0, 0.0, drive_state)
            apply(q_hold)
            if spray_fx is not None:
                spray_fx.clear()
            print(f"[SPRAY][{ctx.name}][ESTOP] 긴급정지 → 스윕 취소")
            return False

        person_near = ctx.person_gate is not None and ctx.person_gate.blocked()
        if person_near:
            publish_cmd(0.0, 0.0, drive_state)
            apply(q_hold)
            if spray_fx is not None:
                spray_fx.clear()
            print(f"[SPRAY][{ctx.name}][AVOID] 사람 감지 → 소독 중단·정지, 스윕 종료 통지 → "
                  f"다음 작업점(거치대 복귀 후 다음 task_select 대기)")
            return True

        if max_steps is not None and step_i > max_steps:
            publish_cmd(0.0, 0.0, drive_state)
            if spray_fx is not None:
                spray_fx.clear()
            print(f"[SPRAY][{ctx.name}] max_steps({max_steps}) 도달 → 조기 종료(디버그)")
            return False

        if not (ctx.gripper.is_closed() if ctx.gripper else True):
            publish_cmd(0.0, 0.0, drive_state)
            if spray_fx is not None:
                spray_fx.clear()
            print(f"[SPRAY][{ctx.name}][WARN] 스윕 중 grip 이 풀림 감지 — 중단")
            ctx.holding_nozzle = False
            return False

        chassis_pos, chassis_quat = read_world_pose(ctx.articulation_root)
        chassis_R = quat_to_matrix(chassis_quat)
        fwd_now = chassis_R @ np.array([1.0, 0.0, 0.0])
        yaw_now = float(np.arctan2(fwd_now[1], fwd_now[0]))
        if prev_yaw is None:
            prev_yaw = yaw_now
        yaw_rate = wrap_pi(yaw_now - prev_yaw) / PHYSICS_DT
        prev_yaw = yaw_now

        if not heading_ready:
            global_start = chassis_pos.copy()
            raw_yaw = yaw_now
            snapped_yaw = round(raw_yaw / (np.pi / 2.0)) * (np.pi / 2.0)
            target_yaw = float(snapped_yaw)
            forward0 = np.array([np.cos(snapped_yaw), np.sin(snapped_yaw), 0.0])
            heading_ready = True
            print(f"[SPRAY][{ctx.name}][PHASE] WIPE 시작 (도착yaw={np.degrees(raw_yaw):.1f} → "
                  f"world축 스냅 {np.degrees(snapped_yaw):.0f})")

        progress = float(np.dot(chassis_pos - global_start, forward0))
        if progress >= forward_distance:
            publish_cmd(0.0, 0.0, drive_state)
            if spray_fx is not None:
                spray_fx.clear()
            print(f"[SPRAY][{ctx.name}] {forward_distance:.1f}m 도달(progress={progress:.2f}) → 스윕 종료")
            return True

        if spray_fx is not None:
            spraying = phase == "WIPE"
            if spraying:
                n_pos, n_quat = read_world_pose(ctx.tcp_path)
                Rn = quat_to_matrix(n_quat)
                s_dir = Rn @ np.array([0.0, 0.0, 1.0])
                spray_fx.update(True, n_pos, s_dir, PHYSICS_DT)
            else:
                spray_fx.update(False, None, None, PHYSICS_DT)

        if phase == "WIPE":
            publish_cmd(0.0, 0.0, drive_state)
            q_target = q_of_s(sweeper.step())
            if sweeper.strokes >= STROKES_PER_WIPE:
                phase = "MOVE"; move_start_prog = progress; cycle += 1
                print(f"[SPRAY][{ctx.name}][{cycle}] WIPE→MOVE")
        else:
            q_target = q_hold
            yaw_err = wrap_pi(yaw_now - target_yaw)
            left = np.array([-forward0[1], forward0[0], 0.0])
            lateral = float(np.dot(chassis_pos - global_start, left))
            w = float(np.clip(-(KP_YAW * yaw_err + KP_LAT * lateral + KD_YAW * yaw_rate), -W_MAX, W_MAX))
            publish_cmd(FORWARD_SPEED, w, drive_state)
            if (progress - move_start_prog) >= MOVE_DISTANCE:
                publish_cmd(0.0, 0.0, drive_state)
                phase = "WIPE"; sweeper.reset_bottom()
                print(f"[SPRAY][{ctx.name}][{cycle}] MOVE→WIPE (progress={progress:.2f})")
        apply(q_target)
        yield


# ════════════════════════════════════════════════════════════════════════════
#  J. 트래시 미션(15_ carter2_mission 을 ctx-일반화 — 공용 쓰레기통 대상으로 어느 로봇이든 수행)
# ════════════════════════════════════════════════════════════════════════════
def g_trash_mission(ctx):
    """공용 쓰레기통(TRASH_CAN_PRIM) 대상 PICK → DUMP → RETURN(원위치 복귀) 3단계. 15_
    carter2_mission 과 동일 로직이되 from_xy 를 고정 스폰좌표 대신 "지금 이 순간의 챠시 위치"로
    계산(호출 시점이 스폰 직후라는 보장이 없어짐 — 작업 선택식이라 이 서브미션이 여러 번, 임의
    위치에서 시작될 수 있음). [2026-07-26] 예전엔 끝에 노즐 거치대(ctx.dock_approach_xy)로 가는
    "DOCK" 4번째 단계가 있었음 — 16_(carter2=trash 전담) 시절엔 "DOCK 복귀"가 곧 유일한 홈이라
    문제없었지만, 17_에서 dock_approach_xy 가 "노즐 거치대"라는 별개 의미로 바뀌면서 trash 작업이
    노즐을 전혀 안 쓰는데도 불필요하게 노즐 거치대를 거쳐갔다(사용자가 라이브에서 관찰해 발견) —
    제거함. 진짜 최종 복귀는 상위 g_task_select_mission 의 g_nav_to_home 이 담당.
    끝에서 무한 유휴 대신 return(최상위 task_select 루프가 IDLE 을 담당)."""
    trash_xy = np.array(TRASH_SPAWN_FIXED)
    trash_origin_xy = trash_xy - TRASH_BBOX_CENTER_OFFSET_XY
    from_xy = get_prim_world_position(ctx.articulation_root)[:2]
    chassis_goal_xy, chassis_goal_yaw = _pick_closest_entry(trash_origin_xy, from_xy)
    approach_dir = rotate_2d(OFFSET_TRASH_FROM_CHASSIS / np.linalg.norm(OFFSET_TRASH_FROM_CHASSIS), chassis_goal_yaw)
    standoff_xy = chassis_goal_xy - approach_dir * FINAL_APPROACH_DISTANCE
    standoff_yaw = float(np.arctan2(approach_dir[1], approach_dir[0]))

    for _ in range(SETTLE_STEPS):
        yield

    sync_rmpflow_base_pose(ctx)
    publish_hmi_state(ctx, "전방 주행")  # [HMI v2] TRASH_STEPS
    yield from g_stow_arm_for_nav(ctx)   # PICK 진입 전 — 아직 쓰레기통 파지 전(빈 손)
    yield from g_run_nav_leg(ctx, standoff_xy, standoff_yaw, chassis_goal_xy, chassis_goal_yaw, "PICK")
    if not simulation_app.is_running():
        return
    sync_rmpflow_base_pose(ctx)

    ctx.status = "PICK: 쓰레기통 파지 시퀀스"
    publish_hmi_state(ctx, "폐기물통 파지")  # [HMI v2]
    yield from g_ramp_to_joint_positions(ctx, TARGET_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    grasp_position = get_prim_world_position(ctx.tool0_path)
    grasp_orientation = get_world_orientation_wxyz(ctx.tool0_path)
    print(f"[INFO][{ctx.name}] 목표 관절 도달. tool0={grasp_position}")

    move_dir = rotate_vector_by_quat(grasp_orientation, np.array([0.0, 0.0, 1.0]))
    move_dir /= np.linalg.norm(move_dir)

    trash_now = get_prim_world_bbox_center(TRASH_CAN_PRIM)
    to_trash = trash_now - grasp_position
    depth = float(np.dot(to_trash, move_dir))
    lateral_vec = to_trash - depth * move_dir
    lateral_err = float(np.linalg.norm(lateral_vec))
    print(f"[INFO][{ctx.name}] 실측 쓰레기통={trash_now}, 수직오차={lateral_err:.3f}m")
    if LATERAL_CORRECTION_MIN < lateral_err <= LATERAL_CORRECTION_MAX:
        # [2026-07-27] MAX_GRIP_DISTANCE 를 1.5cm 로 좁혔으므로, 이후 깊이(depth) 크립만으로는
        # 못 고치는 좌우/상하 잔여오차가 기본 POSITION_TOLERANCE(3cm)보다 타이트해야 그립범위
        # 안에 들어온다 — 여기서만 1cm 로 좁혀서(그립범위 대비 5mm 여유) 호출.
        yield from g_move_to_pose(ctx, grasp_position + lateral_vec, grasp_orientation, "좌우/높이 보정",
                                  position_tolerance=0.01)
        grasp_position = get_prim_world_position(ctx.tool0_path)
    elif lateral_err > LATERAL_CORRECTION_MAX:
        print(f"[WARN][{ctx.name}] 수직오차 과대({lateral_err:.3f}m) → 보정 생략")

    current_target = grasp_position.copy()
    gripped_ok = False
    for creep_step in range(CREEP_MAX_STEPS):
        current_target = current_target + move_dir * CREEP_STEP_SIZE
        for _ in range(CREEP_SETTLE_STEPS):
            yield
            yield from _estop_hold(ctx)      # ★긴급정지 시 팔 정지·대기★
            ctx.robot.apply_action(ctx.rmpflow.forward(
                target_end_effector_position=current_target, target_end_effector_orientation=grasp_orientation))
        ctx.gripper.close()
        if ctx.gripper.is_closed():
            gripped_ok = True
            ctx.holding_trash = True   # [HMI v2] 도킹복귀 시 원위치 반납 판단용
            print(f"[INFO][{ctx.name}] 파지 성공 (creep {creep_step + 1})")
            break
    grasp_position = current_target
    plug_path = create_plug_at_world_pos(TRASH_CAN_PRIM, get_prim_world_position(ctx.tool0_path))
    print(f"[CHECKPOINT][{ctx.name}] gripper", "CLOSED" if gripped_ok else "파지 실패")

    lift_target = grasp_position + LIFT_OFFSET
    yield from g_move_to_pose(ctx, lift_target, grasp_orientation, "들어올리기")
    yield from g_hold_pose(ctx, lift_target, grasp_orientation, GRASP_HOLD_STEPS)

    gap = float(np.linalg.norm(get_prim_world_position(plug_path) - get_prim_world_position(ctx.tool0_path)))
    print(f"[RESULT][{ctx.name}] 그리퍼-plug 간격={gap:.4f}m ({'성공' if gap < 0.03 else '실패 의심'})")

    cur = ctx.robot.get_joint_positions()
    tuck_deg = []
    for name in ARM_JOINT_NAMES:
        idx = ctx.dof_names.index(name)
        tuck_deg.append(TUCK_J1_DEG if name == "joint_1" else float(np.degrees(cur[idx])))
    yield from g_ramp_to_joint_positions(ctx, tuck_deg, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    print(f"[INFO][{ctx.name}] j1 tuck→{TUCK_J1_DEG} 완료")

    face_dir = -BIG_TRASH_APPROACH_DIR
    big_yaw = float(np.arctan2(face_dir[1], face_dir[0]))
    big_standoff = BIG_TRASH_POSITION_XY + BIG_TRASH_APPROACH_DIR * BIG_TRASH_STANDOFF_DISTANCE
    big_goal = BIG_TRASH_POSITION_XY + BIG_TRASH_APPROACH_DIR * BIG_TRASH_FINAL_DISTANCE
    publish_hmi_state(ctx, "수거함 이동")  # [HMI v2]
    yield from g_run_nav_leg(ctx, big_standoff, big_yaw, big_goal, big_yaw, "DUMP")
    if not simulation_app.is_running():
        return
    publish_hmi_state(ctx, "폐기물 투하")  # [HMI v2]
    yield from g_dump_into_big_trash(ctx)

    yield from g_restore_upright_after_dump(ctx)
    yield from g_drive_straight_open_loop(ctx, POST_DUMP_BACKUP_DISTANCE, ctx.articulation_root,
                                          FINAL_APPROACH_SPEED, reverse=True)
    post_dump_yaw = wrap_pi(get_chassis_yaw(ctx.articulation_root) + np.pi)
    yield from g_rotate_in_place(ctx, post_dump_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, ctx.articulation_root)
    print(f"[APPROACH:DUMP][{ctx.name}] {POST_DUMP_BACKUP_DISTANCE:.2f}m 후진 + 180 회전 → RETURN 시작")

    ctx.status = "RETURN: 원위치 복귀 이동"
    publish_hmi_state(ctx, "수거통 원위치")  # [HMI v2]
    ret_from = get_prim_world_position(ctx.articulation_root)[:2]
    ret_goal_xy, ret_goal_yaw = _pick_closest_entry(trash_origin_xy, ret_from)
    ret_dir = rotate_2d(OFFSET_TRASH_FROM_CHASSIS / np.linalg.norm(OFFSET_TRASH_FROM_CHASSIS), ret_goal_yaw)
    ret_standoff_xy = ret_goal_xy - ret_dir * FINAL_APPROACH_DISTANCE
    ret_standoff_yaw = float(np.arctan2(ret_dir[1], ret_dir[0]))
    yield from g_run_nav_leg(ctx, ret_standoff_xy, ret_standoff_yaw, ret_goal_xy, ret_goal_yaw, "RETURN")
    if not simulation_app.is_running():
        return
    sync_rmpflow_base_pose(ctx)

    ctx.status = "RETURN: 안전 관절복귀 후 내려놓기"
    yield from g_ramp_to_joint_positions(ctx, TARGET_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    yield from g_move_to_pose(ctx, grasp_position, grasp_orientation, "원위치 내려놓기",
                              growing_tolerance_max=RETURN_PLACE_GROWING_TOLERANCE_MAX)
    yield from g_hold_pose(ctx, grasp_position, grasp_orientation, GRASP_HOLD_STEPS)
    ctx.gripper.open()
    ctx.holding_trash = False   # [HMI v2]
    for _ in range(GRASP_HOLD_STEPS):
        yield
    print(f"[INFO][{ctx.name}] Surface Gripper 개방 (is_closed={ctx.gripper.is_closed()})")
    retract_target = grasp_position + LIFT_OFFSET
    yield from g_move_to_pose(ctx, retract_target, grasp_orientation, "내려놓은 후 후퇴")
    yield from g_hold_pose(ctx, retract_target, grasp_orientation, GRASP_HOLD_STEPS)
    yield from g_stow_arm_for_nav(ctx)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    print(f"[INFO][{ctx.name}] 팔 안정자세(NAV_STOW_Q_DEG) 복귀 완료")

    yield from g_drive_straight_open_loop(ctx, POST_RETURN_BACKUP_DISTANCE, ctx.articulation_root,
                                          FINAL_APPROACH_SPEED, reverse=True)
    post_ret_yaw = wrap_pi(get_chassis_yaw(ctx.articulation_root) + np.pi)
    # [2026-07-27 사용자 요청] 쓰레기통 두고 후진 후 180도 회전은 항상 시계방향으로.
    yield from g_rotate_in_place(ctx, post_ret_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, ctx.articulation_root,
                                 force_direction="cw")
    print(f"[APPROACH:RETURN][{ctx.name}] {POST_RETURN_BACKUP_DISTANCE:.2f}m 후진 + 180 회전(시계방향) 완료")
    print(f"[INFO][{ctx.name}] 트래시 미션 완료(파지+덤프+원위치복귀).")
    ctx.status = "트래시 완료 — IDLE 복귀"


# ════════════════════════════════════════════════════════════════════════════
#  K. 자원 상호배제 + 분사 서브미션 + task_select 최상위 디스패처
# ════════════════════════════════════════════════════════════════════════════
def g_with_resource(ctx, lock, resource_name, body_gen_fn):
    """공용 자원(쓰레기통/분사지점) 상호배제. 다른 로봇이 쥐고 있으면 대기, 자기 차례가 오면 획득 →
    body 실행 → (정상/조기종료 무관) 반드시 해제. 두 로봇이 동시에 같은 자원을 고르면 먼저 도착한
    쪽이 먼저 쓰고 나머지는 IDLE 대기처럼 순서를 기다린다."""
    while lock["holder"] not in (None, ctx.name):
        ctx.status = f"{resource_name} 자원 대기 중(사용중: {lock['holder']})"
        yield
        if not simulation_app.is_running():
            return
    lock["holder"] = ctx.name
    try:
        yield from body_gen_fn()
    finally:
        lock["holder"] = None


def g_nav_to_dock_approach(ctx, label="DOCK_APPROACH"):
    """거치대 팔 작업(파지/반납) 전 챠시를 자기 전용 거치대 근처(ctx.dock_approach_xy)로 필요할 때만
    nav-leg 이동시킨다(이미 위치·자세가 충분히 근접하면 생략 — 15_ 검증 패턴). [2026-07-26] 팔이 챠시
    뒤쪽에 달려있어(MOUNT_OFFSET) 후진으로 진입(reverse_entry=True)하면 팔↔노즐 거리가 짧아져 더 먼
    DOCK_STANDOFF 를 쓸 수 있음(사용자 제안) — 최종 파킹 방향은 거치대 반대쪽(ctx.dock_park_yaw)."""
    cur_xy = get_prim_world_position(ctx.articulation_root)[:2]
    cur_yaw = get_chassis_yaw(ctx.articulation_root)
    xy_close = float(np.linalg.norm(cur_xy - ctx.dock_approach_xy)) < DOCK_APPROACH_SKIP_XY_RADIUS
    yaw_close = abs(wrap_pi(cur_yaw - ctx.dock_park_yaw)) < DOCK_APPROACH_SKIP_YAW_TOL
    if xy_close and yaw_close:
        print(f"[NAV:{label}][{ctx.name}] 이미 거치대 근처(dist="
              f"{np.linalg.norm(cur_xy - ctx.dock_approach_xy):.3f}m) — nav-leg 생략, 바로 팔 작업")
        sync_rmpflow_base_pose(ctx)
        return

    dock_dir = np.array([np.cos(ctx.dock_approach_yaw), np.sin(ctx.dock_approach_yaw)])
    dock_standoff_xy = ctx.dock_approach_xy - dock_dir * FINAL_APPROACH_DISTANCE
    ctx.status = f"{label}: 거치대 근처로 이동"
    sync_rmpflow_base_pose(ctx)
    yield from g_stow_arm_for_nav(ctx)
    yield from g_run_nav_leg(ctx, dock_standoff_xy, ctx.dock_approach_yaw,
                             ctx.dock_approach_xy, ctx.dock_park_yaw, label, reverse_entry=True)
    if not simulation_app.is_running():
        return
    sync_rmpflow_base_pose(ctx)


def g_nav_to_home(ctx, label="RETURN_HOME"):
    """작업(trash/spray) 완료 후 원래 스폰지점(docking_station_1/02, ctx.home_xy/yaw)으로 복귀한다.
    노즐 거치대 파킹지점(ctx.dock_approach_xy)과는 다른 위치 — 최종 유휴 위치는 항상 원래
    docking_station 이어야 한다는 사용자 결정(2026-07-26)."""
    cur_xy = get_prim_world_position(ctx.articulation_root)[:2]
    cur_yaw = get_chassis_yaw(ctx.articulation_root)
    xy_close = float(np.linalg.norm(cur_xy - ctx.home_xy)) < DOCK_APPROACH_SKIP_XY_RADIUS
    yaw_close = abs(wrap_pi(cur_yaw - ctx.home_yaw)) < DOCK_APPROACH_SKIP_YAW_TOL
    if xy_close and yaw_close:
        print(f"[NAV:{label}][{ctx.name}] 이미 docking_station 근처 — nav-leg 생략")
        return

    home_dir = np.array([np.cos(ctx.home_yaw), np.sin(ctx.home_yaw)])
    home_standoff_xy = ctx.home_xy - home_dir * FINAL_APPROACH_DISTANCE
    ctx.status = f"{label}: docking_station 복귀"
    sync_rmpflow_base_pose(ctx)
    yield from g_stow_arm_for_nav(ctx)
    yield from g_run_nav_leg(ctx, home_standoff_xy, ctx.home_yaw, ctx.home_xy, ctx.home_yaw, label)
    if not simulation_app.is_running():
        return
    sync_rmpflow_base_pose(ctx)


def g_spray_mission_body(ctx):
    """분사 서브미션 : 노즐은 이미 파지된 상태로 시작(호출부 보장). 공용 분사 웨이포인트(ctx.spray_wp_xy)
    nav-leg → 스윕(WIPE/MOVE) → 자기 거치대 근처로 nav-leg 복귀. 반납은 호출부가 이어서 수행."""
    spray_dir = np.array([np.cos(ctx.spray_wp_yaw), np.sin(ctx.spray_wp_yaw)])
    spray_standoff_xy = ctx.spray_wp_xy - spray_dir * FINAL_APPROACH_DISTANCE
    sync_rmpflow_base_pose(ctx)
    ctx.status = "SPRAY: 웨이포인트로 이동"
    publish_hmi_state(ctx, "복도 진입")  # [HMI v2] SPRAY_STEPS
    yield from g_stow_arm_for_nav(ctx)
    yield from g_run_nav_leg(ctx, spray_standoff_xy, ctx.spray_wp_yaw, ctx.spray_wp_xy, ctx.spray_wp_yaw, "SPRAY_GOTO")
    if not simulation_app.is_running():
        return

    ctx.status = "SPRAY: 스윕 진행 중"
    publish_hmi_state(ctx, "소독 분사")  # [HMI v2]
    yield from g_spray_sweep(ctx, forward_distance=FORWARD_DISTANCE)
    if not simulation_app.is_running():
        return

    yield from g_nav_to_dock_approach(ctx, "SPRAY_RETURN")


def g_task_select_mission(ctx, trash_lock, spray_lock):
    """최상위 작업 선택 루프(15_ g_single_robot_mission 이식) — ctx.task_select_state["task"]
    ("trash"|"spray")를 기다렸다가 그에 맞는 서브미션 실행 후 다시 IDLE로 돌아간다.
    "trash"는 공용 쓰레기통, "spray"는 공용 분사지점을 쓰므로 각각 g_with_resource 로 감싸
    다른 로봇과 겹치면 순서를 기다린다(자기 전용 거치대 파지/반납 자체는 잠금 불필요)."""
    while simulation_app.is_running():
        ctx.status = "IDLE: task_select 대기"
        publish_hmi_state(ctx, "대기")  # [HMI v2]
        while ctx.task_select_state["task"] is None:
            yield
            if not simulation_app.is_running():
                return

        task = ctx.task_select_state["task"]
        ctx.task_select_state["task"] = None
        print(f"[MISSION][{ctx.name}] task_select 수신 = '{task}'")

        if task == "trash":
            if ctx.holding_nozzle:
                print(f"[MISSION][{ctx.name}] 노즐 보유 중 → 트래시 작업 전 거치대로 이동 후 반납")
                yield from g_nav_to_dock_approach(ctx, "TRASH_PRE_RETURN")
                yield from g_tool_change_release(ctx)
            yield from g_with_resource(ctx, trash_lock, "trash", lambda: g_trash_mission(ctx))
            publish_hmi_state(ctx, "복귀")  # [HMI v2] TRASH_STEPS/SPRAY_STEPS 공용 마지막 단계
            yield from g_nav_to_home(ctx, "TRASH_RETURN_HOME")
            print(f"[MISSION][{ctx.name}] 트래시 작업 완료 → IDLE 복귀")

        elif task == "spray":
            if not ctx.holding_nozzle:
                publish_hmi_state(ctx, "노즐 접촉")  # [HMI v2] SPRAY_STEPS
                yield from g_nav_to_dock_approach(ctx, "SPRAY_PRE_GRASP")
                ok = yield from g_tool_change_grasp(ctx)
                if not ok:
                    print(f"[MISSION][{ctx.name}][FAIL] 노즐 파지 실패 — 분사 작업 취소, IDLE 복귀")
                    continue
            yield from g_with_resource(ctx, spray_lock, "spray", lambda: g_spray_mission_body(ctx))
            yield from g_tool_change_release(ctx)
            publish_hmi_state(ctx, "복귀")  # [HMI v2]
            yield from g_nav_to_home(ctx, "SPRAY_RETURN_HOME")
            print(f"[MISSION][{ctx.name}] 분사 작업 완료 → IDLE 복귀")

        else:
            print(f"[MISSION][{ctx.name}][WARN] 알 수 없는 task '{task}' — 무시")


def g_return_trash_can(ctx):
    """[HMI v2] 도킹복귀 시 쓰레기통을 쥔 채(ctx.holding_trash=True) 중단됐으면 원위치
    (TRASH_SPAWN_FIXED)에 갖다 놓는다. g_trash_mission 의 PICK 접근·RETURN 내려놓기와 동일한
    빌딩블록(TARGET_JOINTS_DEG 고정 관절목표, _pick_closest_entry, g_move_to_pose 등)을 재사용
    하되, 원래 파지 시 실측했던 정밀 grasp_position 은 그 제너레이터가 버려지며 함께 사라졌으므로
    (g_trash_mission 은 도킹복귀에 의해 통째로 교체됨) 여기서 TARGET_JOINTS_DEG 도달 직후 다시
    실측한 tool0 pose 를 내려놓기 목표로 쓴다. 좌우보정(LATERAL_CORRECTION)은 원래 "쥐기 전에
    쓰레기통 실측 위치"와 비교하는 로직인데, 지금은 쓰레기통이 그리퍼에 붙어 함께 움직이는
    중이라 그 비교 자체가 무의미해 생략한다."""
    trash_xy = np.array(TRASH_SPAWN_FIXED)
    trash_origin_xy = trash_xy - TRASH_BBOX_CENTER_OFFSET_XY
    from_xy = get_prim_world_position(ctx.articulation_root)[:2]
    chassis_goal_xy, chassis_goal_yaw = _pick_closest_entry(trash_origin_xy, from_xy)
    approach_dir = rotate_2d(OFFSET_TRASH_FROM_CHASSIS / np.linalg.norm(OFFSET_TRASH_FROM_CHASSIS), chassis_goal_yaw)
    standoff_xy = chassis_goal_xy - approach_dir * FINAL_APPROACH_DISTANCE
    standoff_yaw = float(np.arctan2(approach_dir[1], approach_dir[0]))

    ctx.status = "TRASH_RECOVER: 쓰레기통 원위치 복귀 이동"
    sync_rmpflow_base_pose(ctx)
    yield from g_run_nav_leg(ctx, standoff_xy, standoff_yaw, chassis_goal_xy, chassis_goal_yaw, "TRASH_RECOVER")
    if not simulation_app.is_running():
        return
    sync_rmpflow_base_pose(ctx)

    ctx.status = "TRASH_RECOVER: 내려놓기"
    yield from g_ramp_to_joint_positions(ctx, TARGET_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    place_position = get_prim_world_position(ctx.tool0_path)
    place_orientation = get_world_orientation_wxyz(ctx.tool0_path)
    yield from g_move_to_pose(ctx, place_position, place_orientation, "원위치 내려놓기",
                              growing_tolerance_max=RETURN_PLACE_GROWING_TOLERANCE_MAX)
    yield from g_hold_pose(ctx, place_position, place_orientation, GRASP_HOLD_STEPS)
    ctx.gripper.open()
    ctx.holding_trash = False
    for _ in range(GRASP_HOLD_STEPS):
        yield
    print(f"[TRASH_RECOVER][{ctx.name}] Surface Gripper 개방 (is_closed={ctx.gripper.is_closed()})")
    retract_target = place_position + LIFT_OFFSET
    yield from g_move_to_pose(ctx, retract_target, place_orientation, "내려놓은 후 후퇴")
    yield from g_hold_pose(ctx, retract_target, place_orientation, GRASP_HOLD_STEPS)
    yield from g_stow_arm_for_nav(ctx)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    print(f"[TRASH_RECOVER][{ctx.name}] 쓰레기통 원위치 복귀 완료")


def g_return_dock_and_reset(ctx):
    """[HMI v2] 수동제어 종료(경로 끝) / '도킹 복귀' 버튼 → 진행 중이던 작업을 버리고 도킹스테이션
    (HOME)으로 복귀하면서 작업 상태를 초기화한다. 노즐을 쥐고 있으면 거치대에 반납, 쓰레기통을
    쥐고 있으면 원위치에 반납 후 복귀(깨끗한 초기화). 완료 후 호출부(main)가 정상 FSM(IDLE)으로
    되돌린다."""
    ctx.status = "DOCK_RETURN: 도킹 복귀 + 작업 초기화"
    publish_hmi_state(ctx, "복귀")
    ctx.cmd_pub.publish(Twist())          # 진행 중이던 스크립트 주행 즉시 정지
    if ctx.spray_fx is not None:
        ctx.spray_fx.clear()   # [HMI v2] 스윕 중 도킹복귀로 끊겼으면 뜬 파티클 정리
    yield from g_stow_arm_for_nav(ctx)
    if ctx.holding_nozzle:
        print(f"[DOCK_RETURN][{ctx.name}] 노즐 보유 중 → 거치대 반납 후 복귀")
        yield from g_nav_to_dock_approach(ctx, "DOCK_RETURN_RELEASE")
        yield from g_tool_change_release(ctx)
    if ctx.holding_trash:
        print(f"[DOCK_RETURN][{ctx.name}] 쓰레기통 보유 중 → 원위치 반납 후 복귀")
        yield from g_return_trash_can(ctx)
    yield from g_nav_to_home(ctx, "DOCK_RETURN_HOME")
    ctx.task_select_state["task"] = None  # 작업 초기화
    ctx.pick_state["start"] = False
    print(f"[DOCK_RETURN][{ctx.name}] 도킹 복귀 완료 → 작업 초기화, IDLE")


# ════════════════════════════════════════════════════════════════════════════
#  L. main — 두 로봇 협조 루프
# ════════════════════════════════════════════════════════════════════════════
def main():
    en_c1 = os.environ.get("ENABLE_C1", "1") == "1"
    en_c2 = os.environ.get("ENABLE_C2", "1") == "1"
    en_people = os.environ.get("ENABLE_PEOPLE", "1") == "1"   # ★YOLO 병합★ 사람 배치 토글
    print(f"[CFG] ENABLE_C1={en_c1} ENABLE_C2={en_c2} ENABLE_PEOPLE={en_people} "
          f"RS_ON={RS_ON} RS_PUBLISH_EVERY={RS_PUBLISH_EVERY} RENDER_EVERY={RENDER_EVERY} "
          f"CAM_DEACTIVATE_UNUSED={CAM_DEACTIVATE_UNUSED}")

    my_world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)
    stage = omni.usd.get_context().get_stage()

    build_env()
    if en_c1 and not build_carter1():
        simulation_app.close(); return
    if en_c1 and not build_nozzle_dock(NOZZLE_DOCK1_SCOPE, NOZZLE_TOOL_PATH_C1, NOZZLE_HOLD_JOINT_PATH_C1,
                                       NOZZLE_HOLD_ANCHOR_PATH_C1, NOZZLE_DOCK1_XY, NOZZLE_DOCK_HEIGHT, "carter1"):
        simulation_app.close(); return
    if en_c2 and not build_carter2():
        simulation_app.close(); return
    if en_c2 and not build_nozzle_dock(NOZZLE_DOCK2_SCOPE, NOZZLE_TOOL_PATH_C2, NOZZLE_HOLD_JOINT_PATH_C2,
                                       NOZZLE_HOLD_ANCHOR_PATH_C2, NOZZLE_DOCK2_XY, NOZZLE_DOCK_HEIGHT, "carter2"):
        simulation_app.close(); return
    if en_people:
        build_people()

    # ★YOLO 병합★ 카메라 prim 은 my_world.reset() 전에 존재해야 render product 가 붙는다(18_ 관례).
    c1_rs_path = build_realsense(C1_CHASSIS, C1_RS_PRIM, RS_OFFSET, RS_FOCAL, "carter1") if en_c1 else None
    c2_rs_path = build_realsense(C2_CHASSIS, C2_RS_PRIM, RS_OFFSET, RS_FOCAL, "carter2") if en_c2 else None

    # [2026-07-27 버그 수정] Camera() 래퍼 자체도 reset() 전에 만들어야 한다(14_1/18_ 이 실제로
    # 하던 순서) — 이 파일은 그동안 prim만 reset 전에 만들고 Camera(...) 생성은 reset 후로
    # 옮겨져 있었다. isaacsim Camera 래퍼는 생성 시점에 render product 를 현재 렌더 파이프라인
    # 상태에 등록하는데, reset() 이 그 상태를 갈아엎은 뒤 등록하면 render product 가 카메라와
    # 제대로 안 붙어 cam.get_rgb() 가 계속 None/구값만 반환할 수 있다(예외 없이 조용히 실패 —
    # 웹 비전 패널이 "YOLO 인식엔 문제 없어 보이는데 화면만 안 뜨는" 것처럼 보이는 근본 원인).
    if c1_rs_path is not None:
        try:
            setup_ros2_camera_publisher(c1_rs_path, C1_RS_TOPIC, C1_RS_FRAME_ID, RS_RESOLUTION, "/World/C1_CameraGraph")
        except Exception:
            print("[RS][carter1][WARN] OmniGraph Camera 생성 실패")
            
    if c2_rs_path is not None:
        try:
            setup_ros2_camera_publisher(c2_rs_path, C2_RS_TOPIC, C2_RS_FRAME_ID, RS_RESOLUTION, "/World/C2_CameraGraph")
        except Exception:
            print("[RS][carter2][WARN] OmniGraph Camera 생성 실패")

    c1_robot = c1_ee_path = c1_tool0_path = None
    if en_c1:
        setup_nozzle_surface_gripper(stage, C1_ARM_ROOT, C1_EE_LINK_NAME, C1_SURFACE_GRIPPER, "carter1")
        boost_drive_limits(C1_CARTER_PRIM)
        c1_robot, c1_ee_path, c1_tool0_path = build_robot_manipulator(
            my_world, C1_ARM_ROOT, C1_ARTICULATION_ROOT, C1_EE_LINK_NAME, "carter1")
    c2_robot = c2_ee_path = c2_tool0_path = None
    if en_c2:
        setup_nozzle_surface_gripper(stage, C2_ARM_ROOT, C2_EE_LINK_NAME, C2_SURFACE_GRIPPER, "carter2")
        boost_drive_limits(C2_SCOPE_PRIM)
        c2_robot, c2_ee_path, c2_tool0_path = build_robot_manipulator(
            my_world, C2_ARM_ROOT, C2_ARTICULATION_ROOT, C2_EE_LINK_NAME, "carter2")

    my_world.reset()
    for _ in range(5):
        my_world.step(render=False)



    rclpy.init()
    ros_node = rclpy.create_node("dual_task_select_yolo_controller")
    clock_pub = ros_node.create_publisher(Clock, "/clock", 10)     # 전역 단일 /clock

    # ── ★HMI 병합★ 웹 긴급정지(/robot/command 직접 구독) ──
    #   17_의 대칭 구조(양쪽 다 동일한 g_task_select_mission)라 18_의 "carter2 만 제너레이터 동결"
    #   방식은 못 씀 — trash_lock/spray_lock 을 쥔 채 얼리면 락업 위험. 대신 person-gate 와 동일한
    #   "제자리 정지" 메커니즘을 공유(g_drive_straight_open_loop/g_rotate_in_place/g_spray_sweep
    #   안에 이미 삽입됨) — 제너레이터는 계속 전진시키고 내부에서 스스로 멈춘다.
    #   HMI robotId 매핑 : disinfect→carter1, waste→carter2(하위호환) + carter1/carter2 직접 지정
    #   (19_에서는 이 직접 매핑이 주 경로 — 역할고정이 아니므로 disinfect/waste 고정 매핑은 의미가
    #   약해짐, 향후 HMI UI 갱신 시 carter1/carter2 직접 지정으로 전환 권장·이 파일 범위 밖).
    estop_flags = {"carter1": False, "carter2": False}
    # ★HMI v2 수동제어/도킹복귀★ : main 루프가 이 값을 보고 해당 로봇의 제너레이터를 즉시 교체한다.
    #   "manual" = 작업 즉시 중단 후 IDLE(비켜서기) → 운영자가 /carterN/goal_pose(Nav2)로 직접 주행.
    #   "dock"   = 작업 중단 + 도킹스테이션 복귀 + 작업 초기화(g_return_dock_and_reset).
    override_cmd = {"carter1": None, "carter2": None}

    def _on_hmi_command(msg):
        try:
            body = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError, ValueError):
            return
        cmd = body.get("command")
        rid = body.get("robotId")
        id_map = {"disinfect": ["carter1"], "waste": ["carter2"],
                  "carter1": ["carter1"], "carter2": ["carter2"],
                  None: ["carter1", "carter2"]}
        targets = id_map.get(rid, [])
        ctx_by_name = {"carter1": c1_ctx, "carter2": c2_ctx}  # 호출 시점 최신값(클로저, c1_ctx/c2_ctx 는 아래서 나중에 대입돼도 OK)
        if cmd == "EMERGENCY_STOP":
            for t in targets:
                estop_flags[t] = True
                tctx = ctx_by_name.get(t)
                if tctx is not None:
                    # [HMI v2] 웹이 "지금 긴급정지 중"을 구분할 수 있는 유일한 신호 — 이게 없으면
                    # process_state 는 정지 직전 라벨에 그대로 멈춰있어 프론트가 estop 상태를 못 봄
                    # (수동제어 잠금해제 조건에 필요, MapPanel.tsx 참고).
                    publish_hmi_state(tctx, "긴급정지")
                    if tctx.spray_fx is not None:
                        tctx.spray_fx.clear()   # [HMI v2] 분사 중 긴급정지 시 뜬 파티클 정리(보험 — g_spray_sweep 자체 체크도 있음)
            print(f"[ESTOP] 긴급정지 수신 → 정지 {targets}")
        elif cmd == "START":
            for t in targets:
                estop_flags[t] = False
            print(f"[ESTOP] START 수신 → 해제 {targets}")
        elif cmd == "MANUAL_OVERRIDE":
            for t in targets:
                override_cmd[t] = "manual"
            print(f"[OVERRIDE] 수동제어 → 작업 즉시 중단 {targets}")
        elif cmd == "DOCK_RETURN":
            for t in targets:
                override_cmd[t] = "dock"
                # ★도킹 복귀 = 언제든지·최우선★ : 긴급정지 중이어도 즉시 풀어야 g_run_nav_leg 의
                # estop 대기 루프(정지된 채 goal 재발행도 안 함)에 걸려 멈춰있지 않고 바로 주행한다.
                # (사람 회피 등 물리 충돌방지는 Nav2 costmap/PersonGate 가 estop_flags 와 무관하게
                # 별도로 계속 작동하므로 안전 자체가 사라지는 건 아님.)
                estop_flags[t] = False
            print(f"[OVERRIDE] 도킹 복귀(작업 초기화) 요청 {targets} (긴급정지 해제 겸함)")

    ros_node.create_subscription(String, "/robot/command", _on_hmi_command, 10)
    print("[ROS] /robot/command 구독(긴급정지 인지) — disinfect→carter1, waste→carter2, carter1/carter2 직접")

    # ★YOLO 병합★ c1+c2 둘 다 사람 회피(19_: 18_의 c1 전용 제한 해제, 사용자 결정).
    c1_gate = PersonGate(ros_node, C1_PERSON_ALERT, my_world, "carter1") if en_c1 else None
    c2_gate = PersonGate(ros_node, C2_PERSON_ALERT, my_world, "carter2") if en_c2 else None



    trash_lock = {"holder": None}
    spray_lock = {"holder": None}

    c1_ctx = None; c1_cmd_pub = None; c1_gen = None; c1_done = not en_c1
    c2_ctx = None; c2_cmd_pub = None; c2_gen = None; c2_done = not en_c2
    c1_in_override = False; c2_in_override = False   # 도킹복귀 제너레이터 실행 중 여부(끝나면 정상 FSM 복귀)

    try:
        if en_c1:
            c1_robot.initialize()
            c1_dof = list(c1_robot.dof_names)
            dp = c1_robot.get_joint_positions()
            # [2026-07-26 사용자 요청] 스폰 직후 팔 초기자세를 STOW_Q 대신 실제 주행에 쓰는
            # NAV_STOW_Q_DEG(무게중심 낮춤+챠시 근접, g_stow_arm_for_nav 와 동일)로 통일.
            for i, name in enumerate(ARM_JOINT_NAMES):
                if name in c1_dof:
                    dp[c1_dof.index(name)] = float(np.radians(NAV_STOW_Q_DEG[i]))
            c1_robot.set_joint_positions(dp)
            for _ in range(10):
                my_world.step(render=True)
            c1_rmpflow, c1_gripper, c1_tool_changer = build_robot_rmpflow_gripper_toolchanger(
                stage, C1_ARM_ROOT, C1_GROUND_PLANE, c1_ee_path, C1_SURFACE_GRIPPER,
                NOZZLE_DOCK1_XY, NOZZLE_DOCK_HEIGHT, NOZZLE_TOOL_PATH_C1, c1_robot, "carter1")

            c1_cmd_pub = ros_node.create_publisher(Twist, C1_CMD_VEL, 10)
            c1_goal_pub = ros_node.create_publisher(PoseStamped, C1_NAV_GOAL, 10)
            c1_pick_state = {"start": False}
            ros_node.create_subscription(Bool, C1_START_PICK,
                                         lambda m: c1_pick_state.__setitem__("start", bool(m.data)), 10)
            c1_task_state = {"task": None}
            ros_node.create_subscription(String, C1_TASK_SELECT,
                                         lambda m: c1_task_state.__setitem__("task", str(m.data).strip().lower()), 10)
            print(f"[ROS] carter1 : pub {C1_NAV_GOAL} + {C1_CMD_VEL}, sub {C1_START_PICK} + {C1_TASK_SELECT}")

            c1_ctx = RobotCtx("carter1", my_world, c1_robot, c1_rmpflow, c1_dof, c1_tool0_path, c1_ee_path,
                              c1_gripper, C1_ARM_ROOT, C1_ARTICULATION_ROOT, ros_node, c1_goal_pub, c1_cmd_pub,
                              c1_pick_state, c1_task_state, NOZZLE_DOCK1_XY, NOZZLE_DOCK_HEIGHT,
                              DOCK1_APPROACH_XY, DOCK1_APPROACH_YAW, NOZZLE_TOOL_PATH_C1, NOZZLE_TCP_PATH_C1,
                              NOZZLE_HOLD_JOINT_PATH_C1, NOZZLE_HOLD_ANCHOR_PATH_C1, SPRAY_WP1_XY, SPRAY_WP1_YAW,
                              HOME1_XY, HOME1_YAW, DOCK1_PARK_YAW)
            c1_ctx.tool_changer = c1_tool_changer
            c1_ctx.person_gate = c1_gate      # ★YOLO 병합★
            c1_ctx.estop_flags = estop_flags  # ★HMI 병합★ (carter1/carter2 공유 dict)
            c1_ctx.hmi_state_pub = ros_node.create_publisher(String, C1_HMI_STATE, 10)  # [HMI v2]
            print(f"[ROS] carter1 : pub {C1_HMI_STATE} (HMI process_state, 직접 발행)")
            _selftest_c1 = os.environ.get("SELFTEST_TASK_C1", "").strip().lower()
            if _selftest_c1 in ("spray", "trash"):
                c1_task_state["task"] = _selftest_c1
                print(f"[SELFTEST] carter1 task_select 자동 트리거 = '{_selftest_c1}'")
            c1_gen = g_task_select_mission(c1_ctx, trash_lock, spray_lock)

        if en_c2:
            c2_robot.initialize()
            c2_dof = list(c2_robot.dof_names)
            dp = c2_robot.get_joint_positions()
            # [2026-07-26 사용자 요청] 스폰 직후 팔 초기자세를 전부-0 대신 실제 주행에 쓰는
            # NAV_STOW_Q_DEG로 통일(carter1 과 동일 값).
            for i, name in enumerate(ARM_JOINT_NAMES):
                if name in c2_dof:
                    dp[c2_dof.index(name)] = float(np.radians(NAV_STOW_Q_DEG[i]))
            c2_robot.set_joint_positions(dp)
            for _ in range(10):
                my_world.step(render=True)
            c2_rmpflow, c2_gripper, c2_tool_changer = build_robot_rmpflow_gripper_toolchanger(
                stage, C2_ARM_ROOT, C2_GROUND_PLANE, c2_ee_path, C2_SURFACE_GRIPPER,
                NOZZLE_DOCK2_XY, NOZZLE_DOCK_HEIGHT, NOZZLE_TOOL_PATH_C2, c2_robot, "carter2")

            c2_cmd_pub = ros_node.create_publisher(Twist, C2_CMD_VEL, 10)
            c2_goal_pub = ros_node.create_publisher(PoseStamped, C2_NAV_GOAL, 10)
            c2_pick_state = {"start": False}
            ros_node.create_subscription(Bool, C2_START_PICK,
                                         lambda m: c2_pick_state.__setitem__("start", bool(m.data)), 10)
            c2_task_state = {"task": None}
            ros_node.create_subscription(String, C2_TASK_SELECT,
                                         lambda m: c2_task_state.__setitem__("task", str(m.data).strip().lower()), 10)
            print(f"[ROS] carter2 : pub {C2_NAV_GOAL} + {C2_CMD_VEL}, sub {C2_START_PICK} + {C2_TASK_SELECT}")

            c2_ctx = RobotCtx("carter2", my_world, c2_robot, c2_rmpflow, c2_dof, c2_tool0_path, c2_ee_path,
                              c2_gripper, C2_ARM_ROOT, C2_ARTICULATION_ROOT, ros_node, c2_goal_pub, c2_cmd_pub,
                              c2_pick_state, c2_task_state, NOZZLE_DOCK2_XY, NOZZLE_DOCK_HEIGHT,
                              DOCK2_APPROACH_XY, DOCK2_APPROACH_YAW, NOZZLE_TOOL_PATH_C2, NOZZLE_TCP_PATH_C2,
                              NOZZLE_HOLD_JOINT_PATH_C2, NOZZLE_HOLD_ANCHOR_PATH_C2, SPRAY_WP1_XY, SPRAY_WP1_YAW,
                              HOME2_XY, HOME2_YAW, DOCK2_PARK_YAW)
            c2_ctx.tool_changer = c2_tool_changer
            c2_ctx.person_gate = c2_gate      # ★YOLO 병합★
            c2_ctx.estop_flags = estop_flags  # ★HMI 병합★ (carter1/carter2 공유 dict)
            c2_ctx.hmi_state_pub = ros_node.create_publisher(String, C2_HMI_STATE, 10)  # [HMI v2]
            print(f"[ROS] carter2 : pub {C2_HMI_STATE} (HMI process_state, 직접 발행)")
            _selftest_c2 = os.environ.get("SELFTEST_TASK_C2", "").strip().lower()
            if _selftest_c2 in ("spray", "trash"):
                c2_task_state["task"] = _selftest_c2
                print(f"[SELFTEST] carter2 task_select 자동 트리거 = '{_selftest_c2}'")
            c2_gen = g_task_select_mission(c2_ctx, trash_lock, spray_lock)

        print("\n[RUN] Play ▶ : carter1 + carter2 각자 task_select 대기 중.\n"
              "      트리거: ros2 topic pub /carter1/task_select std_msgs/msg/String \"data: 'spray'\" --once\n"
              "              ros2 topic pub /carter2/task_select std_msgs/msg/String \"data: 'trash'\" --once\n"
              "      (또는 SELFTEST_TASK_C1/SELFTEST_TASK_C2 env var 로 헤드리스 자동 트리거)\n")

        hb = 0
        step_i = 0
        while simulation_app.is_running():
            step_i += 1
            my_world.step(render=(step_i % RENDER_EVERY == 0))

            t = float(my_world.current_time)
            cmsg = Clock(); cmsg.clock.sec = int(t); cmsg.clock.nanosec = int(round((t - int(t)) * 1e9))
            clock_pub.publish(cmsg)
            rclpy.spin_once(ros_node, timeout_sec=0.0)



            if not my_world.is_playing():
                continue

            if not c1_done:
                # ★수동제어/도킹복귀★ : 명령 오면 진행 중 작업 제너레이터를 즉시 교체(작업 중단).
                if override_cmd["carter1"] is not None:
                    mode = override_cmd["carter1"]; override_cmd["carter1"] = None
                    c1_ctx.task_select_state["task"] = None; c1_ctx.pick_state["start"] = False
                    if c1_cmd_pub is not None:
                        c1_cmd_pub.publish(Twist())
                    if mode == "dock":
                        c1_gen = g_return_dock_and_reset(c1_ctx); c1_in_override = True
                    else:  # "manual" : 작업만 중단하고 IDLE(비켜서기) → Nav2 가 수동 goal 주행
                        c1_gen = g_task_select_mission(c1_ctx, trash_lock, spray_lock); c1_in_override = False
                    print(f"[OVERRIDE][carter1] {mode} → 작업 중단, 제너레이터 교체")
                try:
                    next(c1_gen)
                except StopIteration:
                    if c1_in_override:                     # 도킹복귀 끝 → 정상 FSM(IDLE) 재개
                        c1_in_override = False
                        c1_gen = g_task_select_mission(c1_ctx, trash_lock, spray_lock)
                    else:
                        c1_done = True
                        print("[C1] 미션 제너레이터 종료(비정상 — 통상 IDLE 유휴 루프라 안 끝남)")

            if not c2_done:
                if override_cmd["carter2"] is not None:
                    mode = override_cmd["carter2"]; override_cmd["carter2"] = None
                    c2_ctx.task_select_state["task"] = None; c2_ctx.pick_state["start"] = False
                    if c2_cmd_pub is not None:
                        c2_cmd_pub.publish(Twist())
                    if mode == "dock":
                        c2_gen = g_return_dock_and_reset(c2_ctx); c2_in_override = True
                    else:
                        c2_gen = g_task_select_mission(c2_ctx, trash_lock, spray_lock); c2_in_override = False
                    print(f"[OVERRIDE][carter2] {mode} → 작업 중단, 제너레이터 교체")
                try:
                    next(c2_gen)
                except StopIteration:
                    if c2_in_override:
                        c2_in_override = False
                        c2_gen = g_task_select_mission(c2_ctx, trash_lock, spray_lock)
                    else:
                        c2_done = True
                        print("[C2] 미션 제너레이터 종료(비정상 — 통상 IDLE 유휴 루프라 안 끝남)")

            hb += 1
            if hb % 300 == 0:
                c1s = "(비활성 ENABLE_C1=0)" if not en_c1 else ("완료" if c1_done else c1_ctx.status)
                c2s = "(비활성 ENABLE_C2=0)" if not en_c2 else ("완료" if c2_done else c2_ctx.status)
                g1 = c1_gate.blocked() if c1_gate is not None else False
                g2 = c2_gate.blocked() if c2_gate is not None else False
                print(f"[HB] carter1 = {c1s} (gate={g1}, estop={estop_flags['carter1']})\n"
                      f"     carter2 = {c2s} (gate={g2}, estop={estop_flags['carter2']})")

    except Exception:
        import traceback
        print("\n[FATAL] main 루프 예외 — 아래 파이썬 트레이스백이 진짜 원인입니다:\n")
        traceback.print_exc()
    finally:
        try:
            if c1_cmd_pub is not None:
                c1_cmd_pub.publish(Twist())
            if c2_cmd_pub is not None:
                c2_cmd_pub.publish(Twist())
        except Exception:
            pass
        try:
            omni.timeline.get_timeline_interface().stop()
        except Exception:
            pass
        try:
            ros_node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        simulation_app.close()


if __name__ == "__main__":
    main()
