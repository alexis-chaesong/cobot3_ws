"""
17_dual_task_select_tool_changer_integrated.py  ★로봇 2대 모두 작업 선택식★
================================================================================
16_dual_sg_tool_changer_integrated.py 는 carter1=소독 전담 / carter2=폐기물 전담으로 역할이
고정돼 있고, 웹 HMI(disinfect=carter1/waste=carter2 고정 매핑)와 이미 실연동돼 있다.
15_single_robot_tool_changer_integrated.py 는 carter2 한 대만으로 "/carter2/task_select
(String, trash|spray) 로 작업을 선택하면 그 작업에 맞는 도구를 장착하고 그 작업만 수행"하는
구조를 검증했다.

이 파일 = 그 "작업 선택식" 구조를 로봇 2대(carter1, carter2) 모두에 적용한 것. 각 로봇이 각자
/carter1/task_select, /carter2/task_select 로 독립적으로 "trash"|"spray" 를 선택할 수 있다.

★범위(사용자 결정)★ : 이번 작업은 Isaac 스크립트만. ROS 미션노드 교체(spray_waypoint_mission
폐기, trash_can_nav_pick_mission 류 제너릭 포워더로 통일)와 웹 HMI(로봇별 작업선택 UI) 연동은
범위 밖 — 16번 파일(HMI 연동판)은 그대로 두고 이 파일을 병행 추가한다. 트리거는 ros2 topic pub
또는 SELFTEST_TASK_C1/SELFTEST_TASK_C2 env var 로 한다(15_의 SELFTEST_TASK 패턴을 로봇별 복제).

★자원 배치(사용자 결정)★ :
  · 쓰레기통 1개만(공용) : carter2에 내장된 것(move_tash_can.usd 의 small_trash_can_body)을 그대로
    쓰고, carter1 내장 쓰레기통은 16번처럼 계속 SetActive(False). 두 로봇이 동시에 "trash"를
    고르면 상호배제(trash_lock)로 순번 대기.
  · 노즐 거치대는 로봇별 전용 2개 : 각자 스폰 위치의 -Y 방향으로 0.35m(팔 접근거리, 15_/16_
    검증값) 떨어진 곳에 배치 — carter1 은 기존 +X 오프셋에서 -Y로 변경(두 로봇 배치 통일).
  · 분사 벽 웨이포인트(SPRAY_WP1)는 새 좌표를 추측하지 않고 기존 검증값 하나만 공용 재사용 —
    쓰레기통과 같은 이유로 spray_lock 상호배제 적용(라이브 실측 후 로봇별로 분리할 수 있음, 후속).

★알려진 블로커(코드는 안 고침, 문서만)★ : trash_can_nav_pick_mission.py 의 CARTER_START_POSE 가
carter2 스폰 좌표로 하드코딩돼 있어, namespace:=carter1 인스턴스를 그대로 띄우면 AMCL 초기위치가
틀어진다 — 라이브에서 carter1 trash 작업을 검증하려면 이 노드를 먼저 네임스페이스별 시작pose를
받도록 고쳐야 함(다음 단계).

⚠ 라이브 검증 필요(오프라인 헤드리스만 확인) : 노즐 거치대 좌표(-Y 오프셋)와 분사 웨이포인트
상호배제는 설계 판단이라 Nav2/IK 도달성 실측이 필요하다(15_/16_ 파일들과 동일 컨벤션).

실행(총 5개 터미널 권장, ROS 사이드는 이번 범위 밖이라 구성만 기재) :
  1) 이 스크립트 : python.sh isaacpjt/M0609/17_dual_task_select_tool_changer_integrated.py
  2) Nav2(멀티, carter1/carter2 네임스페이스)
  3) trash_can_nav_pick_mission --ros-args -p namespace:=carter1  (★AMCL 시작pose 블로커 참고★)
  4) trash_can_nav_pick_mission --ros-args -p namespace:=carter2
  5) 트리거 : ros2 topic pub /carter1/task_select std_msgs/msg/String "data: 'spray'" --once
              ros2 topic pub /carter2/task_select std_msgs/msg/String "data: 'trash'" --once
================================================================================
"""
import os

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
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, Vt

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
import isaacsim.robot_motion.motion_generation as mg

import rclpy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped
from rosgraph_msgs.msg import Clock

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
_CARTER_NAV_WS = Path(os.environ.get(
    "CARTER_NAV_WS", str(Path.home() / "IsaacSim-ros_workspaces" / "humble_ws")
))
HOSPITAL_USD = str(_CARTER_NAV_WS / "src" / "navigation" / "carter_navigation"
                    / "maps" / "map" / "modified_hospital.usd")
NOZZLE_USD = str(_WS_ROOT / "src" / "integration" / "integration" / "m0609_with_nozzle.usd")
MOVE_TRASH_USD = str(_WS_ROOT / "src" / "assets" / "scenes" / "move_tash_can.usd")

PHYSICS_DT = 1.0 / 60.0
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]

RENDER_EVERY = 2
CAM_RENDER_W = 640
CAM_RENDER_H = 400

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

C1_START_POSE = dict(x=18.5, y=0.0, z=0.05, yaw_deg=0.0)     # docking_station_1


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

C2_START_POSE = dict(x=16.66290495232035, y=-0.0029517927591273807, yaw_deg=0.0)  # docking_station_02

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
DUMP_RAMP_STEPS = 90
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

DOCK_NUDGE_DISTANCE = 0.10
DOCK_NUDGE_SPEED = 0.10
DOCK_ONESHOT_MAX_ITERS = 3
DOCK_ONESHOT_POS_TOL = 0.02
DOCK_ONESHOT_DRIVE_SPEED = 0.08
DOCK_ONESHOT_SETTLE_STEPS = 20

DOCK_APPROACH_SKIP_XY_RADIUS = 0.15    # 이 안이면 "이미 거치대 근처"로 보고 nav-leg 생략
DOCK_APPROACH_SKIP_YAW_TOL = np.radians(15.0)

# ── 노즐 거치대(로봇별 2개) ──
NOZZLE_SOURCE_PRIMPATH = "/World/m0609/nozzle_base_link"   # tool0_to_nozzle 조인트 밖(형제) — 안 딸려옴
NOZZLE_DOCK_HEIGHT = 0.65
DOCK_STANDOFF = 0.35        # 챠시 파킹지점-거치대 팔 접근거리(15_/16_ 검증값, 절대 늘리지 않음 —
                            # IK grasp reach 가 이 거리 기준으로 튜닝됨). 접근방향은 -Y(사용자 결정).
# [사용자 요청] 거치대가 스폰 바로 옆(0.35m)이라 너무 가까워 보임 + Nav2 실주행 테스트를 위해서도
# "스폰↔거치대"가 제자리 회전만으로 끝나면 의미가 없다 — 그래서 거치대 자체는 스폰에서
# DOCK_HOME_DISTANCE 만큼 멀리 두고, 파킹지점(DOCKn_APPROACH_XY)만 거치대에서 DOCK_STANDOFF(검증된
# IK reach 거리, 불변) 만큼 못미친 곳에 둔다 — nav-leg 가 스폰→파킹지점 구간을 실제로 주행하게 됨.
DOCK_HOME_DISTANCE = 1.5    # 스폰-거치대 거리[m]. ⚠ 라이브 미검증(주변 벽/장애물과 안 부딪히는지 확인 필요).

NOZZLE_DOCK1_SCOPE = "/World/NozzleDock1"
NOZZLE_TOOL_PATH_C1 = f"{NOZZLE_DOCK1_SCOPE}/nozzle_tool"
NOZZLE_TCP_PATH_C1 = f"{NOZZLE_TOOL_PATH_C1}/nozzle_tcp"
NOZZLE_HOLD_JOINT_PATH_C1 = f"{NOZZLE_DOCK1_SCOPE}/hold_joint"
NOZZLE_HOLD_ANCHOR_PATH_C1 = f"{NOZZLE_DOCK1_SCOPE}/hold_anchor"
NOZZLE_DOCK1_XY = np.array([C1_START_POSE["x"], C1_START_POSE["y"] - DOCK_HOME_DISTANCE])
DOCK1_APPROACH_XY = NOZZLE_DOCK1_XY + np.array([0.0, DOCK_STANDOFF])
DOCK1_APPROACH_YAW = -np.pi / 2.0     # 남쪽(-Y)을 보고 거치대 접근

NOZZLE_DOCK2_SCOPE = "/World/NozzleDock2"
NOZZLE_TOOL_PATH_C2 = f"{NOZZLE_DOCK2_SCOPE}/nozzle_tool"
NOZZLE_TCP_PATH_C2 = f"{NOZZLE_TOOL_PATH_C2}/nozzle_tcp"
NOZZLE_HOLD_JOINT_PATH_C2 = f"{NOZZLE_DOCK2_SCOPE}/hold_joint"
NOZZLE_HOLD_ANCHOR_PATH_C2 = f"{NOZZLE_DOCK2_SCOPE}/hold_anchor"
NOZZLE_DOCK2_XY = np.array([C2_START_POSE["x"], C2_START_POSE["y"] - DOCK_HOME_DISTANCE])
DOCK2_APPROACH_XY = NOZZLE_DOCK2_XY + np.array([0.0, DOCK_STANDOFF])
DOCK2_APPROACH_YAW = -np.pi / 2.0

# Surface Gripper 튜닝(15_/16_ 검증값 — grip_travel 을 IK 접근오차보다 넉넉히, clearance 는 최소).
MAX_GRIP_DISTANCE = 0.04
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
TC_TARGET_BIAS_COMPENSATION = np.array([0.006, -0.0003, 0.0])   # URDF-USD 형상 불일치 고정 바이어스

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
FORWARD_DISTANCE = 10.5
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

C2_TASK_SELECT = f"/{NS_CARTER2}/task_select"
C2_NAV_GOAL = f"/{NS_CARTER2}/trash_can_nav_goal"
C2_START_PICK = f"/{NS_CARTER2}/start_pick"
C2_CMD_VEL = f"/{NS_CARTER2}/cmd_vel"


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


def set_camera_resolution(root_path, width, height):
    if not width or not height:
        return 0
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    n = 0
    for prim in Usd.PrimRange(root):
        nm = prim.GetName()
        is_cam_rp = ("camera_render_product" in nm) or (nm == "isaac_create_render_product")
        if not is_cam_rp:
            continue
        wa = prim.GetAttribute("inputs:width"); ha = prim.GetAttribute("inputs:height")
        if wa and wa.IsValid() and ha and ha.IsValid():
            wa.Set(int(width)); ha.Set(int(height)); n += 1
    print(f"[GPU] {root_path} 카메라 render product {n}개 → {width}x{height} 로 축소")
    return n


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
    set_camera_resolution(C1_CARTER_PRIM, CAM_RENDER_W, CAM_RENDER_H)
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
    set_camera_resolution(C2_SCOPE_PRIM, CAM_RENDER_W, CAM_RENDER_H)
    print("[SCENE] carter2 (Nova + Surface Gripper 팔 + 공용 쓰레기통) 완료")
    return True


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
                 spray_wp_xy, spray_wp_yaw):
        self.name = name
        self.world = world; self.robot = robot; self.rmpflow = rmpflow
        self.dof_names = dof_names; self.tool0_path = tool0_path; self.ee_path = ee_path
        self.gripper = gripper
        self.arm_root = arm_root; self.articulation_root = articulation_root
        self.ros_node = ros_node; self.nav_goal_pub = nav_goal_pub; self.cmd_pub = cmd_pub
        self.pick_state = pick_state; self.task_select_state = task_select_state
        self.dock_xy = dock_xy; self.dock_height = dock_height
        self.dock_approach_xy = dock_approach_xy; self.dock_approach_yaw = dock_approach_yaw
        self.tool_path = tool_path; self.tcp_path = tcp_path
        self.hold_joint_path = hold_joint_path; self.hold_anchor_path = hold_anchor_path
        self.spray_wp_xy = spray_wp_xy; self.spray_wp_yaw = spray_wp_yaw
        self.stage = omni.usd.get_context().get_stage()
        self.status = "시작 대기"
        self.tool_changer = None            # ToolChangerController, main() 이 주입
        self.holding_nozzle = False
        self.nozzle_tip_offset = None       # link_6 기준 nozzle_tcp 상대위치(3축) — 파지 직후 실측


def sync_rmpflow_base_pose(ctx):
    base_prim = ctx.stage.GetPrimAtPath(f"{ctx.arm_root}/base_link")
    m = omni.usd.get_world_transform_matrix(base_prim)
    tr = m.ExtractTranslation(); q = m.ExtractRotationQuat(); im = q.GetImaginary()
    pos = np.array([tr[0], tr[1], tr[2]])
    ori = np.array([q.GetReal(), im[0], im[1], im[2]])
    ctx.rmpflow.rmp_flow.set_robot_base_pose(robot_position=pos, robot_orientation=ori)


def g_ramp_to_joint_positions(ctx, target_joints_deg, ramp_steps):
    start = ctx.robot.get_joint_positions().copy()
    target = start.copy()
    arm_idx = [ctx.dof_names.index(n) for n in ARM_JOINT_NAMES if n in ctx.dof_names]
    for idx, rad in zip(arm_idx, np.radians(target_joints_deg)):
        target[idx] = rad
    for step in range(ramp_steps):
        yield
        alpha = (step + 1) / ramp_steps
        wp = start + _smoothstep(alpha) * (target - start)
        ctx.robot.apply_action(ArticulationAction(joint_positions=wp))


def g_ramp_ee_target(ctx, target_position, target_orientation, ramp_steps):
    start = get_prim_world_position(ctx.tool0_path)
    for step in range(ramp_steps):
        yield
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


def g_rotate_in_place(ctx, target_yaw, kp, kd, w_max, max_w_step, tol, chassis_path, max_steps=600):
    prev_yaw = get_chassis_yaw(chassis_path); w_applied = 0.0; settled = 0
    for _ in range(max_steps):
        yield
        yaw = get_chassis_yaw(chassis_path)
        yaw_err = wrap_pi(target_yaw - yaw)
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


def g_run_nav_leg(ctx, standoff_xy, standoff_yaw, chassis_goal_xy, chassis_goal_yaw, label):
    """Nav2 목표(standoff) 발행 → /start_pick 대기 → 실제 위치 기준 회전→직진→회전."""
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
    yield from g_rotate_in_place(ctx, entry_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, ctx.articulation_root)
    yield from g_drive_straight_open_loop(ctx, entry_distance + PRE_ROTATE_NUDGE_DISTANCE, ctx.articulation_root)
    yield from g_rotate_in_place(ctx, chassis_goal_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, ctx.articulation_root)
    if FINAL_NUDGE_DISTANCE > 0.0:
        yield from g_drive_straight_open_loop(ctx, FINAL_NUDGE_DISTANCE, ctx.articulation_root)


def g_caster_nudge(ctx, chassis_path, distance=DOCK_NUDGE_DISTANCE):
    """짧게 전진→후진(같은 거리)해 캐스터 바퀴를 현재 주행축 방향으로 강제 정렬한 뒤 정밀 접근 시작."""
    yield from g_drive_straight_open_loop(ctx, distance, chassis_path, speed=DOCK_NUDGE_SPEED)
    yield from g_drive_straight_open_loop(ctx, distance, chassis_path, speed=DOCK_NUDGE_SPEED, reverse=True)


def g_precise_dock_approach(ctx, goal_xy, goal_yaw, chassis_path, label="DOCK"):
    """Nudge(캐스터 정렬) + One-Shot(정지→1회 측정→계산된 만큼만 이동, 이동 중 재측정 안 함)
    최대 DOCK_ONESHOT_MAX_ITERS 회 반복 후 그 자리에서 최종 회전 정렬(15_/16_ 검증 패턴)."""
    yield from g_caster_nudge(ctx, chassis_path)

    for it in range(DOCK_ONESHOT_MAX_ITERS):
        ctx.cmd_pub.publish(Twist())
        for _ in range(DOCK_ONESHOT_SETTLE_STEPS):
            yield
        pos_xy = get_prim_world_position(chassis_path)[:2]
        to_goal = goal_xy - pos_xy
        dist = float(np.linalg.norm(to_goal))
        if dist < DOCK_ONESHOT_POS_TOL:
            break
        bearing = float(np.arctan2(to_goal[1], to_goal[0]))
        print(f"[DOCK:{label}][{ctx.name}] One-Shot 스캔 #{it + 1}: dist={dist:.3f}m "
              f"bearing={np.degrees(bearing):.1f}도(이동 중 재측정 없음)")
        yield from g_rotate_in_place(ctx, bearing, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                     FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, chassis_path)
        yield from g_drive_straight_open_loop(ctx, dist, chassis_path, speed=DOCK_ONESHOT_DRIVE_SPEED)
    else:
        print(f"[DOCK:{label}][{ctx.name}][WARN] One-Shot {DOCK_ONESHOT_MAX_ITERS}회 반복 후에도 "
              f"위치 허용치({DOCK_ONESHOT_POS_TOL}m) 미달 — 마지막 측정값으로 자세 정렬만 진행")

    yield from g_rotate_in_place(ctx, goal_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, chassis_path)

    final_pos = get_prim_world_position(chassis_path)[:2]
    final_yaw = get_chassis_yaw(chassis_path)
    print(f"[DOCK:{label}][{ctx.name}] 정밀 도킹 완료 pos={np.round(final_pos, 3).tolist()}"
          f"(목표{goal_xy.tolist()}) yaw={np.degrees(final_yaw):.1f}도(목표{np.degrees(goal_yaw):.1f}도)")


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
                       bias_compensation=None):
    """거치대는 고정 위치라 반응형 IK(RMPflow) 대신 "IK 1회 풀어 관절각 확정 → 램프"
    (15_ 실측: 반응형 IK 는 오래 돌수록 오히려 발산). position_tolerance=0.001 +
    TC_TARGET_BIAS_COMPENSATION(URDF-USD 형상 불일치 고정 바이어스 보정, TC_APPROACH_ORIENTATION
    접근에만 유효). 반환 = (관절각 rad 6dof|None, ok)."""
    bias = bias_compensation if bias_compensation is not None else TC_TARGET_BIAS_COMPENSATION
    corrected_target = np.asarray(target_pos) - bias
    q, ok = ik.compute_inverse_kinematics(
        EE_FRAME, corrected_target, target_ori,
        warm_start=warm_start, position_tolerance=0.001, orientation_tolerance=0.02)
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


def g_tool_change_grasp(ctx):
    """자기 전용 거치대(ctx.tool_path/dock_xy)에서 노즐 파지. 성공 시 ctx.holding_nozzle=True,
    ctx.nozzle_tip_offset 갱신. 반환값 = grasp_ok(bool)."""
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
        print(f"[TOOLCHANGE][{ctx.name}][FAIL] 노즐 파지 실패")
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
    ctx.nozzle_tip_offset 실측 완료. 반환 = True(목표 거리 도달) / False(조기종료·조준IK실패·grip풀림)."""
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
    """공용 쓰레기통(TRASH_CAN_PRIM) 대상 PICK → DUMP → RETURN(원위치 복귀) → DOCK(자기 거치대
    복귀) 4단계. 15_ carter2_mission 과 동일 로직이되:
      · from_xy 를 고정 스폰좌표 대신 "지금 이 순간의 챠시 위치"로 계산(호출 시점이 스폰 직후라는
        보장이 없어짐 — 작업 선택식이라 이 서브미션이 여러 번, 임의 위치에서 시작될 수 있음).
      · DOCK 목적지 = ctx.dock_approach_xy/yaw(자기 전용 거치대 근처)로 통일.
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
    yield from g_stow_arm_for_nav(ctx)   # PICK 진입 전 — 아직 쓰레기통 파지 전(빈 손)
    yield from g_run_nav_leg(ctx, standoff_xy, standoff_yaw, chassis_goal_xy, chassis_goal_yaw, "PICK")
    if not simulation_app.is_running():
        return
    sync_rmpflow_base_pose(ctx)

    ctx.status = "PICK: 쓰레기통 파지 시퀀스"
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
        yield from g_move_to_pose(ctx, grasp_position + lateral_vec, grasp_orientation, "좌우/높이 보정")
        grasp_position = get_prim_world_position(ctx.tool0_path)
    elif lateral_err > LATERAL_CORRECTION_MAX:
        print(f"[WARN][{ctx.name}] 수직오차 과대({lateral_err:.3f}m) → 보정 생략")

    current_target = grasp_position.copy()
    gripped_ok = False
    for creep_step in range(CREEP_MAX_STEPS):
        current_target = current_target + move_dir * CREEP_STEP_SIZE
        for _ in range(CREEP_SETTLE_STEPS):
            yield
            ctx.robot.apply_action(ctx.rmpflow.forward(
                target_end_effector_position=current_target, target_end_effector_orientation=grasp_orientation))
        ctx.gripper.close()
        if ctx.gripper.is_closed():
            gripped_ok = True
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
    yield from g_run_nav_leg(ctx, big_standoff, big_yaw, big_goal, big_yaw, "DUMP")
    if not simulation_app.is_running():
        return
    yield from g_dump_into_big_trash(ctx)

    yield from g_restore_upright_after_dump(ctx)
    yield from g_drive_straight_open_loop(ctx, POST_DUMP_BACKUP_DISTANCE, ctx.articulation_root,
                                          FINAL_APPROACH_SPEED, reverse=True)
    post_dump_yaw = wrap_pi(get_chassis_yaw(ctx.articulation_root) + np.pi)
    yield from g_rotate_in_place(ctx, post_dump_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, ctx.articulation_root)
    print(f"[APPROACH:DUMP][{ctx.name}] {POST_DUMP_BACKUP_DISTANCE:.2f}m 후진 + 180 회전 → RETURN 시작")

    ctx.status = "RETURN: 원위치 복귀 이동"
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
    yield from g_rotate_in_place(ctx, post_ret_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, ctx.articulation_root)
    print(f"[APPROACH:RETURN][{ctx.name}] {POST_RETURN_BACKUP_DISTANCE:.2f}m 후진 + 180 회전 → DOCK 시작")

    ctx.status = "DOCK: 자기 거치대 복귀"
    dock_dir = np.array([np.cos(ctx.dock_approach_yaw), np.sin(ctx.dock_approach_yaw)])
    dock_standoff_xy = ctx.dock_approach_xy - dock_dir * FINAL_APPROACH_DISTANCE
    yield from g_run_nav_leg(ctx, dock_standoff_xy, ctx.dock_approach_yaw,
                             ctx.dock_approach_xy, ctx.dock_approach_yaw, "DOCK")
    if not simulation_app.is_running():
        return
    yield from g_precise_dock_approach(ctx, ctx.dock_approach_xy, ctx.dock_approach_yaw,
                                       ctx.articulation_root, "DOCK")
    if not simulation_app.is_running():
        return

    print(f"[INFO][{ctx.name}] 트래시 미션 완료(파지+덤프+원위치복귀+도킹).")
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
    """거치대 팔 작업(파지/반납) 전 챠시를 자기 전용 거치대 근처(ctx.dock_approach_xy/yaw)로 필요할
    때만 nav-leg 이동시킨다(이미 위치·자세가 충분히 근접하면 생략 — 15_ 검증 패턴)."""
    cur_xy = get_prim_world_position(ctx.articulation_root)[:2]
    cur_yaw = get_chassis_yaw(ctx.articulation_root)
    xy_close = float(np.linalg.norm(cur_xy - ctx.dock_approach_xy)) < DOCK_APPROACH_SKIP_XY_RADIUS
    yaw_close = abs(wrap_pi(cur_yaw - ctx.dock_approach_yaw)) < DOCK_APPROACH_SKIP_YAW_TOL
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
                             ctx.dock_approach_xy, ctx.dock_approach_yaw, label)
    if not simulation_app.is_running():
        return
    yield from g_precise_dock_approach(ctx, ctx.dock_approach_xy, ctx.dock_approach_yaw,
                                       ctx.articulation_root, label)
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
    yield from g_stow_arm_for_nav(ctx)
    yield from g_run_nav_leg(ctx, spray_standoff_xy, ctx.spray_wp_yaw, ctx.spray_wp_xy, ctx.spray_wp_yaw, "SPRAY_GOTO")
    if not simulation_app.is_running():
        return

    ctx.status = "SPRAY: 스윕 진행 중"
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
            print(f"[MISSION][{ctx.name}] 트래시 작업 완료 → IDLE 복귀")

        elif task == "spray":
            if not ctx.holding_nozzle:
                yield from g_nav_to_dock_approach(ctx, "SPRAY_PRE_GRASP")
                ok = yield from g_tool_change_grasp(ctx)
                if not ok:
                    print(f"[MISSION][{ctx.name}][FAIL] 노즐 파지 실패 — 분사 작업 취소, IDLE 복귀")
                    continue
            yield from g_with_resource(ctx, spray_lock, "spray", lambda: g_spray_mission_body(ctx))
            yield from g_tool_change_release(ctx)
            print(f"[MISSION][{ctx.name}] 분사 작업 완료 → IDLE 복귀")

        else:
            print(f"[MISSION][{ctx.name}][WARN] 알 수 없는 task '{task}' — 무시")


# ════════════════════════════════════════════════════════════════════════════
#  L. main — 두 로봇 협조 루프
# ════════════════════════════════════════════════════════════════════════════
def main():
    en_c1 = os.environ.get("ENABLE_C1", "1") == "1"
    en_c2 = os.environ.get("ENABLE_C2", "1") == "1"
    print(f"[CFG] ENABLE_C1={en_c1} ENABLE_C2={en_c2}")

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
    ros_node = rclpy.create_node("dual_task_select_tool_changer_controller")
    clock_pub = ros_node.create_publisher(Clock, "/clock", 10)     # 전역 단일 /clock

    trash_lock = {"holder": None}
    spray_lock = {"holder": None}

    c1_ctx = None; c1_cmd_pub = None; c1_gen = None; c1_done = not en_c1
    c2_ctx = None; c2_cmd_pub = None; c2_gen = None; c2_done = not en_c2

    try:
        if en_c1:
            c1_robot.initialize()
            c1_dof = list(c1_robot.dof_names)
            dp = c1_robot.get_joint_positions()
            for i, name in enumerate(ARM_JOINT_NAMES):
                if name in c1_dof:
                    dp[c1_dof.index(name)] = float(STOW_Q[i])
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
                              NOZZLE_HOLD_JOINT_PATH_C1, NOZZLE_HOLD_ANCHOR_PATH_C1, SPRAY_WP1_XY, SPRAY_WP1_YAW)
            c1_ctx.tool_changer = c1_tool_changer
            _selftest_c1 = os.environ.get("SELFTEST_TASK_C1", "").strip().lower()
            if _selftest_c1 in ("spray", "trash"):
                c1_task_state["task"] = _selftest_c1
                print(f"[SELFTEST] carter1 task_select 자동 트리거 = '{_selftest_c1}'")
            c1_gen = g_task_select_mission(c1_ctx, trash_lock, spray_lock)

        if en_c2:
            c2_robot.initialize()
            c2_dof = list(c2_robot.dof_names)
            dp = c2_robot.get_joint_positions()
            for name in ARM_JOINT_NAMES:
                if name in c2_dof:
                    dp[c2_dof.index(name)] = 0.0
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
                              NOZZLE_HOLD_JOINT_PATH_C2, NOZZLE_HOLD_ANCHOR_PATH_C2, SPRAY_WP1_XY, SPRAY_WP1_YAW)
            c2_ctx.tool_changer = c2_tool_changer
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
                try:
                    next(c1_gen)
                except StopIteration:
                    c1_done = True
                    print("[C1] 미션 제너레이터 종료(비정상 — 통상 IDLE 유휴 루프라 안 끝남)")

            if not c2_done:
                try:
                    next(c2_gen)
                except StopIteration:
                    c2_done = True
                    print("[C2] 미션 제너레이터 종료(비정상 — 통상 IDLE 유휴 루프라 안 끝남)")

            hb += 1
            if hb % 300 == 0:
                c1s = "(비활성 ENABLE_C1=0)" if not en_c1 else ("완료" if c1_done else c1_ctx.status)
                c2s = "(비활성 ENABLE_C2=0)" if not en_c2 else ("완료" if c2_done else c2_ctx.status)
                print(f"[HB] carter1 = {c1s}\n     carter2 = {c2s}")

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
