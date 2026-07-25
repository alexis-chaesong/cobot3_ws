"""
16_dual_sg_tool_changer_integrated.py  ★멀티로봇 Phase C — 로봇 2대 모두 Surface Gripper★
================================================================================
13_multi_robot_integrated.py 를 베이스로, 로봇 2대(carter1=소독 / carter2=폐기물)를 한 씬
(modified_hospital)에 스폰하되 ★둘 다 Surface Gripper 하드웨어★로 통일한다. 역할 분담은
13_ 과 동일(carter1=소독 전담, carter2=폐기물 전담) — 역할을 섞지 않는다.

  · carter1 (소독)   : Nova Carter + m0609 Surface Gripper (13_ 의 "노즐 직접 부착"에서 변경).
                       스폰 docking_station_1 (18.5, 0). 이제 시작 시 노즐이 붙어있지 않고,
                       그리퍼로 노즐 거치대(NozzleDock)의 분사노즐을 ★툴체인지(attach)★ 한 뒤
                       기존 소독 스윕(벽면 상하 스윕 + 전진)을 수행한다. 툴체인지/스윕 로직은
                       15_single_robot_tool_changer_integrated.py 에서 검증한 것을 그대로 이식.
                       조율 토픽 : /carter1/start_sweep(←) · /carter1/sweep_done(→) · /carter1/cmd_vel.
                       핸드오프 계약(start_sweep/sweep_done)은 13_ 과 100% 동일 — ★첫 start_sweep
                       (거치대 웨이포인트 도착) = 노즐 파지, 이후 start_sweep = 소독 스윕★ 으로만 확장.
  · carter2 (폐기물) : Nova Carter + m0609 Surface Gripper + 소형 쓰레기통. 13_ 과 로직 100% 동일.
                       스폰 docking_station_02 (16.6629, -0.00295). Nav2 주행→파지→big_trash 덤프
                       →원위치 복귀→도킹. 4_mobile_manipulator_trash_can_nav_pick_test.py 제너레이터.
                       조율 토픽 : /carter2/trash_can_nav_goal(→) · /carter2/start_pick(←) · /carter2/cmd_vel.

★ROS2 네임스페이스 확정(요구사항 #4 — 이후 대시보드 자유클릭 내비게이션에서 재사용)★ :
  · carter1(소독)   → 접두 "/carter1"  (예: /carter1/cmd_vel, /carter1/scan, /carter1/tf,
                       /carter1/chassis/odom, /carter1/start_sweep, /carter1/sweep_done)
  · carter2(폐기물) → 접두 "/carter2"  (예: /carter2/cmd_vel, /carter2/scan, /carter2/tf,
                       /carter2/chassis/odom, /carter2/trash_can_nav_goal, /carter2/start_pick)
  · /clock 만 네임스페이스 없는 전역(두 로봇 Nav2 공유) — 이 스크립트가 딱 하나 발행.
  · HMI robotId 매핑(웹 대시보드) : "disinfect"=carter1, "waste"=carter2 (hmi_link.py 참조).

두 로봇 병합의 핵심 = 프림 경로 분리 + namespace :
  · 두 Nova Carter 는 각각 /World/Carter1, /World/Carter2 스코프 prim 아래에 놓아 경로 충돌을 피한다.
  · ★변경점(13_ 대비)★ : carter1 도 carter2 와 동일한 move_tash_can.usd(Nova+그리퍼팔) 를 참조해
    검증된 Surface Gripper 팔을 그대로 얻는다. carter1 은 폐기물을 안 하므로 그 안의 쓰레기통 prim 을
    비활성화(SetActive(False))하고, 대신 스폰 근처에 NozzleDock(노즐 거치대) 를 세운다.
  · 각 Carter 내부 4개 ActionGraph 의 node_namespace 를 carter1/carter2 로 설정. set_carter_namespace().

협조 루프(데드락 방지) — 13_ 과 동일하되 carter1 도 제너레이터화 :
  · carter1 : 제너레이터(g_carter1_mission) — 첫 start_sweep=노즐 파지(g_tool_change_grasp),
              이후 start_sweep=소독 스윕(g_spray_sweep). (13_ 의 Carter1Spray.tick() 상태머신 대체.)
  · carter2 : 제너레이터(carter2_mission) — 13_ 그대로. main 루프가 매 스텝 두 제너레이터를 next().

실행 (총 5개 터미널 권장) :
  1) 이 스크립트 : python.sh isaacpjt/M0609/16_dual_sg_tool_changer_integrated.py → GUI 에서 Play ▶
  2) Nav2(멀티) : ros2 launch carter_navigation \
        multiple_robot_carter_navigation_modified_hospital.launch.py \
        map:=<carter_navigation>/maps/map/modified_hospital_map.yaml
  3) carter1 미션 : ros2 run commander spray_waypoint_mission --ros-args -p namespace:=carter1 \
        -p sweep_x:="[<dock_x>, 18.8, 18.5]" -p sweep_y:="[<dock_y>, 8.0, 18.5]" \
        -p sweep_yaw:="[<dock_yaw>, 1.5708, -1.5708]"
        ※ ★첫 웨이포인트 = 노즐 거치대 접근 pose(NOZZLE_DOCK_APPROACH)★ 를 반드시 맨 앞에 추가할 것.
          거기서 첫 start_sweep 이 "노즐 파지"를 트리거하고, 이후 웨이포인트에서 실제 소독 스윕이 돈다.
  4) carter2 미션 : ros2 run commander trash_can_nav_pick_mission --ros-args -p namespace:=carter2
  · 웹 HMI 게이트 사용 시 : run_missions_hmi.sh (wait_for_hmi_start:=True) — "통합 시작/개별 시작"
    버튼(→ /robot/command {command:START, robotId}) 계약은 ROS 미션노드(hmi_link.py) 측이므로 이
    스크립트는 변경 불필요. 통합시작=두 미션 동시, 개별시작=robotId 로 한 로봇만.

────────────────────────────────────────────────────────────────────────────────
⚠ Isaac+Nav2 라이브 검증 필요(오프라인 헤드리스로는 씬합성/제너레이터 유휴까지만 확인 가능) :
  [V1] Play 후 두 스폰 위치([SPAWN] c1/c2 chassis world) 가 (18.5,0)/(16.66,0) 근처인지.
  [V2] `ros2 topic list` 에 /carter1/... 와 /carter2/... 가 충돌 없이 분리돼 보이는지(namespace).
  [V3] carter1 이 거치대 웨이포인트에서 노즐 파지 성공([TOOLCHANGE] 노즐 파지 성공) 후 스윕 수행.
  [V4] carter2 가 기존과 동일한 파지/덤프/복귀/도킹 시퀀스를 정상 수행(상대기하 TARGET_JOINTS_DEG).
  [V5] carter1 파지 IK 도달성 : NOZZLE_DOCK_XY 와 거치대 접근 웨이포인트가 맞아야 함(도달범위 밖이면
       [TOOLCHANGE][WARN] IK 실패) — 라이브 실측으로 NOZZLE_DOCK_XY/접근 pose 미세조정 필요할 수 있음.
  ※ 연속 Play 전제(Stop→Play 재개 미지원 — 제너레이터/physics 뷰 상태 리셋 안 됨).
================================================================================
"""
import os

from isaacsim import SimulationApp

# 기본 GUI. 스모크 테스트/헤드리스 실행 시 ISAAC_HEADLESS=1 로 창 없이 부팅.
# 원격 노트북에서 WebRTC로 볼 때는 LIVESTREAM=1 (NVIDIA Isaac Sim WebRTC Streaming Client 필요).
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
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
import isaacsim.robot_motion.motion_generation as mg

import json

import rclpy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped
from rosgraph_msgs.msg import Clock

_THIS_DIR = Path(__file__).resolve().parent                 # isaacpjt/M0609
_WS_ROOT = _THIS_DIR.parent.parent                           # 레포 루트(clone 위치 무관)

# carter2 RMPflow 컨트롤러(그리퍼 팔) : integration/rmpflow 에 common/description yaml 있음.
_C2_RMPFLOW_DIR = str(_WS_ROOT / "src" / "integration" / "integration" / "rmpflow")
if _C2_RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, _C2_RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

# [16번 신규] carter1 Surface-Gripper 툴체인지 : 15_single_robot_tool_changer_integrated.py 가
# 재사용한 것과 동일한 ToolChangerController / surface_gripper_utils (isaac_wiping_task).
_TOOL_CHANGER_DIR = str(_WS_ROOT / "src" / "isaac_wiping_task")
if _TOOL_CHANGER_DIR not in sys.path:
    sys.path.insert(0, _TOOL_CHANGER_DIR)
import surface_gripper_utils  # noqa: E402
from tool_changer import ToolChangerController  # noqa: E402


# ════════════════════════════════════════════════════════════════════════════
#  A. 공통 경로/상수
# ════════════════════════════════════════════════════════════════════════════
# 환경/맵 : Nav2 멀티런치가 로드하는 modified_hospital_map.yaml 과 같은 씬(같은 world 프레임).
# carter_navigation 은 이 레포 밖의 별도 워크스페이스에 있어 상대경로로 못 잡는다 —
# CARTER_NAV_WS env var 로 override 가능(기본값 = 팀 관례 위치).
_CARTER_NAV_WS = Path(os.environ.get(
    "CARTER_NAV_WS", str(Path.home() / "IsaacSim-ros_workspaces" / "humble_ws")
))
HOSPITAL_USD = str(_CARTER_NAV_WS / "src" / "navigation" / "carter_navigation"
                    / "maps" / "map" / "modified_hospital.usd")
CARTER_URL = ("https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
              "Assets/Isaac/5.1/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd")
NOZZLE_USD = str(_WS_ROOT / "src" / "integration" / "integration" / "m0609_with_nozzle.usd")
MOVE_TRASH_USD = str(_WS_ROOT / "src" / "assets" / "scenes" / "move_tash_can.usd")

PHYSICS_DT = 1.0 / 60.0
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]

# ── ★성능 노브 (멀티로봇 GPU 병목 완화)★ ──
# RENDER_EVERY : 메인 루프에서 RTX 렌더를 매 스텝 → N스텝마다 1번(물리/제어/clock 은 매 스텝 유지).
#   1=현재(매 스텝), 2~3=GPU 여유↑·시각FPS↓. 카메라/라이다 등 렌더연동 센서 발행률도 ~1/N 로 낮아짐
#   (저속 Nav2 엔 2 정도 무난). GPU 80W 캡이면 2 권장.
RENDER_EVERY = 2
# 카메라 렌더 해상도 : Nova Carter 기본 1920x1200(로봇당 스테레오 다수) → RTX 부담 큼. 낮추면 픽셀수
#   제곱으로 절감(640x400 ≈ 1/9). 라이다 render product(1280x720)는 스캔 품질 위해 안 건드림.
#   카메라 시야는 유지되며 해상도만 낮아짐. 0 이면 원본 유지(축소 안 함).
CAM_RENDER_W = 640
CAM_RENDER_H = 400

# Nova Carter 내부 4개 ActionGraph 의 node_namespace 값(→ /carterN 접두). PDF "Nova Carter2 추가" 참조.
NS_CARTER1 = "carter1"
NS_CARTER2 = "carter2"

# ── 스코프 프림(경로 충돌 방지) ──
C1_SCOPE = "/World/Carter1"
C2_SCOPE = "/World/Carter2"


# ════════════════════════════════════════════════════════════════════════════
#  B. carter1 (소독 + 툴체인저) 상수
#     ★16번 변경★ : 13_ 은 carter1 을 m0609_with_nozzle.usd(노즐 직접 부착)로 스폰했으나,
#     이제 carter2 와 동일한 move_tash_can.usd(Nova + Surface Gripper 팔)를 참조해 검증된
#     그리퍼 팔을 그대로 얻는다. 폐기물은 안 하므로 그 안 쓰레기통 prim 은 비활성화하고,
#     스폰 근처에 노즐 거치대(NozzleDock)를 세워 그리퍼로 노즐을 툴체인지(attach)한다.
# ════════════════════════════════════════════════════════════════════════════
C1_CARTER_PRIM = f"{C1_SCOPE}/Nova_Carter_ROS"
C1_CHASSIS = f"{C1_CARTER_PRIM}/chassis_link"
C1_ARTICULATION_ROOT = C1_CHASSIS                    # SingleManipulator articulation root(= 챠시)
C1_ARM = f"{C1_SCOPE}/m0609"
C1_ARM_ROOT = C1_ARM                                 # 15_ 제너레이터가 쓰는 ctx.arm_root 값
C1_EE_LINK_NAME = "link_6"
C1_EE_LINK = f"{C1_ARM}/link_6"
C1_BASE_LINK = f"{C1_ARM}/base_link"
C1_SURFACE_GRIPPER = f"{C1_ARM}/{C1_EE_LINK_NAME}/mop_surface_gripper"
C1_TRASH_CAN_PRIM = f"{C1_SCOPE}/small_trash_can_body"   # carter1 은 폐기물 안 함 → 비활성화
C1_GROUND_PLANE = f"{C1_SCOPE}/GroundPlane"
C1_EXTRA_PHYSICS = f"{C1_SCOPE}/PhysicsScene"           # move_tash_can 내장 중복 PhysicsScene → 비활성화

C1_START_POSE = dict(x=18.5, y=0.0, z=0.05, yaw_deg=0.0)     # docking_station_1
MOUNT_OFFSET = Gf.Vec3d(-0.2317, 0.0, 0.5773)
CHASSIS_MASS = 150.0
DRIVE_MAX_ANG_SPEED = 2.5
DRIVE_MAX_ANG_ACCEL = 6.0
DRIVE_MAX_LIN_SPEED = 1.2

# 조준/툴체인지 IK (carter1 rmpflow 자산 재사용 — 15_ 의 IK_URDF_PATH/IK_DESCRIPTION_PATH 와 동일 파일)
C1_URDF = str(_THIS_DIR / "rmpflow" / "m0609_isaac_sim.urdf")
C1_DESC = str(_THIS_DIR / "rmpflow" / "m0609_description.yaml")
IK_URDF_PATH = C1_URDF
IK_DESCRIPTION_PATH = C1_DESC
EE_FRAME = "link_6"

# ── 노즐 거치대(NozzleDock) — 15_single_robot_tool_changer_integrated.py 이식 ──
NOZZLE_SOURCE_PRIMPATH = "/World/m0609/nozzle_base_link"   # tool0_to_nozzle 조인트 밖(형제) — 안 딸려옴
NOZZLE_DOCK_SCOPE = "/World/NozzleDock"
NOZZLE_TOOL_PATH = f"{NOZZLE_DOCK_SCOPE}/nozzle_tool"
NOZZLE_TCP_PATH = f"{NOZZLE_TOOL_PATH}/nozzle_tcp"
NOZZLE_HOLD_JOINT_PATH = f"{NOZZLE_DOCK_SCOPE}/hold_joint"
NOZZLE_HOLD_ANCHOR_PATH = f"{NOZZLE_DOCK_SCOPE}/hold_anchor"
# 거치대 위치 : carter1 스폰(18.5,0) 정면(+X) 0.35m — 15_ 에서 검증한 챠시-거치대 standoff(0.35m).
# ★거치대 접근 웨이포인트(NOZZLE_DOCK_APPROACH) = carter1 스폰 pose★ 로 두면 파지 IK 도달범위 안.
# (라이브 실측으로 미세조정 필요할 수 있음 — 벽/도달성 확인. V5 참고.)
NOZZLE_DOCK_XY = np.array([18.85, 0.0])
NOZZLE_DOCK_HEIGHT = 0.65

# Surface Gripper 튜닝(15_ 2단계 검증값 — grip_travel 을 IK 접근오차보다 넉넉히, clearance 는 최소).
MAX_GRIP_DISTANCE = 0.04
GRIP_DRIVE_STIFFNESS = 5000.0
GRIP_DRIVE_DAMPING = 100.0
CLEARANCE_OFFSET = 0.0005
GRIP_TRAVEL = 0.015

# 툴체인지 접근/파지 상수(15_ / 2_tool_changer_nozzle_demo.py 검증값 — 수직 매달기).
TC_APPROACH_ORIENTATION = np.array([0.0, 1.0, 0.0, 0.0])   # 로컬 Z가 world -Z (팁 아래로)
TC_EE_OFFSET = np.array([0.0, 0.0, 0.15])                  # 접근 시 위 여유공간
TC_GRASP_CLEARANCE = np.zeros(3)
TC_FINGERTIP_OFFSET_FROM_TOOL0 = np.zeros(3)               # 맨몸 팔 : fingertip(link_6)≈tool0
TC_GRASP_SETTLE_STEPS = 30
TC_REDOCK_SETTLE_STEPS = 20
TC_JOINT_RAMP_STEPS = 200
# URDF-USD 형상 불일치 고정 바이어스 보정(15_ 실측, TC_APPROACH_ORIENTATION 접근에만 유효).
TC_TARGET_BIAS_COMPENSATION = np.array([0.006, -0.0003, 0.0])

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
# 파지 직후 자세 → 스윕 저점(q_of_s(-1.0)) 진입을 부드럽게(15_ GUI 확인 후 추가).
SPRAY_ENTRY_RAMP_STEPS = 220

STROKES_PER_WIPE = 2
MOVE_DISTANCE = 0.25
FORWARD_DISTANCE = 10.5
FORWARD_SPEED = 0.35
FORWARD_ACCEL = 0.50
KP_YAW = 2.5
KP_LAT = 2.0        # [캐스터 대각선 보정] 좌우 이탈 되당김 게인 1.2→2.0 (더 세게 직선복귀)
KD_YAW = 0.4        # [캐스터 대각선 보정] yaw_rate 미분 댐핑(c2 drift_kd 차용) — 세게 당겨도 안 출렁이게
W_MAX = 0.8

# carter1 조율/센서 토픽(네임스페이스 접두). Nav2 가 발행하는 /carter1/cmd_vel 을 스윕 중엔 이 스크립트가.
C1_CMD_VEL = f"/{NS_CARTER1}/cmd_vel"
C1_START_SWEEP = f"/{NS_CARTER1}/start_sweep"
C1_SWEEP_DONE = f"/{NS_CARTER1}/sweep_done"

# 분사 파티클 FX (경량 탄도 풀) — 10_1 과 동일
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


# ════════════════════════════════════════════════════════════════════════════
#  C. carter2 (폐기물) 상수 — 4_mobile_manipulator_trash_can_nav_pick_test.py 와 동일 값
# ════════════════════════════════════════════════════════════════════════════
C2_ARTICULATION_ROOT = f"{C2_SCOPE}/Nova_Carter_ROS/chassis_link"
C2_ARM_ROOT = f"{C2_SCOPE}/m0609"
C2_EE_LINK_NAME = "link_6"
C2_TRASH_CAN_PRIM = f"{C2_SCOPE}/small_trash_can_body"
C2_SURFACE_GRIPPER = f"{C2_ARM_ROOT}/{C2_EE_LINK_NAME}/mop_surface_gripper"
C2_GROUND_PLANE = f"{C2_SCOPE}/GroundPlane"
C2_EXTRA_PHYSICS = f"{C2_SCOPE}/PhysicsScene"       # move_tash_can 내장 중복 PhysicsScene → 비활성화

C2_NO_GRIPPER_URDF = str(_WS_ROOT / "src" / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf")

TARGET_JOINTS_DEG = [-90.0, 101.0, 50.0, -94.0, 91.8, -1.1]
# [태성 갱신] 이동 중 j1 자세 170→10도. DUMP 도 동일값 유지(j6 만 실제로 움직여 비움).
TUCK_J1_DEG = 10.0
DUMP_J1_DEG = TUCK_J1_DEG            # [태성] 이동 중과 동일 자세 유지
DUMP_J6_ROTATE_DEG = -180.0          # [태성] 손목 뒤집기 방향 = 시계방향(음수)
DUMP_RAMP_STEPS = 90                 # [태성] dump 는 j6 하나만 크게 → 파지보다 빠른 램프
POST_DUMP_BACKUP_DISTANCE = 0.6      # [태성] 덤프 직후 후진(RETURN Nav2 켜기 전 코스트맵 여유) [m]
POST_RETURN_BACKUP_DISTANCE = 0.6    # [태성] 내려놓기 직후 후진(DOCK Nav2 켜기 전) [m]

# [태성 갱신] big_trash 를 (4,7.5) 로 이동, 접근방향 +Y→+X (맵 재스캔: +X 쪽이 1.4m 로 제일 트임)
BIG_TRASH_POSITION_XY = np.array([4.0, 7.5])
BIG_TRASH_APPROACH_DIR = np.array([1.0, 0.0])
BIG_TRASH_STANDOFF_DISTANCE = 1.3    # [태성] cmd_vel 실이동 = STANDOFF-FINAL = 0.8m
BIG_TRASH_FINAL_DISTANCE = 0.5

LIFT_OFFSET = np.array([0.0, 0.0, 0.75])
POSITION_TOLERANCE = 0.03
MAX_APPROACH_STEPS = 400
# [태성] 원위치 내려놓기용 : 고정 tolerance 대신 0→이 값까지 점점 느슨(지수) → 어색한 미세보정 방지
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

TRASH_SPAWN_FIXED = (14.0, 6.5)      # [태성] 쓰레기통 스폰 (7.5,7.5)→(14,6.5), 벽 여유 2.3m
TRASH_SPAWN_Z = 0.0786364536328308
# [태성] xformOp:translate(로컬원점)와 bbox 중심의 world 오프셋. relocate/entry 계산에 반영.
TRASH_BBOX_CENTER_OFFSET_XY = np.array([0.15, 0.15])
OFFSET_TRASH_FROM_CHASSIS = np.array([-0.750368208, -0.758035257])
C2_START_POSE = dict(x=16.66290495232035, y=-0.0029517927591273807, yaw_deg=0.0)  # docking_station_02

C2_NAV_GOAL = f"/{NS_CARTER2}/trash_can_nav_goal"
C2_START_PICK = f"/{NS_CARTER2}/start_pick"
C2_CMD_VEL = f"/{NS_CARTER2}/cmd_vel"

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
PRE_ROTATE_NUDGE_DISTANCE = 0.25     # [태성] 0.3→0.25


# ════════════════════════════════════════════════════════════════════════════
#  D. namespace 유틸 + 수학 유틸
# ════════════════════════════════════════════════════════════════════════════
def set_carter_namespace(root_path, ns):
    """root_path(=한 Nova Carter 프림) 하위 ROS2 노드에 namespace(ns) 접두를 프로그램 설정.
    Nova Carter 는 네임스페이스 상수가 두 종류라 둘 다 처리해야 한다(라이브 USD 검사로 확정) :
      (1) "node_namespace" 상수 4개(ActionGraph : ros_lidars·transform_tree_odometry·
          differential_drive·chassis_imu) → inputs:value 를 ns 로 설정 → scan·cmd_vel·tf·odom 에 /ns 접두.
      (2) "camera_namespace" 상수(각 hawk 카메라 그래프 : front/right/left/back_hawk) → inputs:value 가
          '/front_stereo_camera' 같은 카메라 이름. 여기에 ns 를 '앞에 덧붙여' '/carterN/front_stereo_camera'
          로 만들어야 카메라 토픽이 /carterN/front_stereo_camera/{left,right}/{image_raw,camera_info} 로
          분리된다(안 하면 두 로봇이 /front_stereo_camera/... 공유해 충돌 = 14-4 #2).
    ※ 카메라의 inputs:nodeNamespace 는 camera_namespace 상수에 '연결'돼 있으므로 노드에 직접 Set 하면
       안 되고(무시됨), 반드시 이 상수 값을 바꿔야 한다."""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    n = 0       # (1) node_namespace 상수 수
    n_cam = 0   # (2) camera_namespace 상수 수
    for prim in Usd.PrimRange(root):
        nm = prim.GetName()
        if nm == "node_namespace":
            a = prim.GetAttribute("inputs:value")
            if a and a.IsValid():
                a.Set(ns); n += 1
        elif nm == "camera_namespace":
            a = prim.GetAttribute("inputs:value")
            if a and a.IsValid():
                cur = (a.Get() or "").strip("/")     # 예: 'front_stereo_camera'
                a.Set(f"/{ns}/{cur}" if cur else f"/{ns}")   # → '/carter1/front_stereo_camera'
                n_cam += 1
    print(f"[NS] {root_path} → ns='{ns}' (node_namespace {n}개, camera_namespace {n_cam}개[카메라])")
    if n == 0:
        print(f"[NS][WARN] {root_path} 하위 node_namespace 노드 못 찾음 — 토픽 접두 실패 위험")
    if n_cam == 0:
        print(f"[NS][WARN] {root_path} 하위 camera_namespace 노드 못 찾음 — 카메라 토픽 분리 실패 위험")
    return n


def set_camera_resolution(root_path, width, height):
    """root_path 하위 '카메라' render product 노드의 inputs:width/height 를 낮춰 RTX 렌더 부담을 줄인다.
    (라이다 render product '*_lidar_render_product' 는 스캔 품질 위해 제외 — 이름으로 필터.)
    Play/첫 tick 전에 호출해야 render product 가 이 해상도로 생성된다(build 단계에서 호출)."""
    if not width or not height:
        return 0
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    n = 0
    for prim in Usd.PrimRange(root):
        nm = prim.GetName()
        # 카메라 render product 노드만 : hawk 은 '*_camera_render_product', owl 은 'isaac_create_render_product'
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
    """[15_ 이식] spray_orientation_quat() 을 "챠시 로컬 기준 조준방향"으로 재해석해 현재
    base_quat 로 합성(position 과 동일하게 챠시 프레임을 통째로 회전)한 진짜 월드 orientation 반환.
    carter1 은 항상 스폰 yaw 근처에서만 스윕하지만, 웨이포인트 도착 yaw 오차를 흡수하도록 15_ 방식 채택."""
    R_world = quat_to_matrix(base_quat) @ quat_to_matrix(spray_orientation_quat())
    return matrix_to_quat_wxyz(R_world)


def aim_link6_world_from_offset(base_pos, base_quat, ori_quat, z, tip_offset_link6_frame):
    """[15_ 이식] carter1 의 스칼라 Z 오프셋(NOZZLE_OFFSET) 방식을 3축 벡터로 일반화. tip_offset_
    link6_frame = 파지 직후 실측한 link_6 기준 노즐팁 상대위치(3축, ctx.nozzle_tip_offset)."""
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
    """[15_ 이식] child 를 parent 프레임에서 본 (상대위치 np3, 상대회전행렬 3x3)."""
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


# carter2 전용(4_ 와 동일) — world_transform 기반 헬퍼
def get_prim_world_position(prim_path):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    m = omni.usd.get_world_transform_matrix(prim)
    t = m.ExtractTranslation()
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


def find_articulation_root(root_path):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if root_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return root_path
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    return root_path


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


def boost_drive_limits(carter_prim_path):
    """차동구동 컨트롤러 각속도 클램프 상향(U턴 병목 해소, 6-15). 로봇별로 호출."""
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


# ════════════════════════════════════════════════════════════════════════════
#  E. 씬 구성
# ════════════════════════════════════════════════════════════════════════════
def build_env():
    """/World + hospital 환경 (한 번만)."""
    stage = omni.usd.get_context().get_stage()
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    world_prim.GetReferences().AddReference(HOSPITAL_USD)
    for _ in range(60):
        simulation_app.update()
    print(f"[LOAD] hospital env = {HOSPITAL_USD}")


def _place_xform(prim_path, x, y, z, yaw_deg):
    """XformCommonAPI 우회(6-13) : op 비우고 단일 transform 행렬로 배치."""
    stage = omni.usd.get_context().get_stage()
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    xf.ClearXformOpOrder()
    m = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), float(yaw_deg)))
    m.SetTranslateOnly(Gf.Vec3d(x, y, z))
    xf.AddTransformOp().Set(m)


def build_carter1():
    """[16번] carter1 = move_tash_can.usd(Nova + Surface Gripper 팔 + 쓰레기통, 이미 병합) 전체를
    /World/Carter1 스코프로 참조(carter2 와 동일 에셋 → 검증된 그리퍼 팔 재사용). 단,
    (a) 폐기물 안 하므로 내장 쓰레기통 prim 비활성화, (b) Nova 를 carter1 홈(18.5,0)으로 재배치,
    (c) 스폰 근처 노즐 거치대에서 그리퍼로 노즐을 툴체인지한다(build_nozzle_dock).
    build_carter2 와 대칭 구조 — 유일 차이는 홈 좌표 + 쓰레기통 비활성화."""
    stage = omni.usd.get_context().get_stage()
    scope = stage.DefinePrim(C1_SCOPE, "Xform")
    # identity 보장(참조된 /World 의 상대기하가 스코프 xform 으로 흔들리지 않게 — 재배치는 Nova prim 에서)
    UsdGeom.Xformable(scope).ClearXformOpOrder()
    scope.GetReferences().AddReference(Sdf.Reference(assetPath=MOVE_TRASH_USD, primPath="/World"))
    for _ in range(80):
        simulation_app.update()

    if not stage.GetPrimAtPath(C1_ARTICULATION_ROOT).IsValid():
        print(f"[FATAL] {C1_ARTICULATION_ROOT} 없음 — carter1 로드 실패"); return False

    # 중복 PhysicsScene 비활성화(환경/World 기본과 충돌 방지) — build_carter2 와 동일.
    extra = stage.GetPrimAtPath(C1_EXTRA_PHYSICS)
    if extra.IsValid():
        extra.SetActive(False)
        print(f"[SCENE] {C1_EXTRA_PHYSICS} 비활성화(중복 PhysicsScene)")

    # carter1 은 폐기물 안 함 → 내장 쓰레기통 prim 비활성화(씬에서 감춤 + 물리 제외).
    trash = stage.GetPrimAtPath(C1_TRASH_CAN_PRIM)
    if trash.IsValid():
        trash.SetActive(False)
        print(f"[SCENE] {C1_TRASH_CAN_PRIM} 비활성화(carter1 은 폐기물 미수행)")

    # ★carter1 홈 재배치★ : move_tash_can 은 Nova 를 carter2 홈(16.66,0)에 authoring 했으므로,
    #   Nova prim 을 carter1 홈(18.5,0)으로 override 배치한다(Play 전 authoring). 팔은 별도 사전배치
    #   후 Play 시 articulation FK 가 이 챠시 위로 스냅한다(build_carter2 의 팔 사전배치와 동일 원리).
    _place_xform(C1_CARTER_PRIM, C1_START_POSE["x"], C1_START_POSE["y"],
                 C1_START_POSE["z"], C1_START_POSE["yaw_deg"])
    simulation_app.update()

    chassis = stage.GetPrimAtPath(C1_ARTICULATION_ROOT)
    chassis_m = UsdGeom.Xformable(chassis).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    _wp = Gf.Transform(chassis_m).GetTranslation()
    print(f"[SPAWN] c1 chassis world = ({_wp[0]:.3f}, {_wp[1]:.3f}, {_wp[2]:.3f}) "
          f"(목표 {C1_START_POSE['x']:.2f},{C1_START_POSE['y']:.2f})")

    # 팔 사전배치(가시성) — build_carter2 와 동일. Play 후 물리가 같은 위치로 확정하므로 정합.
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


# ── 노즐 거치대(NozzleDock) + Surface Gripper 셋업 (15_ 이식, carter1 전용) ──
def build_nozzle_dock():
    """[15_ 이식] m0609_with_nozzle.usd 의 nozzle_base_link 서브트리만 거치대 경로에 참조해 "수직
    매달기"(TC_APPROACH_ORIENTATION=(0,1,0,0) → 로컬 Z가 world -Z)로 세우고, 임시 hold_joint(정적
    anchor prim 에 FixedJoint)로 고정. carter1 이 파지 성공 직후 release_hold_joint() 로 비활성화."""
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, NOZZLE_DOCK_SCOPE)
    tool_prim = stage.DefinePrim(NOZZLE_TOOL_PATH, "Xform")
    tool_prim.GetReferences().AddReference(
        Sdf.Reference(assetPath=NOZZLE_USD, primPath=NOZZLE_SOURCE_PRIMPATH))
    for _ in range(20):
        simulation_app.update()
    if not stage.GetPrimAtPath(NOZZLE_TOOL_PATH).IsValid():
        print(f"[FATAL] {NOZZLE_TOOL_PATH} 로드 실패"); return False

    dock_quat = TC_APPROACH_ORIENTATION
    rot = Gf.Rotation(Gf.Quatd(float(dock_quat[0]), Gf.Vec3d(*dock_quat[1:])))
    m = Gf.Matrix4d().SetRotate(rot).SetTranslateOnly(
        Gf.Vec3d(float(NOZZLE_DOCK_XY[0]), float(NOZZLE_DOCK_XY[1]), float(NOZZLE_DOCK_HEIGHT)))
    xf = UsdGeom.Xformable(tool_prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)
    print(f"[SPAWN] 노즐 거치대 = ({NOZZLE_DOCK_XY[0]:.3f},{NOZZLE_DOCK_XY[1]:.3f},{NOZZLE_DOCK_HEIGHT:.3f}) 수직 매달기")

    # 임시 거치 조인트 : 정적 anchor prim 을 body0 로(15_ : body0 미지정 시 물리 폭발 확인).
    anchor_prim = stage.DefinePrim(NOZZLE_HOLD_ANCHOR_PATH, "Xform")
    anchor_xf = UsdGeom.Xformable(anchor_prim)
    anchor_xf.ClearXformOpOrder()
    anchor_xf.AddTransformOp().Set(m)

    hold_joint = UsdPhysics.FixedJoint.Define(stage, NOZZLE_HOLD_JOINT_PATH)
    hold_joint.CreateBody0Rel().SetTargets([NOZZLE_HOLD_ANCHOR_PATH])
    hold_joint.CreateBody1Rel().SetTargets([NOZZLE_TOOL_PATH])
    hold_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    hold_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    hold_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    hold_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    hold_joint.CreateExcludeFromArticulationAttr().Set(True)
    hold_joint.CreateJointEnabledAttr().Set(True)
    print(f"[SCENE] 임시 거치 조인트 authoring = {NOZZLE_HOLD_JOINT_PATH} (anchor={NOZZLE_HOLD_ANCHOR_PATH})")
    return True


def release_hold_joint(stage):
    prim = stage.GetPrimAtPath(NOZZLE_HOLD_JOINT_PATH)
    UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(False)
    print(f"[INFO] 임시 거치 조인트 비활성화 = {NOZZLE_HOLD_JOINT_PATH}")


def engage_hold_joint(stage):
    prim = stage.GetPrimAtPath(NOZZLE_HOLD_JOINT_PATH)
    UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(True)
    print(f"[INFO] 임시 거치 조인트 재활성화 = {NOZZLE_HOLD_JOINT_PATH}")


def setup_nozzle_surface_gripper(stage):
    """[15_ 이식] carter1 그리퍼(move_tash_can 내장 mop_surface_gripper)의 기존 D6 조인트 속성만
    2단계 검증값(CLEARANCE_OFFSET/GRIP_TRAVEL 등)으로 직접 재기록(새로 만들면 attachmentPoints 관계
    깨질 위험)."""
    joint_path = f"{C1_ARM}/{C1_EE_LINK_NAME}/mop_surface_gripper_joints/mop_attachment_joint"
    gripper_prim = stage.GetPrimAtPath(C1_SURFACE_GRIPPER)
    joint_prim = stage.GetPrimAtPath(joint_path)
    if gripper_prim.IsValid() and joint_prim.IsValid():
        gripper_prim.GetAttribute("isaac:maxGripDistance").Set(float(MAX_GRIP_DISTANCE))
        joint_prim.GetAttribute("isaac:clearanceOffset").Set(float(CLEARANCE_OFFSET))
        joint_prim.GetAttribute("limit:transZ:physics:high").Set(float(GRIP_TRAVEL))
        joint_prim.GetAttribute("drive:transZ:physics:stiffness").Set(float(GRIP_DRIVE_STIFFNESS))
        joint_prim.GetAttribute("drive:transZ:physics:damping").Set(float(GRIP_DRIVE_DAMPING))
        print(f"[GRIPPER] {C1_SURFACE_GRIPPER} 기존 조인트 재튜닝 "
              f"(clearance={CLEARANCE_OFFSET}, travel={GRIP_TRAVEL}, maxGripDistance={MAX_GRIP_DISTANCE})")
        return C1_SURFACE_GRIPPER
    print(f"[GRIPPER][WARN] {C1_SURFACE_GRIPPER} 또는 {joint_path} 없음 — 새로 authoring")
    return surface_gripper_utils.setup_mop_surface_gripper(
        stage, fingertip_prim_path=f"{C1_ARM}/{C1_EE_LINK_NAME}",
        gripper_prim_path=C1_SURFACE_GRIPPER,
        max_grip_distance=MAX_GRIP_DISTANCE, grip_drive_stiffness=GRIP_DRIVE_STIFFNESS,
        grip_drive_damping=GRIP_DRIVE_DAMPING, clearance_offset=CLEARANCE_OFFSET, grip_travel=GRIP_TRAVEL)


def build_carter1_control(my_world):
    """carter1 SingleManipulator 를 scene 에 등록(world.reset 전) → (robot, ee_path, tool0_path)."""
    stage = omni.usd.get_context().get_stage()
    tune_arm_drives(C1_ARM)

    def find_by_name(root, name):
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if prim.GetName() == name:
                return str(prim.GetPath())
        return None

    ee_path = find_by_name(C1_ARM, C1_EE_LINK_NAME)
    tool0_path = find_by_name(C1_ARM, "tool0")
    if ee_path is None:
        raise RuntimeError(f"c1 link_6 못 찾음 (ee={ee_path})")
    robot = my_world.scene.add(SingleManipulator(
        prim_path=C1_ARTICULATION_ROOT, name="carter1_m0609",
        end_effector_prim_path=ee_path, gripper=None))
    return robot, ee_path, tool0_path


def build_carter2():
    """carter2 = move_tash_can.usd(Nova + 그리퍼팔 + 쓰레기통, 이미 병합됨) 전체를 /World/Carter2 스코프로
    참조. 상대기하 보존을 위해 통째로 참조하고 스코프 xform 은 identity 로 둔다."""
    stage = omni.usd.get_context().get_stage()
    scope = stage.DefinePrim(C2_SCOPE, "Xform")
    # identity 보장(참조된 /World 의 xform 이 스코프에 얹혀도 상대 좌표가 흔들리지 않게)
    UsdGeom.Xformable(scope).ClearXformOpOrder()
    scope.GetReferences().AddReference(Sdf.Reference(assetPath=MOVE_TRASH_USD, primPath="/World"))
    for _ in range(80):
        simulation_app.update()

    if not stage.GetPrimAtPath(C2_ARTICULATION_ROOT).IsValid():
        print(f"[FATAL] {C2_ARTICULATION_ROOT} 없음 — carter2 로드 실패"); return False

    # 중복 PhysicsScene 비활성화(환경/World 기본과 충돌 방지)
    extra = stage.GetPrimAtPath(C2_EXTRA_PHYSICS)
    if extra.IsValid():
        extra.SetActive(False)
        print(f"[SCENE] {C2_EXTRA_PHYSICS} 비활성화(중복 PhysicsScene)")

    # 쓰레기통을 재현 가능한 고정 스폰으로 이동(4_ 와 동일)
    trash = stage.GetPrimAtPath(C2_TRASH_CAN_PRIM)
    if trash.IsValid():
        ta = trash.GetAttribute("xformOp:translate")
        if ta and ta.IsValid():
            # [태성] TRASH_SPAWN_FIXED 는 bbox '중심'이 놓일 좌표 → 원점(translate)은 offset 만큼 뺀 값.
            _origin = np.asarray(TRASH_SPAWN_FIXED) - TRASH_BBOX_CENTER_OFFSET_XY
            ta.Set(Gf.Vec3d(float(_origin[0]), float(_origin[1]), TRASH_SPAWN_Z))
            print(f"[SPAWN] c2 trash 중심 → {TRASH_SPAWN_FIXED} (원점 {_origin.tolist()})")

    chassis2 = stage.GetPrimAtPath(C2_ARTICULATION_ROOT)
    chassis2_m = UsdGeom.Xformable(chassis2).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    _wp = Gf.Transform(chassis2_m).GetTranslation()
    print(f"[SPAWN] c2 chassis world = ({_wp[0]:.3f}, {_wp[1]:.3f}, {_wp[2]:.3f}) "
          f"(목표 {C2_START_POSE['x']:.2f},{C2_START_POSE['y']:.2f})")

    # ★ 팔 사전 배치(가시성) : move_tash_can 은 그리퍼 팔을 "원점 근처"에 authoring 해서, Play(물리) 전에는
    #   팔이 카터에서 ~16m 떨어져 보인다(="카터만, 팔 없음"으로 오인). articulation FK 가 Play 시 카터 위로
    #   스냅하지만, 혼동 방지 위해 로드 시점에 carter1 과 동일하게 팔 루트를 chassis 위(MOUNT_OFFSET)로 배치.
    #   (Play 후 물리가 같은 위치로 확정하므로 정합. 스냅 전후 pose 동일 — 헤드리스 검증됨: base_link≈(16.43,0,0.58))
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

    set_carter_namespace(f"{C2_SCOPE}/Nova_Carter_ROS", NS_CARTER2)
    set_camera_resolution(f"{C2_SCOPE}/Nova_Carter_ROS", CAM_RENDER_W, CAM_RENDER_H)
    print("[SCENE] carter2 (Nova + 그리퍼팔 + 쓰레기통) 완료")
    return True


# ════════════════════════════════════════════════════════════════════════════
#  F. carter1 소독+툴체인지 제너레이터 (SprayFX/Sweeper + 15_ 툴체인지/스윕 이식)
#     C1Ctx · 툴체인지(grasp/release) · 분사 스윕 · 최상위 미션(g_carter1_mission).
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
        """[15_ 이식] g_spray_sweep 이 끝나면 update() 가 더 안 불려 마지막 파티클이 화면에 얼어붙는다
        — 제너레이터 종료 시(완주/조기종료/grip풀림) 반드시 호출해 위젯을 0으로 밀어 즉시 지운다."""
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


class C1Ctx:
    """carter1(소독+툴체인저) 제너레이터가 공유하는 핸들 묶음. 15_ 의 C2Ctx(툴체인지 필드 포함)를
    carter1 경로/토픽에 맞춰 재구성한 것. arm_root/articulation_root 를 ctx 로 넘겨 15_ 의 툴체인지/
    스윕 제너레이터를 로봇 비종속으로 재사용한다."""
    def __init__(self, world, robot, dof_names, ee_path, tool0_path, gripper,
                 ros_node, cmd_pub, sweep_done_pub, sweep_state):
        self.world = world; self.robot = robot; self.dof_names = dof_names
        self.ee_path = ee_path; self.tool0_path = tool0_path; self.gripper = gripper
        self.ros_node = ros_node; self.cmd_pub = cmd_pub
        self.sweep_done_pub = sweep_done_pub
        self.sweep_state = sweep_state          # {"req": None|True|False} ← /carter1/start_sweep
        self.arm_root = C1_ARM_ROOT             # 15_ 제너레이터의 base_link 조회 기준
        self.articulation_root = C1_ARTICULATION_ROOT
        self.stage = omni.usd.get_context().get_stage()
        self.status = "시작 대기"
        # 툴체인지 상태
        self.tool_changer = None                # ToolChangerController, main() 에서 주입
        self.holding_nozzle = False
        self.nozzle_tip_offset = None           # link_6 기준 nozzle_tcp 상대위치(3축) — 파지 직후 실측


def build_carter1_toolchanger(stage, c1_ee_path):
    """[15_ build_common_control 축약] carter1 은 트래시 Cartesian 이동이 없어 RMPflow 불필요 —
    Surface Gripper + ToolChangerController 만 초기화. setup_nozzle_surface_gripper 는 씬 빌드 단계
    (reset 이전)에서 이미 호출됐다고 가정."""
    gripper = SurfaceGripper(end_effector_prim_path=c1_ee_path, surface_gripper_path=C1_SURFACE_GRIPPER)
    gripper.initialize()
    dock_quat = TC_APPROACH_ORIENTATION
    tool_changer = ToolChangerController(
        rg2_fingertip_prim_path=c1_ee_path,
        mop_handle_prim_path=NOZZLE_TOOL_PATH,
        stand_position=np.array([NOZZLE_DOCK_XY[0], NOZZLE_DOCK_XY[1], NOZZLE_DOCK_HEIGHT]),
        stand_orientation=dock_quat,
        approach_orientation=dock_quat,
        fingertip_offset_from_ik_frame=TC_FINGERTIP_OFFSET_FROM_TOOL0,
        rg2_gripper=None,
        surface_gripper_prim_path=C1_SURFACE_GRIPPER,
        auto_create_surface_gripper=False,
    )
    tool_changer.initialize()
    return gripper, tool_changer


def _tc_solve_and_ramp(ctx, ik, target_pos, target_ori, warm_start, ramp_steps, label,
                       bias_compensation=None):
    """[15_ 이식] 거치대는 고정 위치라 반응형 IK(RMPflow) 대신 "IK 1회 풀어 관절각 확정 → 램프".
    position_tolerance=0.001 + TC_TARGET_BIAS_COMPENSATION(URDF-USD 형상 불일치 고정 바이어스 보정).
    반환 = (관절각 rad 6dof|None, ok)."""
    bias = bias_compensation if bias_compensation is not None else TC_TARGET_BIAS_COMPENSATION
    corrected_target = np.asarray(target_pos) - bias
    q, ok = ik.compute_inverse_kinematics(
        EE_FRAME, corrected_target, target_ori,
        warm_start=warm_start, position_tolerance=0.001, orientation_tolerance=0.02)
    if not ok:
        print(f"[TOOLCHANGE][WARN] {label} IK 실패")
        return None, False
    q6 = np.asarray(q[:6])
    yield from g_ramp_to_joint_positions(ctx, np.degrees(q6), ramp_steps)
    actual_pos = get_prim_world_position(ctx.ee_path)
    actual_err = actual_pos - np.asarray(target_pos)
    print(f"[TOOLCHANGE] {label} 관절이동 완료 q={np.round(q6, 3)} "
          f"(link_6 오차 |e|={np.linalg.norm(actual_err) * 1000:.2f}mm)")
    return q6, True


def g_tool_change_grasp(ctx):
    """[15_ 이식] 거치대에서 노즐 파지. 성공 시 ctx.holding_nozzle=True, ctx.nozzle_tip_offset 갱신.
    파지 실패/IK 실패 시 상공 안전자세로 복귀 후 False 반환(폐기물 grasp 재시도 패턴과 동일 — 실패해도
    최상위 루프가 죽지 않게)."""
    tc = ctx.tool_changer
    # 긴급정지 취소 신호(start_sweep=False)가 이미 와 있으면 파지 시작 안 함(챠시는 정지 상태라
    # 주행 위험은 없지만, 취소를 존중해 즉시 STANDBY 로 되돌린다).
    if ctx.sweep_state.get("req") is False:
        print("[TOOLCHANGE][ESTOP] 파지 시작 전 취소 감지 → 중단")
        return False
    handle_position, handle_orientation = tc.approach_tool_stand()
    base_pos, base_quat = read_world_pose(f"{ctx.arm_root}/base_link")
    ik = mg.LulaKinematicsSolver(robot_description_path=IK_DESCRIPTION_PATH, urdf_path=IK_URDF_PATH)
    ik.set_robot_base_pose(base_pos, base_quat)

    q_above, ok_above = yield from _tc_solve_and_ramp(
        ctx, ik, handle_position + TC_EE_OFFSET, handle_orientation, None, TC_JOINT_RAMP_STEPS, "노즐 상공 접근")
    if not ok_above:
        return False
    if ctx.sweep_state.get("req") is False:      # 상공 접근 후 취소 감지
        print("[TOOLCHANGE][ESTOP] 상공 접근 후 취소 감지 → 중단")
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
        print("[TOOLCHANGE][FAIL] 노즐 파지 실패")
        yield from g_ramp_to_joint_positions(ctx, np.degrees(q_above), TC_JOINT_RAMP_STEPS)
        return False

    release_hold_joint(ctx.stage)
    for _ in range(10):
        yield

    rel_pos, _ = relative_pose(ctx.ee_path, NOZZLE_TCP_PATH)
    ctx.nozzle_tip_offset = rel_pos
    base_offset = get_prim_world_position(NOZZLE_TOOL_PATH) - get_prim_world_position(ctx.ee_path)
    tc.fingertip_offset_from_ik_frame = base_offset
    ctx.holding_nozzle = True
    print(f"[TOOLCHANGE] 노즐 파지 성공. link_6 기준 tcp 오프셋={np.round(rel_pos, 4)} "
          f"(|rel|={np.linalg.norm(rel_pos):.4f})")
    yield from g_ramp_to_joint_positions(ctx, np.degrees(q_above), TC_JOINT_RAMP_STEPS)
    return True


def g_tool_change_release(ctx):
    """[15_ 이식] 노즐을 거치대에 반납(현재 기본 미션 경로엔 미배선 — 반납하려면 마지막에 거치대
    복귀 웨이포인트 + 별도 트리거 필요. 확장용으로 유지). 성공 시 ctx.holding_nozzle=False, hold_joint
    재체결, fingertip_offset 리셋."""
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
    engage_hold_joint(ctx.stage)
    for _ in range(TC_REDOCK_SETTLE_STEPS):
        yield
    ctx.holding_nozzle = False
    ctx.nozzle_tip_offset = None
    tc.fingertip_offset_from_ik_frame = TC_FINGERTIP_OFFSET_FROM_TOOL0.copy()
    print(f"[TOOLCHANGE] 노즐 반납 {'성공' if release_ok else '실패'} + 재도킹 완료")
    yield from g_ramp_to_joint_positions(ctx, np.degrees(q_above), TC_JOINT_RAMP_STEPS)
    return release_ok


def g_spray_sweep(ctx, forward_distance=FORWARD_DISTANCE, max_steps=None):
    """[15_ 이식] WIPE(정지+상하 스윕)↔MOVE(팔 고정+전진, heading-hold) 반복. 사전조건:
    ctx.holding_nozzle=True, ctx.nozzle_tip_offset 실측 완료. 조준 IK 오프셋은 파지 직후 실측치 사용.
    반환 = True(목표 거리 도달) / False(조기종료·조준IK실패·grip풀림)."""
    if not ctx.holding_nozzle or ctx.nozzle_tip_offset is None:
        print("[SPRAY][FAIL] 노즐 파지 상태에서만 호출 가능(holding=False) — 중단")
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
    print(f"[SPRAY] 조준 IK q_low ok={ok_lo} q_high ok={ok_hi}")
    if not (ok_lo and ok_hi):
        print(f"[SPRAY][FAIL] 조준 IK 실패 (low={ok_lo}, high={ok_hi}) — 이 웨이포인트 스윕 불가")
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
    print(f"[SPRAY] 스윕 자세로 진입(부드럽게, {SPRAY_ENTRY_RAMP_STEPS}스텝)")
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
        # ★긴급정지/취소★ : 미션노드(_hmi_estop)가 /carter1/start_sweep=False 를 발행하면
        #   sweep_state["req"]=False → 즉시 스윕 중단 + cmd_vel 0. (13_ Carter1Spray.tick 의
        #   "handoff req False → STANDBY" 와 동일 역할. req 는 소비하지 않고 False 로 남겨둬
        #   호출부 g_carter1_mission 이 '완료'가 아닌 '취소'로 인지하게 한다.)
        if ctx.sweep_state.get("req") is False:
            publish_cmd(0.0, 0.0, drive_state)
            if spray_fx is not None:
                spray_fx.clear()
            print("[SPRAY][ESTOP] /carter1/start_sweep=False 수신 → 스윕 중단, cmd_vel 0")
            return False
        if max_steps is not None and step_i > max_steps:
            publish_cmd(0.0, 0.0, drive_state)
            if spray_fx is not None:
                spray_fx.clear()
            print(f"[SPRAY] max_steps({max_steps}) 도달 → 조기 종료(디버그)")
            return False

        if not (ctx.gripper.is_closed() if ctx.gripper else True):
            publish_cmd(0.0, 0.0, drive_state)
            if spray_fx is not None:
                spray_fx.clear()
            print("[SPRAY][WARN] 스윕 중 grip 이 풀림 감지 — 중단")
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
            print(f"[SPRAY][PHASE] WIPE 시작 (도착yaw={np.degrees(raw_yaw):.1f} → world축 스냅 {np.degrees(snapped_yaw):.0f})")

        progress = float(np.dot(chassis_pos - global_start, forward0))
        if progress >= forward_distance:
            publish_cmd(0.0, 0.0, drive_state)
            if spray_fx is not None:
                spray_fx.clear()
            print(f"[SPRAY] {forward_distance:.1f}m 도달(progress={progress:.2f}) → 스윕 종료")
            return True

        if spray_fx is not None:
            spraying = phase == "WIPE"
            if spraying:
                n_pos, n_quat = read_world_pose(NOZZLE_TCP_PATH)
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
                print(f"[SPRAY][{cycle}] WIPE→MOVE")
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
                print(f"[SPRAY][{cycle}] MOVE→WIPE (progress={progress:.2f})")
        apply(q_target)
        yield


def g_carter1_mission(ctx):
    """[16번 신규] carter1 최상위 소독 미션 — spray_waypoint_mission 의 /carter1/start_sweep ↔
    /carter1/sweep_done 핸드오프(13_ 계약 그대로)에 ★툴체인지 단계만 확장★ :
      · 첫 start_sweep(거치대 웨이포인트 도착) → g_tool_change_grasp (노즐 파지)
      · 이후 start_sweep(벽면 웨이포인트) → g_spray_sweep (기존 소독 스윕)
    각 단계 완료 후 /carter1/sweep_done=True 를 ~30스텝 발행해 미션노드가 다음 웨이포인트로 진행.
    ※ 노즐 반납(g_tool_change_release)은 현재 미배선 — 반납하려면 마지막 거치대 복귀 웨이포인트 +
      별도 트리거 필요(확장). 단일 미션 실행 동안 노즐은 계속 장착 상태로 유지."""
    ctx.status = "STANDBY: /carter1/start_sweep 대기"
    while True:
        # start_sweep=True 상승엣지 대기 (False/reset 은 흘려보냄)
        while ctx.sweep_state.get("req") is not True:
            if ctx.sweep_state.get("req") is False:
                ctx.sweep_state["req"] = None
            ctx.status = f"STANDBY: /carter1/start_sweep 대기 (노즐장착={ctx.holding_nozzle})"
            yield
        ctx.sweep_state["req"] = None

        if not ctx.holding_nozzle:
            ctx.status = "노즐 장착 중(툴체인지)"
            print("[C1][HANDOFF] 첫 start_sweep → 노즐 툴체인지 시작")
            ok = yield from g_tool_change_grasp(ctx)
            print(f"[C1] 노즐 파지 {'성공' if ok else '실패'}")
        else:
            ctx.status = "소독 분사 스윕"
            print("[C1][HANDOFF] start_sweep → 소독 스윕 시작")
            yield from g_spray_sweep(ctx)

        # ★긴급정지 처리★ : 방금 단계가 취소(start_sweep=False)로 끝났으면 완료통지(sweep_done)를
        #   보내면 안 된다(미션이 '스윕 완료'로 오인해 다음 웨이포인트로 진행). 취소 플래그를 소비하고
        #   cmd_vel 0 확정 후 STANDBY 로 복귀 — 웹 재시작(START)까지 대기.
        if ctx.sweep_state.get("req") is False:
            ctx.sweep_state["req"] = None
            ctx.cmd_pub.publish(Twist())
            ctx.status = "긴급정지 — STANDBY (재시작 대기)"
            print("[C1][ESTOP] 단계 취소 감지 → sweep_done 미발행, STANDBY 복귀")
            continue

        # 완료 통지 : /carter1/sweep_done=True 를 ~30스텝 반복 발행(13_ done_ticks 패턴)
        ctx.status = "단계 완료 → /carter1/sweep_done"
        for _ in range(30):
            ctx.sweep_done_pub.publish(Bool(data=True))
            yield
        ctx.status = "STANDBY: /carter1/start_sweep 대기"


# ════════════════════════════════════════════════════════════════════════════
#  G. carter2 폐기물 미션 — 4_ 를 제너레이터화 (world.step → yield)
#     각 헬퍼는 generator : main 이 next() 로 한 스텝씩 전진. apply_action 후 yield.
# ════════════════════════════════════════════════════════════════════════════
class C2Ctx:
    """carter2 제너레이터가 공유하는 핸들 묶음."""
    def __init__(self, world, robot, rmpflow, dof_names, tool0_path, gripper,
                 ros_node, goal_pub, cmd_pub, pick_state):
        self.world = world; self.robot = robot; self.rmpflow = rmpflow
        self.dof_names = dof_names; self.tool0_path = tool0_path; self.gripper = gripper
        self.ros_node = ros_node; self.goal_pub = goal_pub; self.cmd_pub = cmd_pub
        self.pick_state = pick_state
        self.stage = omni.usd.get_context().get_stage()
        self.status = "시작 대기"        # 하트비트 표시용 현재 단계


def sync_rmpflow_base_pose(ctx):
    base_prim = ctx.stage.GetPrimAtPath(f"{C2_ARM_ROOT}/base_link")
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
                   growing_tolerance_max=None, growing_tolerance_tau=None):
    """[태성] growing_tolerance_max 를 주면 고정 POSITION_TOLERANCE 대신 스텝 진행에 따라
    0→growing_tolerance_max 로 지수(1-e^(-step/tau))처럼 점점 느슨해지는 허용오차 사용 →
    원위치 내려놓기처럼 진입점이 매번 달라질 때 어색한 미세보정 대신 자연스럽게 멈춘다."""
    start = get_prim_world_position(ctx.tool0_path)
    distance = float(np.linalg.norm(target_position - start))
    ramp_steps = _ramp_steps_for_distance(distance, max_linear_speed)
    print(f"[INFO] c2 {label} 이동 (dist={distance:.3f}m)")
    yield from g_ramp_ee_target(ctx, target_position, target_orientation, ramp_steps)
    tau = growing_tolerance_tau if growing_tolerance_tau else MAX_APPROACH_STEPS / 4.0
    for step in range(MAX_APPROACH_STEPS):
        yield
        ee = get_prim_world_position(ctx.tool0_path)
        ctx.robot.apply_action(ctx.rmpflow.forward(
            target_end_effector_position=target_position, target_end_effector_orientation=target_orientation))
        if growing_tolerance_max is not None:
            cur_tol = growing_tolerance_max * (1.0 - np.exp(-step / tau))
        else:
            cur_tol = POSITION_TOLERANCE
        if float(np.linalg.norm(ee - target_position)) < cur_tol:
            print(f"[INFO] c2 {label} 도달 (step={step}, tol={cur_tol:.3f}m)"); return
    print(f"[WARN] c2 {label} {MAX_APPROACH_STEPS} step 내 미수렴")


def g_hold_pose(ctx, target_position, target_orientation, steps):
    for _ in range(steps):
        yield
        ctx.robot.apply_action(ctx.rmpflow.forward(
            target_end_effector_position=target_position, target_end_effector_orientation=target_orientation))


def g_rotate_in_place(ctx, target_yaw, kp, kd, w_max, max_w_step, tol, chassis_path, max_steps=600):
    # [태성] 종료조건 = "yaw_err 가 tol 안에 5스텝 연속 유지"(디바운스). 기존 w_applied<0.02 병행조건은
    # 잔여 진동/노이즈로 동시만족이 어려워 max_steps(=10초)를 다 채우는 일이 있었음(회전 2번이면 최악 20초).
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
    # [태성] reverse=True 면 같은 heading 을 유지한 채 뒤로 후진. distance 는 항상 양수(이동거리 크기).
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


def g_run_nav_leg(ctx, standoff_xy, standoff_yaw, chassis_goal_xy, chassis_goal_yaw, label):
    """Nav2 목표(standoff) 발행 → /start_pick 대기 → 실제 위치 기준 회전→직진→회전.
    /clock 은 main 이 발행하므로 여기선 목표만 발행."""
    for _ in range(30):        # 이전 구간의 잔류 /start_pick 을 흘려보냄(main 이 매 스텝 spin_once)
        yield
    ctx.pick_state["start"] = False

    goal = PoseStamped(); goal.header.frame_id = "map"
    goal.pose.position.x = float(standoff_xy[0]); goal.pose.position.y = float(standoff_xy[1])
    goal.pose.orientation.z = float(np.sin(standoff_yaw / 2.0))
    goal.pose.orientation.w = float(np.cos(standoff_yaw / 2.0))

    print(f"[NAV:{label}] c2 standoff={standoff_xy.tolist()} yaw={np.degrees(standoff_yaw):.1f} 발행")
    ctx.status = f"{label}: Nav2 이동 대기(/carter2/trash_can_nav_goal 발행중, /carter2/start_pick 기다림)"
    while not ctx.pick_state["start"]:
        yield
        # 목표 stamp 는 시뮬시간(/clock 과 동일 시간원)으로 — 4_ 와 동일(노드 wall clock 아님)
        st = float(ctx.world.current_time)
        goal.header.stamp.sec = int(st)
        goal.header.stamp.nanosec = int(round((st - int(st)) * 1e9))
        ctx.goal_pub.publish(goal)
        if not simulation_app.is_running():
            return

    print(f"[NAV:{label}] c2 /start_pick 수신 → 최종 접근")
    ctx.status = f"{label}: 최종 접근(회전→직진→회전)"
    cur = get_prim_world_position(C2_ARTICULATION_ROOT)[:2]
    to_goal = chassis_goal_xy - cur
    entry_distance = float(np.linalg.norm(to_goal))
    entry_yaw = float(np.arctan2(to_goal[1], to_goal[0]))
    yield from g_rotate_in_place(ctx, entry_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, C2_ARTICULATION_ROOT)
    yield from g_drive_straight_open_loop(ctx, entry_distance + PRE_ROTATE_NUDGE_DISTANCE, C2_ARTICULATION_ROOT)
    yield from g_rotate_in_place(ctx, chassis_goal_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, C2_ARTICULATION_ROOT)
    if FINAL_NUDGE_DISTANCE > 0.0:
        yield from g_drive_straight_open_loop(ctx, FINAL_NUDGE_DISTANCE, C2_ARTICULATION_ROOT)


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
    print(f"[INFO] c2 big_trash 덤프 완료 (j1={DUMP_J1_DEG}, j6+={DUMP_J6_ROTATE_DEG})")


def g_restore_upright_after_dump(ctx):
    """[태성] dump 에서 j6 을 DUMP_J6_ROTATE_DEG 만큼 돌려 기울였던 쓰레기통을, RETURN 전에
    그만큼 반대로 돌려 다시 위를 향하게 되돌린다(j1 포함 나머지 관절은 현재값 유지)."""
    cur = ctx.robot.get_joint_positions()
    restore_deg = []
    for name in ARM_JOINT_NAMES:
        idx = ctx.dof_names.index(name)
        if name == "joint_6":
            restore_deg.append(float(np.degrees(cur[idx])) - DUMP_J6_ROTATE_DEG)
        else:
            restore_deg.append(float(np.degrees(cur[idx])))
    yield from g_ramp_to_joint_positions(ctx, restore_deg, DUMP_RAMP_STEPS)
    print(f"[INFO] c2 쓰레기통 다시 위로 복귀 (j6-={DUMP_J6_ROTATE_DEG})")


def _pick_closest_entry(trash_origin_xy, from_xy):
    """[태성] 쓰레기통 4변(진입점) 중 from_xy 에서 가장 가까운 (chassis_goal_xy, yaw) 반환.
    OFFSET_TRASH_FROM_CHASSIS 는 원점 기준 실측이라 trash_origin_xy(=중심-offset)를 써야 한다."""
    best = None
    for k in range(4):
        theta = k * (np.pi / 2.0)
        goal_xy = trash_origin_xy - rotate_2d(OFFSET_TRASH_FROM_CHASSIS, theta)
        d = float(np.linalg.norm(goal_xy - from_xy))
        if best is None or d < best[2]:
            best = (goal_xy, theta, d)
    return best[0], best[1]


def carter2_mission(ctx):
    """carter2 전체 미션 제너레이터 (4_ main 시퀀스를 yield 화).
    [태성] PICK → DUMP → RETURN(원위치 복귀·내려놓기) → DOCK(도킹 복귀) 4단계."""
    # 진입점/standoff 계산 (4_ 와 동일). trash_xy=중심, trash_origin_xy=원점(중심-offset).
    trash_xy = np.array(TRASH_SPAWN_FIXED)
    trash_origin_xy = trash_xy - TRASH_BBOX_CENTER_OFFSET_XY
    spawn_xy = np.array([C2_START_POSE["x"], C2_START_POSE["y"]])
    chassis_goal_xy, chassis_goal_yaw = _pick_closest_entry(trash_origin_xy, spawn_xy)
    approach_dir = rotate_2d(OFFSET_TRASH_FROM_CHASSIS / np.linalg.norm(OFFSET_TRASH_FROM_CHASSIS), chassis_goal_yaw)
    standoff_xy = chassis_goal_xy - approach_dir * FINAL_APPROACH_DISTANCE
    standoff_yaw = float(np.arctan2(approach_dir[1], approach_dir[0]))

    # 쓰레기통 바닥 안착 대기
    for _ in range(SETTLE_STEPS):
        yield

    sync_rmpflow_base_pose(ctx)
    # PICK 구간 주행
    yield from g_run_nav_leg(ctx, standoff_xy, standoff_yaw, chassis_goal_xy, chassis_goal_yaw, "PICK")
    if not simulation_app.is_running():
        return
    sync_rmpflow_base_pose(ctx)

    # 파지 시퀀스 (3_/4_ 와 동일)
    ctx.status = "PICK: 쓰레기통 파지 시퀀스"
    yield from g_ramp_to_joint_positions(ctx, TARGET_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    grasp_position = get_prim_world_position(ctx.tool0_path)
    grasp_orientation = get_world_orientation_wxyz(ctx.tool0_path)
    print(f"[INFO] c2 목표 관절 도달. tool0={grasp_position}")

    move_dir = rotate_vector_by_quat(grasp_orientation, np.array([0.0, 0.0, 1.0]))
    move_dir /= np.linalg.norm(move_dir)

    trash_now = get_prim_world_bbox_center(C2_TRASH_CAN_PRIM)
    to_trash = trash_now - grasp_position
    depth = float(np.dot(to_trash, move_dir))
    lateral_vec = to_trash - depth * move_dir
    lateral_err = float(np.linalg.norm(lateral_vec))
    print(f"[INFO] c2 실측 쓰레기통={trash_now}, 수직오차={lateral_err:.3f}m")
    if LATERAL_CORRECTION_MIN < lateral_err <= LATERAL_CORRECTION_MAX:
        yield from g_move_to_pose(ctx, grasp_position + lateral_vec, grasp_orientation, "좌우/높이 보정")
        grasp_position = get_prim_world_position(ctx.tool0_path)
    elif lateral_err > LATERAL_CORRECTION_MAX:
        print(f"[WARN] c2 수직오차 과대({lateral_err:.3f}m) → 보정 생략")

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
            print(f"[INFO] c2 파지 성공 (creep {creep_step + 1})")
            break
    grasp_position = current_target
    plug_path = create_plug_at_world_pos(C2_TRASH_CAN_PRIM, get_prim_world_position(ctx.tool0_path))
    print("[CHECKPOINT] c2 gripper", "CLOSED" if gripped_ok else "파지 실패")

    lift_target = grasp_position + LIFT_OFFSET
    yield from g_move_to_pose(ctx, lift_target, grasp_orientation, "들어올리기")
    yield from g_hold_pose(ctx, lift_target, grasp_orientation, GRASP_HOLD_STEPS)

    gap = float(np.linalg.norm(get_prim_world_position(plug_path) - get_prim_world_position(ctx.tool0_path)))
    print(f"[RESULT] c2 그리퍼-plug 간격={gap:.4f}m ({'성공' if gap < 0.03 else '실패 의심'})")

    # j1 tuck
    cur = ctx.robot.get_joint_positions()
    tuck_deg = []
    for name in ARM_JOINT_NAMES:
        idx = ctx.dof_names.index(name)
        tuck_deg.append(TUCK_J1_DEG if name == "joint_1" else float(np.degrees(cur[idx])))
    yield from g_ramp_to_joint_positions(ctx, tuck_deg, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    print(f"[INFO] c2 j1 tuck→{TUCK_J1_DEG} 완료")

    # DUMP 구간 : big_trash 로 이동
    face_dir = -BIG_TRASH_APPROACH_DIR
    big_yaw = float(np.arctan2(face_dir[1], face_dir[0]))
    big_standoff = BIG_TRASH_POSITION_XY + BIG_TRASH_APPROACH_DIR * BIG_TRASH_STANDOFF_DISTANCE
    big_goal = BIG_TRASH_POSITION_XY + BIG_TRASH_APPROACH_DIR * BIG_TRASH_FINAL_DISTANCE
    yield from g_run_nav_leg(ctx, big_standoff, big_yaw, big_goal, big_yaw, "DUMP")
    if not simulation_app.is_running():
        return
    yield from g_dump_into_big_trash(ctx)

    # ── [태성] 덤프 후 : 쓰레기통 다시 위로 → 후진 + 180 회전 → RETURN ──
    yield from g_restore_upright_after_dump(ctx)
    yield from g_drive_straight_open_loop(ctx, POST_DUMP_BACKUP_DISTANCE, C2_ARTICULATION_ROOT,
                                          FINAL_APPROACH_SPEED, reverse=True)
    post_dump_yaw = wrap_pi(get_chassis_yaw(C2_ARTICULATION_ROOT) + np.pi)
    yield from g_rotate_in_place(ctx, post_dump_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, C2_ARTICULATION_ROOT)
    print(f"[APPROACH:DUMP] c2 {POST_DUMP_BACKUP_DISTANCE:.2f}m 후진 + 180 회전 → RETURN 시작")

    # ── RETURN : 원위치로 복귀해서 내려놓기 (현재 위치 기준 최근접 진입점 재선택) ──
    ctx.status = "RETURN: 원위치 복귀 이동"
    ret_from = get_prim_world_position(C2_ARTICULATION_ROOT)[:2]
    ret_goal_xy, ret_goal_yaw = _pick_closest_entry(trash_origin_xy, ret_from)
    ret_dir = rotate_2d(OFFSET_TRASH_FROM_CHASSIS / np.linalg.norm(OFFSET_TRASH_FROM_CHASSIS), ret_goal_yaw)
    ret_standoff_xy = ret_goal_xy - ret_dir * FINAL_APPROACH_DISTANCE
    ret_standoff_yaw = float(np.arctan2(ret_dir[1], ret_dir[0]))
    yield from g_run_nav_leg(ctx, ret_standoff_xy, ret_standoff_yaw, ret_goal_xy, ret_goal_yaw, "RETURN")
    if not simulation_app.is_running():
        return
    sync_rmpflow_base_pose(ctx)

    # dump 로 뒤틀린 손목(j6)에서 바로 Cartesian 이동하면 차체와 충돌 위험(차체는 RMPflow 장애물 미등록)
    # → 먼저 검증된 TARGET_JOINTS_DEG 관절자세로 안전복귀 후 내려놓기.
    ctx.status = "RETURN: 안전 관절복귀 후 내려놓기"
    yield from g_ramp_to_joint_positions(ctx, TARGET_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    # 파지 시점 pose 로 내려가 그 자리에 내려놓기(진입점 매번 달라 growing tolerance).
    yield from g_move_to_pose(ctx, grasp_position, grasp_orientation, "원위치 내려놓기",
                              growing_tolerance_max=RETURN_PLACE_GROWING_TOLERANCE_MAX)
    yield from g_hold_pose(ctx, grasp_position, grasp_orientation, GRASP_HOLD_STEPS)
    ctx.gripper.open()
    for _ in range(GRASP_HOLD_STEPS):
        yield
    print(f"[INFO] c2 Surface Gripper 개방 (is_closed={ctx.gripper.is_closed()})")
    # 내려놓은 뒤 팔 위로 후퇴 → 쓰레기통과 충돌 회피
    retract_target = grasp_position + LIFT_OFFSET
    yield from g_move_to_pose(ctx, retract_target, grasp_orientation, "내려놓은 후 후퇴")
    yield from g_hold_pose(ctx, retract_target, grasp_orientation, GRASP_HOLD_STEPS)
    # 팔 홈 자세(0도) 복귀 → 주행 안전
    yield from g_ramp_to_joint_positions(ctx, [0.0] * 6, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    print("[INFO] c2 팔 홈(0deg) 복귀 완료")

    # ── 후진 + 180 회전 → DOCK ──
    yield from g_drive_straight_open_loop(ctx, POST_RETURN_BACKUP_DISTANCE, C2_ARTICULATION_ROOT,
                                          FINAL_APPROACH_SPEED, reverse=True)
    post_ret_yaw = wrap_pi(get_chassis_yaw(C2_ARTICULATION_ROOT) + np.pi)
    yield from g_rotate_in_place(ctx, post_ret_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, C2_ARTICULATION_ROOT)
    print(f"[APPROACH:RETURN] c2 {POST_RETURN_BACKUP_DISTANCE:.2f}m 후진 + 180 회전 → DOCK 시작")

    # ── DOCK : docking_station_02(C2_START_POSE)로 주차 복귀. 물체 안 다루니 standoff==goal. ──
    ctx.status = "DOCK: 도킹 복귀"
    dock_goal_xy = np.array([C2_START_POSE["x"], C2_START_POSE["y"]])
    dock_yaw = float(np.radians(C2_START_POSE["yaw_deg"] + 180.0))   # [태성] 스폰 반대(+180) 방향 주차
    yield from g_run_nav_leg(ctx, dock_goal_xy, dock_yaw, dock_goal_xy, dock_yaw, "DOCK")
    if not simulation_app.is_running():
        return

    print("[INFO] c2 미션 완료(파지+덤프+원위치복귀+도킹). 이후 대기.")
    ctx.status = "미션 완료(4단계: PICK+DUMP+RETURN+DOCK) — 대기"
    while simulation_app.is_running():
        yield


def build_carter2_control(my_world):
    """carter2 SingleManipulator 를 scene 에 등록(world.reset 전 호출) → (robot, ee_path, tool0_path)."""
    stage = omni.usd.get_context().get_stage()
    tune_arm_drives(C2_ARM_ROOT)

    def find_by_name(root, name):
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
            if prim.GetName() == name:
                return str(prim.GetPath())
        return None

    ee_path = find_by_name(C2_ARM_ROOT, C2_EE_LINK_NAME)
    tool0_path = find_by_name(C2_ARM_ROOT, "tool0")
    if ee_path is None or tool0_path is None:
        raise RuntimeError(f"c2 link_6/tool0 못 찾음 (ee={ee_path}, tool0={tool0_path})")

    robot = my_world.scene.add(SingleManipulator(
        prim_path=C2_ARTICULATION_ROOT, name="carter2_m0609",
        end_effector_prim_path=ee_path, gripper=None))
    return robot, ee_path, tool0_path


# ════════════════════════════════════════════════════════════════════════════
#  H. main — 두 로봇 협조 루프
# ════════════════════════════════════════════════════════════════════════════
def main():
    # 로봇별 격리 토글(디버그) : ENABLE_C1=0 이면 carter1 스킵, ENABLE_C2=0 이면 carter2 스킵.
    # 통합 크래시 원인 좁힐 때 한 로봇씩 켜서 확인. 기본은 둘 다 on.
    en_c1 = os.environ.get("ENABLE_C1", "1") == "1"
    en_c2 = os.environ.get("ENABLE_C2", "1") == "1"
    print(f"[CFG] ENABLE_C1={en_c1} ENABLE_C2={en_c2}")

    my_world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)
    stage = omni.usd.get_context().get_stage()

    build_env()
    if en_c1 and not build_carter1():
        simulation_app.close(); return
    if en_c1 and not build_nozzle_dock():
        simulation_app.close(); return
    if en_c2 and not build_carter2():
        simulation_app.close(); return

    c1_robot = c1_ee_path = c1_tool0_path = None
    if en_c1:
        setup_nozzle_surface_gripper(stage)      # 그리퍼 D6 조인트 재튜닝(reset 전 authoring)
        boost_drive_limits(C1_CARTER_PRIM)
        # carter1 제어체 추가(reset 전에 scene.add 필요) — 내부에서 tune_arm_drives(C1_ARM) 수행
        c1_robot, c1_ee_path, c1_tool0_path = build_carter1_control(my_world)
    c2_robot = c2_ee_path = c2_tool0_path = None
    if en_c2:
        boost_drive_limits(f"{C2_SCOPE}/Nova_Carter_ROS")
        # carter2 제어체 추가(reset 전에 scene.add 필요)
        c2_robot, c2_ee_path, c2_tool0_path = build_carter2_control(my_world)

    my_world.reset()
    for _ in range(5):
        my_world.step(render=False)

    # ── ROS ──
    rclpy.init()
    ros_node = rclpy.create_node("dual_sg_tool_changer_controller")
    clock_pub = ros_node.create_publisher(Clock, "/clock", 10)     # 전역 단일 /clock

    # ── 웹 HMI 긴급정지 인지(/robot/command 직접 구독) ──
    #   carter2(폐기물)는 최종접근·덤프·도킹 구간을 이 스크립트가 cmd_vel 로 직접 몰기 때문에,
    #   미션노드의 stop_wheels(0) 만으로는 이 스크립트가 계속 덮어써 안 멈춘다. → 여기서 estop 을
    #   직접 받아 해당 로봇 제너레이터를 '동결'하고 cmd_vel 0 을 강제한다(메인 루프 아래 참조).
    #   carter1 은 미션노드가 start_sweep=False 를 쏘고 g_spray_sweep 이 이를 보고 스스로 STANDBY 로
    #   재동기화하므로 동결하지 않는다(동결하면 재시작 시 미션과 어긋남) — 안전용 cmd_vel 0 만 보조.
    #   HMI robotId 매핑 : disinfect→carter1, waste→carter2, null→둘 다.
    estop_flags = {"carter1": False, "carter2": False}

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
        if cmd == "EMERGENCY_STOP":
            for t in targets:
                estop_flags[t] = True
            print(f"[ESTOP] 긴급정지 수신 → 동결 {targets}")
        elif cmd == "START":
            for t in targets:
                estop_flags[t] = False
            print(f"[ESTOP] START 수신 → 해제 {targets}")

    ros_node.create_subscription(String, "/robot/command", _on_hmi_command, 10)
    print("[ROS] /robot/command 구독(긴급정지 인지) — disinfect→carter1, waste→carter2")

    c1_ctx = None
    c1_cmd_pub = None
    c1_gen = None
    c1_done = not en_c1
    c2_cmd_pub = None
    c2_gen = None
    c2_done = not en_c2

    try:
        # carter1 제어 초기화 (Surface Gripper + ToolChangerController + 소독 미션 제너레이터)
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
            c1_gripper, c1_tool_changer = build_carter1_toolchanger(stage, c1_ee_path)

            c1_cmd_pub = ros_node.create_publisher(Twist, C1_CMD_VEL, 10)
            c1_sweep_done_pub = ros_node.create_publisher(Bool, C1_SWEEP_DONE, 10)
            c1_sweep_state = {"req": None}
            ros_node.create_subscription(Bool, C1_START_SWEEP,
                                         lambda m: c1_sweep_state.__setitem__("req", bool(m.data)), 10)
            print(f"[ROS] carter1 : pub {C1_SWEEP_DONE} + {C1_CMD_VEL}, sub {C1_START_SWEEP}")
            c1_ctx = C1Ctx(my_world, c1_robot, c1_dof, c1_ee_path, c1_tool0_path, c1_gripper,
                           ros_node, c1_cmd_pub, c1_sweep_done_pub, c1_sweep_state)
            c1_ctx.tool_changer = c1_tool_changer
            c1_gen = g_carter1_mission(c1_ctx)

        # carter2 제어 초기화
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
            c2_rmpflow = RMPFlowController(name="carter2_cspace", robot_articulation=c2_robot,
                                           urdf_path=C2_NO_GRIPPER_URDF)
            sync_base_prim = omni.usd.get_context().get_stage().GetPrimAtPath(f"{C2_ARM_ROOT}/base_link")
            _m = omni.usd.get_world_transform_matrix(sync_base_prim)
            _t = _m.ExtractTranslation(); _q = _m.ExtractRotationQuat(); _im = _q.GetImaginary()
            c2_rmpflow.rmp_flow.set_robot_base_pose(
                robot_position=np.array([_t[0], _t[1], _t[2]]),
                robot_orientation=np.array([_q.GetReal(), _im[0], _im[1], _im[2]]))
            # ground plane 은 컨트롤러가 아니라 내부 정책(rmp_flow)에 등록(4_ 는 컨트롤러에 호출 →
            # add_ground_plane 없어 AttributeError. RmpFlow 정책엔 있음). 시그니처차 대비 방어.
            try:
                c2_rmpflow.rmp_flow.add_ground_plane(prim_path=C2_GROUND_PLANE, z_position=0.0)
                print(f"[RMP] c2 ground plane 등록: {C2_GROUND_PLANE}")
            except TypeError:
                try:
                    c2_rmpflow.rmp_flow.add_ground_plane()
                    print("[RMP] c2 ground plane 등록(무인자)")
                except Exception as e:
                    print(f"[RMP][WARN] ground plane 등록 실패({e}) → 스킵(검증 파지자세는 바닥 안 침범)")
            except Exception as e:
                print(f"[RMP][WARN] ground plane 등록 실패({e}) → 스킵(검증 파지자세는 바닥 안 침범)")

            c2_gripper = SurfaceGripper(end_effector_prim_path=c2_ee_path,
                                        surface_gripper_path=C2_SURFACE_GRIPPER)
            c2_gripper.initialize()

            c2_goal_pub = ros_node.create_publisher(PoseStamped, C2_NAV_GOAL, 10)
            c2_cmd_pub = ros_node.create_publisher(Twist, C2_CMD_VEL, 10)
            c2_pick_state = {"start": False}
            ros_node.create_subscription(Bool, C2_START_PICK,
                                         lambda m: c2_pick_state.__setitem__("start", bool(m.data)), 10)
            print(f"[ROS] carter2 : pub {C2_NAV_GOAL} + {C2_CMD_VEL}, sub {C2_START_PICK}")

            c2_ctx = C2Ctx(my_world, c2_robot, c2_rmpflow, c2_dof, c2_tool0_path, c2_gripper,
                           ros_node, c2_goal_pub, c2_cmd_pub, c2_pick_state)
            c2_gen = carter2_mission(c2_ctx)

        print("\n[RUN] Play ▶ : carter1(소독 툴체인지+스윕) + carter2(폐기물 nav-pick) 동시 구동.\n"
              "      두 로봇은 같은 루프에서 동시에 돈다. 각자 '자기 미션'이 명령해야 실제로 움직인다:\n"
              "        carter1 → Nav2(carter1) + spray_waypoint_mission -p namespace:=carter1\n"
              "                  (첫 웨이포인트=노즐 거치대 → 첫 start_sweep 이 노즐 파지를 트리거)\n"
              "        carter2 → Nav2(carter2) + trash_can_nav_pick_mission -p namespace:=carter2\n"
              "      ~5초마다 [HB] 하트비트로 두 미션 상태를 출력한다.\n")

        hb = 0
        step_i = 0
        while simulation_app.is_running():
            # ★성능 : 렌더를 RENDER_EVERY 스텝마다 1번(물리/제어/clock 은 매 스텝). GPU 부하 ~1/N.
            step_i += 1
            my_world.step(render=(step_i % RENDER_EVERY == 0))

            # 전역 /clock 발행 (한 번만)
            t = float(my_world.current_time)
            cmsg = Clock(); cmsg.clock.sec = int(t); cmsg.clock.nanosec = int(round((t - int(t)) * 1e9))
            clock_pub.publish(cmsg)
            rclpy.spin_once(ros_node, timeout_sec=0.0)

            if not my_world.is_playing():
                continue

            # 두 미션 제너레이터를 매 스텝 한 스텝치씩 전진(협조 루프, 서로 블로킹 안 함).
            # carter1 : 긴급정지여도 계속 전진시킨다(그래야 g_spray_sweep 이 start_sweep=False 를
            #   보고 스스로 멈추고 STANDBY 로 재동기화). 추가로 안전용 cmd_vel 0 을 덮어씀.
            if not c1_done:
                try:
                    next(c1_gen)
                except StopIteration:
                    c1_done = True
                    print("[C1] 미션 제너레이터 종료")
                if estop_flags["carter1"] and c1_cmd_pub is not None:
                    c1_cmd_pub.publish(Twist())

            # carter2 : 긴급정지면 제너레이터를 '동결'(전진 안 함)하고 cmd_vel 0 강제 → 이 스크립트가
            #   최종접근/덤프/도킹 구간에서 forward cmd_vel 로 덮어쓰던 것을 차단. START 로 해제되면
            #   같은 지점에서 이어서 재개(미션노드가 Isaac 재발행 goal 을 다시 릴레이해 재동기화).
            if not c2_done:
                if estop_flags["carter2"]:
                    if c2_cmd_pub is not None:
                        c2_cmd_pub.publish(Twist())
                else:
                    try:
                        next(c2_gen)
                    except StopIteration:
                        c2_done = True
                        print("[C2] 미션 제너레이터 종료")

            # ── 하트비트 : 두 로봇 미션이 모두 살아 도는지 + 각자 무엇을 기다리는지 (~5초) ──
            hb += 1
            if hb % 300 == 0:
                if not en_c1:
                    c1s = "(비활성 ENABLE_C1=0)"
                elif c1_done:
                    c1s = "완료(제너레이터 종료)"
                else:
                    c1s = c1_ctx.status
                if not en_c2:
                    c2s = "(비활성 ENABLE_C2=0)"
                elif c2_done:
                    c2s = "완료(제너레이터 종료)"
                else:
                    c2s = c2_ctx.status
                print(f"[HB] carter1 = {c1s}\n     carter2 = {c2s}")

    except Exception:
        import traceback
        print("\n[FATAL] main 루프 예외 — 아래 파이썬 트레이스백이 진짜 원인입니다:\n")
        traceback.print_exc()
    finally:
        # 정지 명령 + 정돈된 종료(순서: 타임라인 정지 → ROS 종료 → 시뮬 종료). 셧다운 크래시 완화.
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
