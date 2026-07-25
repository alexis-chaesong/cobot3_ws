"""
15_single_robot_tool_changer_integrated.py  ★단일로봇 툴체인저 통합 — 3단계 1/5★
================================================================================
"Surface Gripper(이하 SG)를 이용한 Tool Changer 개발" 설계문서(2026-07-24) + 검증
1단계(src/isaac_wiping_task/2_tool_changer_nozzle_demo.py, 파지 gap 4.4mm) +
2단계(같은 파일, carter1과 동일 조준 IK가 실측 오프셋으로 풀림) 결과를 실제 hospital 씬 +
carter2 전체 미션(13_multi_robot_integrated.py 의 PICK/DUMP/RETURN/DOCK)과 합치는 3단계.

★번호 15인 이유★ : 설계문서 원안 파일명은 14_... 였으나, 이 저장소에서 14번대는 별도
YOLO 프로젝트가 예약 중이라 15번을 씀.

carter1(소독 노즐팔)은 씬에서 완전히 제거한다. carter2(Nova Carter + 맨몸 m0609 + 기존
mop_surface_gripper + 쓰레기통) 한 대가, 평소엔 그리퍼로 쓰레기통을 처리하다가 사용자가
"분사" 작업을 선택하면 노즐 거치대(Nozzle Dock)에서 노즐을 집어 소독까지 하는 통합 미션을
수행한다(고정 순차 미션이 아니라 **작업 선택식** — 확정된 요구사항).

이 파일은 5단계로 나눠 작성/검증한다(이 저장소 관례: 작은 단위 → 헤드리스 검증 → 통합):
  [1/5, 현재] 씬만 : carter1 제거 + carter2 + Nozzle Dock(hold_joint 포함). main() 은
      아직 미션 FSM 없이 스폰 결과만 헤드리스로 확인하는 최소 버전.
  [2/5] 툴체인지 제너레이터 (g_tool_change_grasp/release)
  [3/5] 분사 스윕 제너레이터 (g_spray_sweep, Sweeper/q_of_s/SprayFX + 실측 오프셋 조준)
  [4/5] task_select 토픽(/carter2/task_select, String "spray"|"trash") FSM 병합
  [5/5] 통합 헤드리스 스모크

⚠ 라이브 검증 필요(오프라인 헤드리스만 확인) : Nav2/ROS 미션노드/맵이 뜬 라이브 환경이
이번 세션엔 없어, 실제 주행(PICK/DUMP/RETURN/DOCK, 분사 웨이포인트 이동)은 코드 경로만
확인하고 완주 검증은 다음 라이브 세션에서 해야 한다. 실행은 기존 3-스크립트 관례를 따름 :
  1) run_isaac.sh 대신 이 스크립트를 직접 : python.sh isaacpjt/M0609/15_single_robot_tool_changer_integrated.py
     (또는 ISAAC_HEADLESS=1 로 헤드리스)
  2) Nav2(단일로봇, carter2 namespace) : carter_navigation.launch.py 또는 기존 멀티런치 재사용
  3) 미션 : trash_can_nav_pick_mission.py 는 변경 없이 재사용(좌표 무관 제너릭 goal 포워더라
     트래시/분사 어느 leg든 그대로 씀). namespace:=carter2.

ROS 사이드는 신규 작성 불필요(계획 단계에서 확인) : trash_can_nav_pick_mission.py 가 이미
"Isaac이 퍼블리시하는 goal을 Nav2에 전달하고 결과만 돌려주는" 제너릭 포워더라, 분사 웨이포인트
이동에도 기존 /carter2/trash_can_nav_goal + /carter2/start_pick 채널을 그대로 쓴다.
HMI(hmi_link.py, disinfect=carter1/waste=carter2)는 이번 단계에서 건드리지 않는다(범위 밖).
13_multi_robot_integrated.py 와 기존 2-로봇 HMI 시스템은 그대로 유지(이 파일은 병행 추가).
================================================================================
"""
import os

from isaacsim import SimulationApp

# 기본 GUI. 헤드리스 스모크 테스트 시 ISAAC_HEADLESS=1.
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
from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, Sdf, Gf, Vt

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
import isaacsim.robot_motion.motion_generation as mg

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

# 툴체인지(1·2단계 검증 모듈) : src/isaac_wiping_task 재사용.
_TOOL_CHANGER_DIR = str(_WS_ROOT / "src" / "isaac_wiping_task")
if _TOOL_CHANGER_DIR not in sys.path:
    sys.path.insert(0, _TOOL_CHANGER_DIR)
import surface_gripper_utils  # noqa: E402
from tool_changer import ToolChangerController  # noqa: E402


# ════════════════════════════════════════════════════════════════════════════
#  A. 공통 경로/상수
# ════════════════════════════════════════════════════════════════════════════
# carter_navigation 은 이 레포 밖의 별도 워크스페이스에 있어 상대경로로 못 잡는다 —
# CARTER_NAV_WS env var 로 override 가능(기본값 = 팀 관례 위치).
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

# 렌더 성능 노브(13_ 과 동일 메커니즘) — 로봇 1대뿐이라 부담은 덜하지만 그대로 유지.
RENDER_EVERY = 2
CAM_RENDER_W = 640
CAM_RENDER_H = 400

# Nova Carter 내부 ActionGraph node_namespace. carter2 하나뿐이지만, 기존
# trash_can_nav_pick_mission.py 가 namespace:=carter2 로 이미 짝지어져 있어 그대로 유지
# (재작성 없이 재사용하기 위함 — 계획 문서 "ROS 사이드는 신규 작성 불필요" 참고).
NS_CARTER2 = "carter2"

C2_SCOPE = "/World/Carter2"


# ════════════════════════════════════════════════════════════════════════════
#  B. carter2 (폐기물 + 툴체인저) 상수 — 13_multi_robot_integrated.py 와 동일 값
# ════════════════════════════════════════════════════════════════════════════
C2_ARTICULATION_ROOT = f"{C2_SCOPE}/Nova_Carter_ROS/chassis_link"
C2_ARM_ROOT = f"{C2_SCOPE}/m0609"
C2_EE_LINK_NAME = "link_6"
C2_TRASH_CAN_PRIM = f"{C2_SCOPE}/small_trash_can_body"
C2_SURFACE_GRIPPER = f"{C2_ARM_ROOT}/{C2_EE_LINK_NAME}/mop_surface_gripper"
C2_GROUND_PLANE = f"{C2_SCOPE}/GroundPlane"
C2_EXTRA_PHYSICS = f"{C2_SCOPE}/PhysicsScene"

C2_NO_GRIPPER_URDF = str(_WS_ROOT / "src" / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf")

# 차동구동 클램프 상향(U턴 병목 해소, 6-15) — 원래 carter1 절(B)에 있었으나 carter2 에도 그대로 적용.
DRIVE_MAX_ANG_SPEED = 2.5
DRIVE_MAX_ANG_ACCEL = 6.0
DRIVE_MAX_LIN_SPEED = 1.2

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
C2_START_POSE = dict(x=16.66290495232035, y=-0.0029517927591273807, yaw_deg=0.0)

C2_NAV_GOAL = f"/{NS_CARTER2}/trash_can_nav_goal"
C2_START_PICK = f"/{NS_CARTER2}/start_pick"
C2_CMD_VEL = f"/{NS_CARTER2}/cmd_vel"
C2_TASK_SELECT = f"/{NS_CARTER2}/task_select"          # [3단계 신규] String "spray"|"trash"

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

# [사용자 제안 — "정밀 도킹 및 주행 궤도 고도화 계획서"에서 마커/비전 부분은 빼고 제어 로직만 반영]
# g_run_nav_leg 최종접근(회전→직진 오픈루프→회전)은 직진 거리를 "진입 시점 1회"만 재고 이후엔
# 다시 안 재서, 초기 정렬 오차가 그대로 남을 수 있다. 처음엔 이걸 "매 스텝 계속 재측정하는 폐루프
# 서보"로 고쳤는데, 목표 수 cm 근방에서 "목표 방향(bearing)"이 위치 잡음(ground truth 라 센서
# 노이즈는 없지만 거리가 0에 가까워지면 atan2 자체가 특이점에 가까워짐)에 극도로 민감해져 chattering이
# 나고 자세(yaw) 보정이 아예 안 걸리는 문제가 실측으로 확인됐다(라이브 SPRAY_RETURN, yaw 71.4도 방치).
# → 계획서의 "Nudge(사전 정렬) + One-Shot(정지 후 1회 측정 → 그 값으로만 이동, 이동 중엔 재측정 안 함)"
# 패턴으로 교체: 이동 중엔 절대 목표까지의 벡터를 다시 안 재고, 완전히 멈춰서 측정 → 계산된 만큼만
# 이동 → 다시 멈춰서 재측정, 을 몇 차례 반복(discrete/저빈도 폐루프)해 근본적으로 chattering을 없앤다.
DOCK_NUDGE_DISTANCE = 0.10     # 캐스터 정렬용 사전 전후진 거리(계획서 Phase 2)
DOCK_NUDGE_SPEED = 0.10
DOCK_ONESHOT_MAX_ITERS = 3     # 정지-측정-이동 반복 최대 횟수
DOCK_ONESHOT_POS_TOL = 0.02
DOCK_ONESHOT_DRIVE_SPEED = 0.08   # 최종 보정 구간은 느리게(오버슈트 방지). 최종 자세 허용치는
                                  # g_rotate_in_place 가 쓰는 FINAL_ROTATE_TOLERANCE_RAD 재사용.
DOCK_ONESHOT_SETTLE_STEPS = 20    # 측정 전 완전 정지 대기(모션 잔진동 없이 깨끗한 1회 측정 확보)


# ════════════════════════════════════════════════════════════════════════════
#  C. 노즐 거치대(Nozzle Dock) 상수 — 2단계 검증(2_tool_changer_nozzle_demo.py) 이식
# ════════════════════════════════════════════════════════════════════════════
NOZZLE_SOURCE_PRIMPATH = "/World/m0609/nozzle_base_link"   # tool0_to_nozzle 조인트 밖(형제) — 안 딸려옴
NOZZLE_DOCK_SCOPE = "/World/NozzleDock"
NOZZLE_TOOL_PATH = f"{NOZZLE_DOCK_SCOPE}/nozzle_tool"
NOZZLE_TCP_PATH = f"{NOZZLE_TOOL_PATH}/nozzle_tcp"
NOZZLE_HOLD_JOINT_PATH = f"{NOZZLE_DOCK_SCOPE}/hold_joint"
NOZZLE_HOLD_ANCHOR_PATH = f"{NOZZLE_DOCK_SCOPE}/hold_anchor"
NOZZLE_RADIUS = 0.0315   # 로컬 bbox 실측(2단계) — 정착/충돌 여유 추정에만 참고용
NOZZLE_LENGTH = 0.142

# [재배치 — 다시 원위치] 한때 carter2 홈(C2_START_POSE) 바로 앞(0.35m)으로 옮겼었으나(파지/반납
# nav-leg 왕복을 줄이려는 목적), 사용자 지적대로 어차피 분사 작업은 항상 이 거치대를 오가야 하므로
# 왕복 자체는 불가피하고, 오히려 홈 바로 옆에 두면 나중에 씬에 실제 거치대(받침대) 모델을 세웠을 때
# carter2가 홈에서 대기/도킹 정밀보정(g_precise_dock_approach)을 하다가 그 받침대와 부딪힐 위험이
# 있다 — 그래서 원래 설계문서 제안대로 옛 carter1 스폰(18.5,0)으로 되돌린다(홈에서 1.84m 떨어진,
# 이미 빈 공간으로 검증된 위치). 팔 작업용 접근 지점(DOCK_APPROACH_XY)은 이 거치대 기준으로 별도
# 정의(더 아래 참고) — carter2 홈과는 이제 무관하다.
NOZZLE_DOCK_XY = np.array([18.5, 0.0])
NOZZLE_DOCK_HEIGHT = 0.65

# Surface Gripper 튜닝. [GUI 확인 후 재조정] D6 조인트는 이미 "그리퍼 쪽 결착점(고정) + 물체
# 원점(결착점)을 grip 시 서로 당겨 맞추는" 메커니즘이다 — 그런데 GRIP_TRAVEL(당겨 맞출 수 있는
# 최대범위)이 4mm 로 좁아서, IK 접근 오차(~5.6mm 실측)를 다 못 당기고 그만큼 중심이 어긋나 보였다.
# 새 메커니즘을 만들 필요 없이 travel 을 실측 오차보다 넉넉히 늘리고 clearance(항상 남기는 최소
# 여유)는 0에 가깝게 줄여 "완전히 중심 맞춰 고정"되도록 한다.
MAX_GRIP_DISTANCE = 0.04
GRIP_DRIVE_STIFFNESS = 5000.0
GRIP_DRIVE_DAMPING = 100.0
CLEARANCE_OFFSET = 0.0005
GRIP_TRAVEL = 0.015

# 조준 IK 상수(carter1 과 동일 — 13_multi_robot_integrated.py 참고)
WALL_X = 0.575
AIM_Y = 0.0
Z_LOW = 0.12
Z_HIGH = 0.80

# 툴체인지 접근/파지 상수 (2_tool_changer_nozzle_demo.py 와 동일)
# [GUI 확인 후 수정] 거치대 orientation 을 spray_orientation_quat() 에 맞췄더니 노즐이 "수평"으로
# 매달려 보기 안 좋았음(조준용 orientation 이라 로컬 Z가 world +X를 향함). 파지 orientation과 조준
# orientation이 반드시 같을 필요는 없음(파지 직후 오프셋을 실측해서 쓰므로) — 그래서 거치/파지는
# 2단계 스탠드얼론 데모에서 이미 검증한 "수직 매달기"(팁 아래, (0,1,0,0)→로컬Z가 world -Z) 로 되돌린다.
TC_APPROACH_ORIENTATION = np.array([0.0, 1.0, 0.0, 0.0])
TC_EE_OFFSET = np.array([0.0, 0.0, 0.15])       # 접근 시 위 여유공간(2단계 검증값)
TC_GRASP_CLEARANCE = np.zeros(3)
TC_FINGERTIP_OFFSET_FROM_TOOL0 = np.zeros(3)    # carter2 도 맨몸 팔이라 fingertip(link_6)≈tool0
TC_GRASP_SETTLE_STEPS = 30
TC_REDOCK_SETTLE_STEPS = 20
# [GUI 확인 후 재설계] 거치대는 고정 위치라 매 스텝 반응형 IK(RMPflow)를 쓸 이유가 없었다 — 실제로
# 파지 후 link_6-노즐 오프셋이 2.3cm 남는 문제를 "수렴 예산 부족"으로 보고 g_move_to_pose 의 예산을
# 400→900으로 늘렸더니 오히려 급격히 악화됐다(잔여오차 최대 167mm, 파지 실패 — 반응형 IK가 오래
# 돌수록 진동/발산). 트래시 파지(TARGET_JOINTS_DEG)·분사 스윕(q_of_s)과 동일하게 "IK 한 번 풀어서
# 관절각 확정 → 그 관절각으로 램프"(_tc_solve_and_ramp, LulaKinematicsSolver) 방식으로 교체.
TC_JOINT_RAMP_STEPS = 200

# [경험적 보정 오프셋] 진단으로 확인(URDF-USD 형상 불일치, 근본 한계) : 실제 시뮬레이션 link_6 이
# IK 목표보다 항상 이 정도만큼 "더 간다"(여러 번 실측, 거치대 위치를 바꿔도 동일 — 같은 접근
# orientation/상대기하에서 재현되는 고정 바이어스). IK 에 넘기는 target 에서 미리 이만큼 빼두면
# 실제 도착 지점이 진짜 목표에 더 가까워진다. ★TC_APPROACH_ORIENTATION(수직 매달기) 자세로 접근할
# 때만 유효한 값★ — 조준(spray_orientation_quat) 등 다른 orientation/기하에는 이 값이 안 맞을 수
# 있어 g_spray_sweep 의 별도 IK 호출에는 적용하지 않는다.
TC_TARGET_BIAS_COMPENSATION = np.array([0.006, -0.0003, 0.0])
# [시도했으나 기각] 반납 전용 정적 보정값을 역산해 따로 둬봤으나(파지/반납이 6축 여유자유도로 다른
# 관절해에 수렴해 바이어스 방향이 다르다는 가설) 실측 결과 오히려 악화(2.56mm→8.3mm) — 반납 쪽
# 편차는 목표를 살짝만 바꿔도 IK 가 다른 해로 수렴해 정적 상수로 안 잡힌다. g_tool_change_release
# 는 대신 파지 직후 실측한 동적 오프셋(tool_changer.fingertip_offset_from_ik_frame)을 쓴다(2.56mm,
# 지금까지 시도 중 최선).


# ════════════════════════════════════════════════════════════════════════════
#  C2. 분사 스윕(WIPE/MOVE) 상수 — carter1(Carter1Spray, 13_multi_robot_integrated.py)과 동일 값.
#     조준 IK 오프셋만 고정상수(NOZZLE_OFFSET) 대신 파지 직후 실측치(ctx.nozzle_tip_offset) 사용.
# ════════════════════════════════════════════════════════════════════════════
IK_URDF_PATH = str(_WS_ROOT / "isaacpjt" / "M0609" / "rmpflow" / "m0609_isaac_sim.urdf")
IK_DESCRIPTION_PATH = str(_WS_ROOT / "isaacpjt" / "M0609" / "rmpflow" / "m0609_description.yaml")
EE_FRAME = "link_6"

J1_INDEX = 0
J1_OFFSET = -np.pi / 2     # 팔 겨냥 = 로봇 몸체 기준 오른쪽(carter1 과 동일 트릭 — 180도 회전하면 반대편 벽)
J5_INDEX = 4
J5_FLICK = -0.5

S_CRUISE = 2.0
S_ACCEL = 3.0
S_HOLD_STEPS = 6
STOW_Q = np.array([0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0])
MAX_JOINT_STEP = 0.06
# [GUI 확인 후 추가] 파지 직후 자세 → 스윕 저점(q_of_s(-1.0)) 진입을 부드럽게(3~4초).
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

# 분사 파티클 FX (경량 탄도 풀) — carter1과 동일.
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
#  D. namespace 유틸 + 수학 유틸 (13_multi_robot_integrated.py 와 동일)
# ════════════════════════════════════════════════════════════════════════════
def set_carter_namespace(root_path, ns):
    """13_multi_robot_integrated.py 와 완전히 동일(변경 없음). carter2 하나뿐이지만 기존 ROS
    미션노드(namespace:=carter2)와 짝을 맞추기 위해 그대로 적용한다."""
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
    """13_multi_robot_integrated.py 와 완전히 동일(변경 없음)."""
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
    """carter1(13_multi_robot_integrated.py)과 동일한 조준 EE orientation. carter1은 항상 스폰
    yaw(0도)에 고정된 채로만 이 값을 썼기 때문에 사실상 "챠시(팔 base_link) 로컬 프레임 기준"
    조준방향이다 — 그대로 월드 orientation 으로 써도 됐던 건 그 챠시가 늘 같은 방향만 보고
    있었기 때문. carter2 웨이포인트별로 다른 yaw 로 도착할 때는 aim_orientation_world() 로
    현재 base_quat 와 합성해서 써야 한다."""
    R = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
    return matrix_to_quat_wxyz(R)


def aim_orientation_world(base_quat):
    """★버그 발견·수정★ : g_spray_sweep 이 spray_orientation_quat() 을 챠시 yaw 와 무관하게 항상
    같은 "월드 고정" 방향으로 IK 에 넘기고 있었다 — 위치 목표는 local_to_world() 로 매번 현재
    base_quat 기준으로 정확히 회전되는데, 자세 목표만 회전이 안 돼 서로 어긋난 셈. carter2 가
    항상 같은 yaw(스폰 방향)에서만 스윕했던 carter1 방식으로는 안 드러났지만, carter2 는 웨이포인트
    마다 다른 yaw 로 도착(SPRAY_WP1=90도)하므로 어긋남이 커져 특히 더 뻗어야 하는 Z_HIGH 지점의
    IK 가 안 풀리는 현상으로 실측 재현됨(TC_DEBUG_YAW_DEG=90 헤드리스 테스트, q_high ok=False).
    spray_orientation_quat() 을 "챠시 로컬 기준 조준방향"으로 재해석해 현재 base_quat 로 합성
    (position 과 동일하게 챠시 프레임을 통째로 회전)한 진짜 월드 orientation 을 반환한다."""
    R_world = quat_to_matrix(base_quat) @ quat_to_matrix(spray_orientation_quat())
    return matrix_to_quat_wxyz(R_world)


def local_to_world(base_pos, base_quat, p_local):
    return base_pos + quat_to_matrix(base_quat) @ np.asarray(p_local)


def aim_link6_world_from_offset(base_pos, base_quat, ori_quat, z, tip_offset_link6_frame):
    """carter1의 aim_link6_world() 를 스칼라 Z 오프셋(NOZZLE_OFFSET)에서 3축 벡터로 일반화
    (2단계 2_tool_changer_nozzle_demo.py 에서 이미 검증). tip_offset_link6_frame 은 파지 직후
    실측한 link_6 기준 노즐팁 상대위치(3축)."""
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
    """child 를 parent 프레임에서 본 (상대위치 np3, 상대회전행렬 3x3). test_nozzle_attach.py 와 동일."""
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
    """13_multi_robot_integrated.py 와 완전히 동일(변경 없음)."""
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
    """13_multi_robot_integrated.py 와 완전히 동일(변경 없음)."""
    w = q_wxyz[0]; qv = np.asarray(q_wxyz[1:4], dtype=float); v = np.asarray(v, dtype=float)
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def get_chassis_yaw(prim_path):
    """13_multi_robot_integrated.py 와 완전히 동일(변경 없음)."""
    w, x, y, z = get_world_orientation_wxyz(prim_path)
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _smoothstep(a):
    return a * a * (3.0 - 2.0 * a)


def _ramp_steps_for_distance(distance, max_speed):
    raw = distance / max_speed / PHYSICS_DT
    return int(np.clip(round(raw), MIN_INTERP_STEPS, MAX_INTERP_STEPS))


def boost_drive_limits(carter_prim_path):
    """13_multi_robot_integrated.py 와 완전히 동일(변경 없음). 차동구동 컨트롤러 각속도 클램프
    상향(U턴 병목 해소, 6-15)."""
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


def setup_gui_camera_and_light(stage, target_prim_path):
    """[사용자 요청] GUI 확인할 때마다 카메라/조명을 수동으로 맞추는 게 번거롭다고 해서, Play 직후
    자동으로 m0609 를 비추는 Perspective 카메라 + 기본 조명을 맞춰준다.
    조명은 이 저장소에 이미 있는 패턴(4_v2_mobile_manipulator_trash_can_nav_pick_test.py 의
    ensure_min_scene_light)과 동일하게 DomeLight 를 보강 — hospital 자체 조명이 약해도 항상
    밝게 보이도록 하는 실제 씬 라이트라, 뷰포트 메뉴의 "Lighting Mode" 설정과 무관하게 항상 먹힌다."""
    light_path = "/World/GuiAssistDomeLight"
    if not stage.GetPrimAtPath(light_path).IsValid():
        dome = UsdLux.DomeLight.Define(stage, light_path)
        dome.CreateIntensityAttr(500.0)
        print(f"[GUI] 기본 조명 보강: {light_path}")

    target_pos = get_prim_world_position(target_prim_path)
    eye = target_pos + np.array([1.8, -1.8, 1.3])
    set_camera_view(eye=eye, target=target_pos, camera_prim_path="/OmniverseKit_Persp")
    print(f"[GUI] Perspective 카메라 → {target_prim_path} 조준 (eye={np.round(eye, 2)})")


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
    """XformCommonAPI 우회(6-13) : op 비우고 단일 transform 행렬로 배치. 13_ 와 동일."""
    stage = omni.usd.get_context().get_stage()
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    xf.ClearXformOpOrder()
    m = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), float(yaw_deg)))
    m.SetTranslateOnly(Gf.Vec3d(x, y, z))
    xf.AddTransformOp().Set(m)


def build_carter2():
    """carter2 = move_tash_can.usd(Nova + 그리퍼팔 + 쓰레기통, 이미 병합됨) 전체를 /World/Carter2 스코프로
    참조. 13_multi_robot_integrated.py 와 완전히 동일(변경 없음)."""
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

    trash = stage.GetPrimAtPath(C2_TRASH_CAN_PRIM)
    if trash.IsValid():
        ta = trash.GetAttribute("xformOp:translate")
        if ta and ta.IsValid():
            _origin = np.asarray(TRASH_SPAWN_FIXED) - TRASH_BBOX_CENTER_OFFSET_XY
            ta.Set(Gf.Vec3d(float(_origin[0]), float(_origin[1]), TRASH_SPAWN_Z))
            print(f"[SPAWN] c2 trash 중심 → {TRASH_SPAWN_FIXED} (원점 {_origin.tolist()})")

    chassis2 = stage.GetPrimAtPath(C2_ARTICULATION_ROOT)
    chassis2_m = UsdGeom.Xformable(chassis2).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    _wp = Gf.Transform(chassis2_m).GetTranslation()
    print(f"[SPAWN] c2 chassis world = ({_wp[0]:.3f}, {_wp[1]:.3f}, {_wp[2]:.3f}) "
          f"(목표 {C2_START_POSE['x']:.2f},{C2_START_POSE['y']:.2f})")

    arm2 = stage.GetPrimAtPath(C2_ARM_ROOT)
    if arm2.IsValid():
        MOUNT_OFFSET = Gf.Vec3d(-0.2317, 0.0, 0.5773)
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


def reposition_carter2_near(stage, xy, yaw_deg=0.0):
    """[디버그 전용] carter2 전체(섀시+팔)를 임의 XY/yaw 로 재배치. 툴체인지 제너레이터만
    떼어 테스트할 때(TC_DEBUG_ONLY), Nav2 없이 챠시를 거치대 근처로 순간이동시키기 위함
    (실제 미션에선 g_run_nav_leg 가 이 역할을 함). build_carter2() 의 팔 사전배치 로직과 동일.

    ★주의★ : C2_ARTICULATION_ROOT(chassis_link)에 직접 _place_xform 을 걸면 world 위치가
    엉뚱해진다(헤드리스로 직접 확인: chassis=(18,0) 로 걸었는데 world 는 (34.66,...) — 원래
    스폰값 16.66 이 그대로 남은 채 더해짐). chassis_link 자신의 로컬 xform 은 거의 identity이고
    실제 스폰 위치는 부모 Nova_Carter_ROS 프림의 authoring 에서 온다(move_tash_can.usd 구조) →
    반드시 Nova_Carter_ROS 를 재배치해야 한다."""
    MOUNT_OFFSET = Gf.Vec3d(-0.2317, 0.0, 0.5773)
    carter_prim_path = f"{C2_SCOPE}/Nova_Carter_ROS"
    chassis = stage.GetPrimAtPath(C2_ARTICULATION_ROOT)
    _place_xform(carter_prim_path, float(xy[0]), float(xy[1]), 0.0, float(yaw_deg))
    chassis_m = UsdGeom.Xformable(chassis).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    arm2 = stage.GetPrimAtPath(C2_ARM_ROOT)
    arm2_m = Gf.Matrix4d().SetTranslate(MOUNT_OFFSET) * chassis_m
    xf2 = UsdGeom.Xformable(arm2)
    xf2.ClearXformOpOrder()
    xf2.AddTransformOp().Set(arm2_m)
    print(f"[DEBUG] carter2 재배치 → chassis=({xy[0]:.2f},{xy[1]:.2f}), yaw={yaw_deg:.0f}")


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


def build_nozzle_dock():
    """[3단계 신규] m0609_with_nozzle.usd 의 nozzle_base_link 서브트리만 거치대 경로에 참조.
    tool0_to_nozzle 조인트는 이 서브트리 밖(형제)이라 자동으로 딸려오지 않는다(1단계에서 확인).

    2단계(2_tool_changer_nozzle_demo.py)에서 검증한 "수직 매달기" 방식 그대로 : 장착면(원점)이 위,
    팁이 아래로 노출되게 세워서(TC_APPROACH_ORIENTATION=(0,1,0,0) → 로컬 Z가 world -Z) 임시
    hold_joint(정적 anchor prim에 FixedJoint)로 고정해 두고, 파지 성공 확인 직후
    release_hold_joint() 로 비활성화한다. 파지 시점 orientation(TC_APPROACH_ORIENTATION)과 조준
    시점 orientation(spray_orientation_quat())이 다를 필요는 없다 — 파지 직후 link_6 기준 오프셋을
    실측(relative_pose)해서 쓰므로, 파지할 때 팔이 어떤 자세였는지는 조준 계산에 영향 없다.
    ★GUI 확인 후 수정★ : 처음엔 조준 orientation과 맞춰 수평으로 매달았었는데(로컬 Z가 world +X)
    시각적으로 부자연스러워 검증된 수직 자세로 되돌림."""
    stage = omni.usd.get_context().get_stage()
    UsdGeom.Xform.Define(stage, NOZZLE_DOCK_SCOPE)
    tool_prim = stage.DefinePrim(NOZZLE_TOOL_PATH, "Xform")
    tool_prim.GetReferences().AddReference(
        Sdf.Reference(assetPath=NOZZLE_USD, primPath=NOZZLE_SOURCE_PRIMPATH)
    )
    for _ in range(20):
        simulation_app.update()
    if not stage.GetPrimAtPath(NOZZLE_TOOL_PATH).IsValid():
        print(f"[FATAL] {NOZZLE_TOOL_PATH} 로드 실패"); return False

    dock_quat = TC_APPROACH_ORIENTATION  # (w,x,y,z) — 파지 접근 orientation과 동일 정렬(수직 매달기)
    xf = UsdGeom.Xformable(tool_prim)
    xf.ClearXformOpOrder()
    rot = Gf.Rotation(Gf.Quatd(float(dock_quat[0]), Gf.Vec3d(*dock_quat[1:])))
    m = Gf.Matrix4d().SetRotate(rot).SetTranslateOnly(
        Gf.Vec3d(float(NOZZLE_DOCK_XY[0]), float(NOZZLE_DOCK_XY[1]), float(NOZZLE_DOCK_HEIGHT))
    )
    xf.AddTransformOp().Set(m)
    print(f"[SPAWN] 노즐 거치대 = ({NOZZLE_DOCK_XY[0]:.3f},{NOZZLE_DOCK_XY[1]:.3f},{NOZZLE_DOCK_HEIGHT:.3f}), "
          f"조준orientation 정렬(매달린 자세)")

    # 임시 거치 조인트 : 정적 anchor prim을 명시적으로 body0 지정(2단계에서 body0 미지정 시 물리
    # 폭발하는 버그를 확인해 anchor 방식으로 고정함).
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
    """C2_SURFACE_GRIPPER(carter2 기존 그리퍼)는 move_tash_can.usd 에 이미 저작돼 있지만(오프라인
    pxr 조사로 확인), clearanceOffset=0.008 / grip_travel(transZ limit high)=0.01 로 — 2단계에서
    "파지 gap 20mm의 원인"으로 확인했던 바로 그 헐거운 기본값이다. 새로 만들지 않고(이미 있는
    D6 조인트를 지우고 다시 만들면 attachmentPoints 관계가 걸린 다른 참조가 깨질 수 있어 위험) 기존
    조인트 속성만 2단계에서 확정한 값(CLEARANCE_OFFSET/GRIP_TRAVEL)으로 직접 재기록한다."""
    joint_path = f"{C2_ARM_ROOT}/{C2_EE_LINK_NAME}/mop_surface_gripper_joints/mop_attachment_joint"
    gripper_prim = stage.GetPrimAtPath(C2_SURFACE_GRIPPER)
    joint_prim = stage.GetPrimAtPath(joint_path)
    if gripper_prim.IsValid() and joint_prim.IsValid():
        gripper_prim.GetAttribute("isaac:maxGripDistance").Set(float(MAX_GRIP_DISTANCE))
        joint_prim.GetAttribute("isaac:clearanceOffset").Set(float(CLEARANCE_OFFSET))
        joint_prim.GetAttribute("limit:transZ:physics:high").Set(float(GRIP_TRAVEL))
        joint_prim.GetAttribute("drive:transZ:physics:stiffness").Set(float(GRIP_DRIVE_STIFFNESS))
        joint_prim.GetAttribute("drive:transZ:physics:damping").Set(float(GRIP_DRIVE_DAMPING))
        print(f"[GRIPPER] {C2_SURFACE_GRIPPER} 기존 조인트를 2단계 검증값으로 재튜닝 "
              f"(clearance={CLEARANCE_OFFSET}, travel={GRIP_TRAVEL}, maxGripDistance={MAX_GRIP_DISTANCE})")
        return C2_SURFACE_GRIPPER

    print(f"[GRIPPER][WARN] {C2_SURFACE_GRIPPER} 또는 {joint_path} 없음 — 새로 authoring")
    fingertip_path = f"{C2_ARM_ROOT}/{C2_EE_LINK_NAME}"
    return surface_gripper_utils.setup_mop_surface_gripper(
        stage, fingertip_prim_path=fingertip_path,
        gripper_prim_path=C2_SURFACE_GRIPPER,
        max_grip_distance=MAX_GRIP_DISTANCE,
        grip_drive_stiffness=GRIP_DRIVE_STIFFNESS,
        grip_drive_damping=GRIP_DRIVE_DAMPING,
        clearance_offset=CLEARANCE_OFFSET,
        grip_travel=GRIP_TRAVEL,
    )


# ════════════════════════════════════════════════════════════════════════════
#  F. carter2 제너레이터 컨텍스트 + 헬퍼 (13_multi_robot_integrated.py 와 완전히 동일, 변경 없음)
# ════════════════════════════════════════════════════════════════════════════
class C2Ctx:
    """carter2 제너레이터가 공유하는 핸들 묶음. [3단계 추가] tool_changer/holding_nozzle/
    nozzle_tip_offset 은 툴체인지 상태 추적용(2/5 단계에서 사용)."""
    def __init__(self, world, robot, rmpflow, dof_names, tool0_path, ee_path, gripper,
                 ros_node, goal_pub, cmd_pub, pick_state):
        self.world = world; self.robot = robot; self.rmpflow = rmpflow
        self.dof_names = dof_names; self.tool0_path = tool0_path; self.ee_path = ee_path
        self.gripper = gripper
        self.ros_node = ros_node; self.goal_pub = goal_pub; self.cmd_pub = cmd_pub
        self.pick_state = pick_state
        self.stage = omni.usd.get_context().get_stage()
        self.status = "시작 대기"
        # [3단계] 툴체인지 상태
        self.tool_changer = None          # ToolChangerController, main() 에서 주입
        self.holding_nozzle = False
        self.nozzle_tip_offset = None      # link_6 기준 nozzle_tcp 상대위치(3축) — 파지 직후 실측
        # [4단계] 작업 선택 FSM 상태 — 기본값(디버그 경로는 안 씀), run_full_mission() 이 덮어씀.
        self.task_select_state = {"task": None}


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
                   growing_tolerance_max=None, growing_tolerance_tau=None, position_tolerance=None,
                   max_approach_steps=None):
    """13_multi_robot_integrated.py 와 동일(트래시 제너레이터가 재사용). position_tolerance/
    max_approach_steps 는 [3단계]에서 추가한 선택 인자(기본 None 이면 기존과 동일하게 모듈 상수
    사용 — 호출부가 인자를 안 넘기면 동작 무변경). 툴체인지(파지/반납)는 결국 이 함수(RMPflow 반응형
    IK) 대신 _tc_solve_and_ramp(LulaKinematicsSolver 1회 IK + 관절램프)로 교체했다 — 거치대가
    고정 위치라 반응형 추적이 불필요했고, 오래 돌수록 오히려 진동/발산하는 문제를 실측으로 확인함."""
    start = get_prim_world_position(ctx.tool0_path)
    distance = float(np.linalg.norm(target_position - start))
    ramp_steps = _ramp_steps_for_distance(distance, max_linear_speed)
    print(f"[INFO] c2 {label} 이동 (dist={distance:.3f}m)")
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
            print(f"[INFO] c2 {label} 도달 (step={step}, tol={cur_tol:.3f}m)"); return
    final_dist = float(np.linalg.norm(get_prim_world_position(ctx.tool0_path) - target_position))
    print(f"[WARN] c2 {label} {steps_budget} step 내 미수렴 (잔여오차={final_dist*1000:.1f}mm)")


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


def g_run_nav_leg(ctx, standoff_xy, standoff_yaw, chassis_goal_xy, chassis_goal_yaw, label):
    """Nav2 목표(standoff) 발행 → /start_pick 대기 → 실제 위치 기준 회전→직진→회전."""
    for _ in range(30):
        yield
    ctx.pick_state["start"] = False

    goal = PoseStamped(); goal.header.frame_id = "map"
    goal.pose.position.x = float(standoff_xy[0]); goal.pose.position.y = float(standoff_xy[1])
    goal.pose.orientation.z = float(np.sin(standoff_yaw / 2.0))
    goal.pose.orientation.w = float(np.cos(standoff_yaw / 2.0))

    print(f"[NAV:{label}] c2 standoff={standoff_xy.tolist()} yaw={np.degrees(standoff_yaw):.1f} 발행")
    ctx.status = f"{label}: Nav2 이동 대기({C2_NAV_GOAL} 발행중, {C2_START_PICK} 기다림)"
    while not ctx.pick_state["start"]:
        yield
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


def g_caster_nudge(ctx, chassis_path=C2_ARTICULATION_ROOT, distance=DOCK_NUDGE_DISTANCE):
    """[사용자 제안 — Nudge] 직전 회전/주행으로 캐스터 바퀴가 아무 방향으로나 트레일링된 채 정밀
    도킹을 시작하면 첫 스텝에 대각선으로 튀는(caster drift) 원인이 된다. 짧게 전진→후진(같은
    거리)해 캐스터를 현재 주행축 방향으로 강제 정렬한 뒤 정밀 접근을 시작한다."""
    yield from g_drive_straight_open_loop(ctx, distance, chassis_path, speed=DOCK_NUDGE_SPEED)
    yield from g_drive_straight_open_loop(ctx, distance, chassis_path, speed=DOCK_NUDGE_SPEED, reverse=True)


def g_precise_dock_approach(ctx, goal_xy, goal_yaw, chassis_path=C2_ARTICULATION_ROOT, label="DOCK"):
    """[사용자 제안 — Nudge + One-Shot] g_run_nav_leg 의 최종접근(회전→직진 오픈루프→회전)은
    직진 거리를 진입 시점에 1회만 재고 이후 다시 안 재서 잔여 오차가 남을 수 있다.
    ★첫 버전(폐기)★: 매 스텝 계속 재측정하는 연속 폐루프로 짜봤더니, 목표 수 cm 근방에서 "목표
    방향(bearing)"이 (ground truth라 센서 노이즈는 없어도) 위치가 0에 가까워질수록 atan2 자체가
    특이점에 가까워져 극도로 민감해지고, 위치보정↔자세보정 모드가 매 스텝 뒤바뀌는 chattering이
    실측(라이브 SPRAY_RETURN)으로 확인됨(위치는 1.5cm로 수렴했는데 yaw는 71.4도 방치, 900스텝 소진).
    ★현재 버전★: 이동 중엔 절대 목표 벡터를 재측정하지 않는 "정지→1회 측정→계산된 만큼만 이동"을
    최대 DOCK_ONESHOT_MAX_ITERS 회 반복(저빈도 discrete 폐루프라 chattering 자체가 구조적으로 불가능)
    하고, 최종 자세는 위치와 무관하게 그 자리에서 회전만 하는 g_rotate_in_place(각도만 보므로 안정적)
    로 정렬한다. 시작 전엔 g_caster_nudge 로 캐스터부터 정렬(계획서 Phase 2)."""
    yield from g_caster_nudge(ctx, chassis_path)

    for it in range(DOCK_ONESHOT_MAX_ITERS):
        ctx.cmd_pub.publish(Twist())
        for _ in range(DOCK_ONESHOT_SETTLE_STEPS):     # 완전 정지 후 깨끗한 1회 측정 확보
            yield
        pos_xy = get_prim_world_position(chassis_path)[:2]
        yaw = get_chassis_yaw(chassis_path)
        to_goal = goal_xy - pos_xy
        dist = float(np.linalg.norm(to_goal))
        if dist < DOCK_ONESHOT_POS_TOL:
            break
        bearing = float(np.arctan2(to_goal[1], to_goal[0]))
        print(f"[DOCK:{label}] One-Shot 스캔 #{it + 1}: dist={dist:.3f}m bearing={np.degrees(bearing):.1f}도"
              f" (이동 중 재측정 없음 — 이 값으로만 이동)")
        yield from g_rotate_in_place(ctx, bearing, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                     FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, chassis_path)
        yield from g_drive_straight_open_loop(ctx, dist, chassis_path, speed=DOCK_ONESHOT_DRIVE_SPEED)
    else:
        print(f"[DOCK:{label}][WARN] One-Shot {DOCK_ONESHOT_MAX_ITERS}회 반복 후에도 "
              f"위치 허용치({DOCK_ONESHOT_POS_TOL}m) 미달 — 마지막 측정값으로 자세 정렬만 진행")

    yield from g_rotate_in_place(ctx, goal_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, chassis_path)

    final_pos = get_prim_world_position(chassis_path)[:2]
    final_yaw = get_chassis_yaw(chassis_path)
    print(f"[DOCK:{label}] 정밀 도킹(Nudge+One-Shot) 완료 pos={np.round(final_pos, 3).tolist()}"
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
    print(f"[INFO] c2 big_trash 덤프 완료 (j1={DUMP_J1_DEG}, j6+={DUMP_J6_ROTATE_DEG})")


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
    print(f"[INFO] c2 쓰레기통 다시 위로 복귀 (j6-={DUMP_J6_ROTATE_DEG})")


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
#  G. [2/5 단계] 툴체인지 제너레이터 — 2_tool_changer_nozzle_demo.py 를 C2Ctx 제너레이터
#     컨벤션으로 이식. ToolChangerController(tool_changer.py) 재사용.
# ════════════════════════════════════════════════════════════════════════════
def _tc_solve_and_ramp(ctx, ik, target_pos, target_ori, warm_start, ramp_steps, label,
                       bias_compensation=None):
    """[RMPflow 대체] 거치대는 고정 위치라 매 스텝 반응형 IK(RMPflow)를 돌 이유가 없다 — 오히려
    실측으로 오래 돌수록 진동/발산하는 문제만 확인됨(400→900 step 로 늘렸더니 잔차가 2.3cm→최대
    17cm 로 악화, 파지 실패). 트래시 파지(TARGET_JOINTS_DEG)·분사 스윕(q_of_s)과 동일하게
    "IK 한 번 풀어서 관절각 확정 → 그 관절각으로 부드럽게 램프" 방식으로 통일한다.
    반환값 = (풀린 관절각 rad 6dof 또는 None, ok).
    ★position_tolerance=0.001★ : grip_travel 을 넓혀도 X축 잔차(5.5mm)가 전혀 안 줄어든 걸 실측
    확인 — D6 조인트의 forward_axis(Z)만 컴플라이언트(당겨서 맞춤)이고 옆방향(X/Y)은 완전히 잠긴
    축이라(surface_gripper_utils.create_attachment_point_joint: forward_axis 외 모든 trans 축을
    low>high 로 잠금) 그리퍼 쪽에서 옆방향 오차를 보정할 방법이 아예 없다. 즉 잔차의 진짜 원인은
    그리퍼가 아니라 IK 자체가 5mm 허용오차에서 "충분히 가깝다"고 멈춘 것 — 여기를 1mm 로 조인다.
    ★TC_TARGET_BIAS_COMPENSATION★ : 그래도 남는 잔차는 URDF-USD 형상 불일치(진단 확정, Lula 자체
    FK 는 target 과 완벽히 일치하나 실제 물리 시뮬레이션은 항상 같은 방향으로 벗어남)라 IK 파라미터로
    못 줄인다 — 파지 접근에서 실측된 고정 바이어스만큼 목표를 미리 보정해서 요청한다(반납 쪽은 IK 가
    다른 관절해로 수렴해 이 상수가 안 맞고, 정적 상수로 따로 보정하려 해도 더 악화됨을 확인 —
    반납은 g_tool_change_release 가 파지 직후 실측한 동적 오프셋으로 별도 보정한다)."""
    bias = bias_compensation if bias_compensation is not None else TC_TARGET_BIAS_COMPENSATION
    corrected_target = np.asarray(target_pos) - bias
    q, ok = ik.compute_inverse_kinematics(
        EE_FRAME, corrected_target, target_ori,
        warm_start=warm_start, position_tolerance=0.001, orientation_tolerance=0.02)
    if not ok:
        print(f"[TOOLCHANGE][WARN] {label} IK 실패")
        return None, False
    q6 = np.asarray(q[:6])

    # [진단] Lula 자체 FK(URDF 모델 기준)가 예측하는 link_6 위치 vs 우리가 요청한 target_pos —
    # 이게 이미 어긋나 있으면 "URDF FK 모델 자체가 target 근처 어디로 수렴했나"의 문제이고,
    # 맞다면(=Lula 내부는 일관됨) 문제는 URDF 기하학과 실제 USD 시뮬레이션 모델의 불일치.
    try:
        fk_pos, fk_rot = ik.compute_forward_kinematics(EE_FRAME, q)
        fk_err = np.asarray(fk_pos) - np.asarray(target_pos)
        print(f"[DIAG] {label} Lula FK(URDF) 예측 link_6={np.round(np.asarray(fk_pos), 4)} "
              f"target={np.round(np.asarray(target_pos), 4)} 오차={np.round(fk_err, 4)} (|e|={np.linalg.norm(fk_err)*1000:.2f}mm)")
    except Exception as e:
        print(f"[DIAG] compute_forward_kinematics 실패/미지원: {e}")

    yield from g_ramp_to_joint_positions(ctx, np.degrees(q6), ramp_steps)

    actual_pos = get_prim_world_position(ctx.ee_path)
    actual_err = actual_pos - np.asarray(target_pos)
    print(f"[DIAG] {label} 실제 시뮬레이션 link_6={np.round(actual_pos, 4)} target={np.round(np.asarray(target_pos), 4)} "
          f"오차={np.round(actual_err, 4)} (|e|={np.linalg.norm(actual_err)*1000:.2f}mm)")
    print(f"[TOOLCHANGE] {label} 관절이동 완료 q={np.round(q6, 3)}")
    return q6, True


def g_tool_change_grasp(ctx):
    """거치대에서 노즐 파지. 성공 시 ctx.holding_nozzle=True, ctx.nozzle_tip_offset 갱신.
    반환값(제너레이터 StopIteration.value) = grasp_ok(bool) — 호출부에서
    `ok = yield from g_tool_change_grasp(ctx)` 로 받는다."""
    tc = ctx.tool_changer
    handle_position, handle_orientation = tc.approach_tool_stand()
    base_pos, base_quat = read_world_pose(f"{C2_ARM_ROOT}/base_link")
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

    grasp_ok = tc.surface_gripper.is_closed()
    if not grasp_ok:
        print("[TOOLCHANGE][FAIL] 노즐 파지 실패")
        yield from g_ramp_to_joint_positions(ctx, np.degrees(q_above), TC_JOINT_RAMP_STEPS)
        return False

    release_hold_joint(ctx.stage)
    for _ in range(10):
        yield

    rel_pos, _ = relative_pose(ctx.ee_path, NOZZLE_TCP_PATH)
    ctx.nozzle_tip_offset = rel_pos
    # [실측 비교 결과 — 동적 보정으로 되돌림] 시도한 3가지 중 이게 제일 좋았다: 고정좌표만 4.6mm,
    # 반납전용 정적 보정(추정치) 8.3mm(오히려 악화 — 반납 IK가 목표를 살짝만 바꿔도 다른 관절해로
    # 수렴해 편차가 안 고정됨), 이 동적 보정(파지 직후 실측한 world-frame 오프셋을 반납 목표에 반영)
    # 이 2.56mm 로 최선. fingertip_offset_from_ik_frame 는 "world-frame 오차 벡터"로 정의돼 있으므로
    # (tool_changer.py 주석) 로컬 프레임 회전 없이 순수 위치 차이를 그대로 넣는다(예전 버그 수정판).
    base_offset = get_prim_world_position(NOZZLE_TOOL_PATH) - get_prim_world_position(ctx.ee_path)
    tc.fingertip_offset_from_ik_frame = base_offset
    ctx.holding_nozzle = True
    print(f"[TOOLCHANGE] 노즐 파지 성공. link_6 기준 tcp 오프셋={np.round(rel_pos, 4)} "
          f"(|rel|={np.linalg.norm(rel_pos):.4f}), 장착면 오프셋={np.round(base_offset, 4)}")

    yield from g_ramp_to_joint_positions(ctx, np.degrees(q_above), TC_JOINT_RAMP_STEPS)
    return True


def g_tool_change_release(ctx):
    """노즐을 거치대에 반납. 성공 시 ctx.holding_nozzle=False, hold_joint 재체결(다음 파지 대비),
    fingertip_offset_from_ik_frame 리셋(다음 "빈 손 접근" 정확도 유지).
    ★실측 비교(3가지 시도)★ : 고정좌표만 4.6mm / 반납전용 정적 보정(추정치) 8.3mm(악화) /
    파지 직후 실측한 동적 보정(이 함수가 쓰는 방식) 2.56mm — 동적 보정이 제일 좋아 이걸 채택.
    반환값 = release_ok(bool)."""
    tc = ctx.tool_changer

    # 스윕 직후엔 팔이 조준용 관절각(q_of_s, spray_orientation_quat 기반)에 가 있다 — 알려진 안전
    # 관절자세(STOW_Q)로 먼저 돌아온 뒤 시작(carter2_mission DUMP 후 "안전 관절복귀"와 동일 원리).
    yield from g_ramp_to_joint_positions(ctx, np.degrees(STOW_Q), SPRAY_ENTRY_RAMP_STEPS)

    stand_position, stand_orientation = tc.stand_return_target()
    base_pos, base_quat = read_world_pose(f"{C2_ARM_ROOT}/base_link")
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
    # 재도킹까지 자유낙하 시간을 짧게(2단계에서 확인: 길면 hold_joint 재체결 시 큰 오차를
    # 강체 조인트로 한번에 보정하려다 물리 불안정 위험).
    for _ in range(15):
        yield
    release_ok = not tc.surface_gripper.is_closed()

    engage_hold_joint(ctx.stage)
    for _ in range(TC_REDOCK_SETTLE_STEPS):
        yield

    ctx.holding_nozzle = False
    ctx.nozzle_tip_offset = None
    tc.fingertip_offset_from_ik_frame = TC_FINGERTIP_OFFSET_FROM_TOOL0.copy()  # 다음 "빈 손 접근" 대비 리셋
    print(f"[TOOLCHANGE] 노즐 반납 {'성공' if release_ok else '실패'} + 재도킹 완료")

    yield from g_ramp_to_joint_positions(ctx, np.degrees(q_above), TC_JOINT_RAMP_STEPS)
    return release_ok


# ════════════════════════════════════════════════════════════════════════════
#  H. [3/5 단계] 분사 스윕(WIPE/MOVE) 제너레이터 — Carter1Spray(13_multi_robot_integrated.py)
#     의 tick() 상태머신을 C2Ctx 제너레이터 컨벤션으로 이식. Sweeper/SprayFX 는 로봇 인스턴스에
#     종속되지 않는 순수 로직이라 변경 없이 그대로 옮긴다. 조준 IK 오프셋만 carter1의 고정상수
#     대신 파지 직후 실측치(ctx.nozzle_tip_offset)를 쓴다(설계문서 3장, 2단계에서 이미 검증됨).
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
    """13_multi_robot_integrated.py 와 완전히 동일(변경 없음) — 로봇 인스턴스에 종속되지 않는
    순수 시각효과(경량 탄도 파티클 풀)."""
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
        """[GUI 확인 후 추가] 이 인스턴스는 g_spray_sweep 제너레이터 로컬 변수라, 제너레이터가
        끝나면 더 이상 update() 가 호출되지 않아 마지막 프레임 파티클이 화면에 얼어붙어 남는다
        (실측으로 확인됨). 제너레이터가 어떤 이유로 끝나든(완주/조기종료/grip풀림) 반드시 호출해
        위젯을 0으로 밀어 즉시 지운다."""
        self.age[:] = SPRAY_LIFETIME + 1.0
        self._w_attr.Set(Vt.FloatArray.FromNumpy(np.zeros(self.N, dtype=np.float32)))


class Sweeper:
    """13_multi_robot_integrated.py 와 완전히 동일(변경 없음)."""
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
    """WIPE(정지+상하 스윕)↔MOVE(팔 고정+전진, heading-hold) 반복 — carter1 tick() 의 제너레이터화.
    사전조건 : ctx.holding_nozzle=True, ctx.nozzle_tip_offset 실측 완료(g_tool_change_grasp 이후).
    max_steps 를 주면 forward_distance 도달과 무관하게 그 스텝 수만큼만 돌고 반환(디버그용 —
    실제 10.5m 완주는 시간이 오래 걸려 3/5 헤드리스 검증에선 짧게 끊어 확인한다).
    반환값 = True(목표 거리 도달) / False(max_steps 조기종료 · 조준IK실패 · grip풀림 등 실패).
    ★라이브 실측(2026-07-24)★ : 조준 IK 실패 시 예전엔 RuntimeError 를 던져 최상위 미션 루프
    전체가 죽는 버그가 있었음(spray 웨이포인트에서 q_high 못 풀림, low=True/high=False로 재현) —
    이젠 False 를 반환해 호출부(g_spray_mission_body)가 그대로 거치대 복귀로 넘어가게 한다."""
    if not ctx.holding_nozzle or ctx.nozzle_tip_offset is None:
        print("[SPRAY][FAIL] 노즐을 파지한 상태에서만 호출 가능(ctx.holding_nozzle=False) — 중단")
        return False

    stage = ctx.stage
    base_pos, base_quat = read_world_pose(f"{C2_ARM_ROOT}/base_link")
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
        print(f"[SPRAY][FAIL] 조준 IK 실패 (low={ok_lo}, high={ok_hi}) — 이 웨이포인트에서 스윕 불가, "
              f"거치대 복귀로 넘어감(WALL_X/Z_HIGH 나 챠시 도착 yaw 재확인 필요)")
        return False
    q_low = np.asarray(q_low[:6]); q_high = np.asarray(q_high[:6])
    q_mid = 0.5 * (q_low + q_high); q_half = 0.5 * (q_high - q_low)

    def q_of_s(s):
        q = q_mid + q_half * s
        q[J5_INDEX] += J5_FLICK * s
        q[J1_INDEX] += J1_OFFSET
        return q

    arm_idx = np.array([ctx.dof_names.index(n) for n in ARM_JOINT_NAMES])

    # [GUI 확인 후 추가] apply() 는 q_applied 를 매 스텝 MAX_JOINT_STEP 만큼만 움직이지만, q_applied
    # 의 "시작값"을 곧바로 q_of_s(-1.0)(스윕 저점)으로 잡아버려서 팔의 실제 현재 관절각(방금
    # g_tool_change_grasp 이 끝낸 자세, 스윕 자세와 무관)과 첫 커맨드 사이에 램프가 전혀 없었다 —
    # 그 결과 관절 드라이브(강성 1e8)가 실제 각도에서 스윕 저점으로 사실상 순간 스냅함(체감상
    # "너무 빠르게 자세를 잡는" 원인). g_ramp_to_joint_positions 로 현재 실제 자세→스윕 저점을
    # 부드럽게 이어준 뒤에 스윕을 시작한다.
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

        chassis_pos, chassis_quat = read_world_pose(C2_ARTICULATION_ROOT)
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

        # 분사 FX : WIPE 구간에서 실측 노즐 팁(nozzle_tcp) 실시간 world pose 로 방출.
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


def drive_generator_to_completion(gen, my_world):
    """13_ main 루프의 `my_world.step(); next(gen)` 관례를 그대로 따르는 디버그용 드라이버.
    StopIteration.value(제너레이터 return 값)를 그대로 돌려준다."""
    while True:
        my_world.step(render=True)
        try:
            next(gen)
        except StopIteration as e:
            return e.value


# TC_DEBUG_ONLY=1 이면 챠시를 거치대 근처로 순간이동시키고 툴체인지(파지→반납)만 실행 후 종료
# (Nav2/미션 FSM 없이 2/5 단계 결과만 빠르게 확인하기 위한 임시 경로. 4/5 단계에서 정식 FSM 추가되면
# 이 분기는 남겨두되 실제 사용은 정식 FSM 이 대체).
TC_DEBUG_ONLY = os.environ.get("TC_DEBUG_ONLY", "0") == "1"
TC_DEBUG_CHASSIS_OFFSET_X = 0.35  # 거치대에서 이만큼 떨어진 곳에 챠시를 세움([GUI 확인 후] 0.5→0.35, 더 가깝게)

# SPRAY_DEBUG_ONLY=1 이면(TC_DEBUG_ONLY 를 내포) 파지 후 g_spray_sweep 을 SPRAY_DEBUG_MAX_STEPS
# 만큼만 돌려본다(전체 10.5m 완주는 시간이 오래 걸려 헤드리스 확인엔 부적합 — WIPE↔MOVE 몇 사이클
# 도는 것만 확인하면 3/5 단계 목적엔 충분: "grip 유지·팔 궤적 확인").
SPRAY_DEBUG_ONLY = os.environ.get("SPRAY_DEBUG_ONLY", "0") == "1"
SPRAY_DEBUG_MAX_STEPS = int(os.environ.get("SPRAY_DEBUG_MAX_STEPS", "600"))  # 10초 @ 60Hz


# ════════════════════════════════════════════════════════════════════════════
#  I. [4/5 단계] 트래시 서브미션 — carter2_mission() (13_multi_robot_integrated.py) 그대로 이식.
#     ★유일한 변경점★ : 원본 끝의 "미션 완료 → while simulation_app.is_running(): yield"
#     (그 자리에 영원히 유휴) 를 그냥 return 으로 바꿨다 — 작업 선택식 최상위 루프
#     (g_single_robot_mission)가 "유휴 대기"를 대신 담당해야 다음 task_select 를 받을 수 있기
#     때문(원본은 애초에 단일 실행 후 끝이라 이 문제가 없었음). PICK/DUMP/RETURN/DOCK 로직 자체는
#     단 한 줄도 안 건드림.
# ════════════════════════════════════════════════════════════════════════════
def carter2_mission(ctx):
    """carter2 전체 트래시 미션 제너레이터 (4_ main 시퀀스를 yield 화).
    PICK → DUMP → RETURN(원위치 복귀·내려놓기) → DOCK(도킹 복귀) 4단계."""
    trash_xy = np.array(TRASH_SPAWN_FIXED)
    trash_origin_xy = trash_xy - TRASH_BBOX_CENTER_OFFSET_XY
    spawn_xy = np.array([C2_START_POSE["x"], C2_START_POSE["y"]])
    chassis_goal_xy, chassis_goal_yaw = _pick_closest_entry(trash_origin_xy, spawn_xy)
    approach_dir = rotate_2d(OFFSET_TRASH_FROM_CHASSIS / np.linalg.norm(OFFSET_TRASH_FROM_CHASSIS), chassis_goal_yaw)
    standoff_xy = chassis_goal_xy - approach_dir * FINAL_APPROACH_DISTANCE
    standoff_yaw = float(np.arctan2(approach_dir[1], approach_dir[0]))

    for _ in range(SETTLE_STEPS):
        yield

    sync_rmpflow_base_pose(ctx)
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

    cur = ctx.robot.get_joint_positions()
    tuck_deg = []
    for name in ARM_JOINT_NAMES:
        idx = ctx.dof_names.index(name)
        tuck_deg.append(TUCK_J1_DEG if name == "joint_1" else float(np.degrees(cur[idx])))
    yield from g_ramp_to_joint_positions(ctx, tuck_deg, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    print(f"[INFO] c2 j1 tuck→{TUCK_J1_DEG} 완료")

    face_dir = -BIG_TRASH_APPROACH_DIR
    big_yaw = float(np.arctan2(face_dir[1], face_dir[0]))
    big_standoff = BIG_TRASH_POSITION_XY + BIG_TRASH_APPROACH_DIR * BIG_TRASH_STANDOFF_DISTANCE
    big_goal = BIG_TRASH_POSITION_XY + BIG_TRASH_APPROACH_DIR * BIG_TRASH_FINAL_DISTANCE
    yield from g_run_nav_leg(ctx, big_standoff, big_yaw, big_goal, big_yaw, "DUMP")
    if not simulation_app.is_running():
        return
    yield from g_dump_into_big_trash(ctx)

    yield from g_restore_upright_after_dump(ctx)
    yield from g_drive_straight_open_loop(ctx, POST_DUMP_BACKUP_DISTANCE, C2_ARTICULATION_ROOT,
                                          FINAL_APPROACH_SPEED, reverse=True)
    post_dump_yaw = wrap_pi(get_chassis_yaw(C2_ARTICULATION_ROOT) + np.pi)
    yield from g_rotate_in_place(ctx, post_dump_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, C2_ARTICULATION_ROOT)
    print(f"[APPROACH:DUMP] c2 {POST_DUMP_BACKUP_DISTANCE:.2f}m 후진 + 180 회전 → RETURN 시작")

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
    print(f"[INFO] c2 Surface Gripper 개방 (is_closed={ctx.gripper.is_closed()})")
    retract_target = grasp_position + LIFT_OFFSET
    yield from g_move_to_pose(ctx, retract_target, grasp_orientation, "내려놓은 후 후퇴")
    yield from g_hold_pose(ctx, retract_target, grasp_orientation, GRASP_HOLD_STEPS)
    yield from g_ramp_to_joint_positions(ctx, [0.0] * 6, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        yield
    print("[INFO] c2 팔 홈(0deg) 복귀 완료")

    yield from g_drive_straight_open_loop(ctx, POST_RETURN_BACKUP_DISTANCE, C2_ARTICULATION_ROOT,
                                          FINAL_APPROACH_SPEED, reverse=True)
    post_ret_yaw = wrap_pi(get_chassis_yaw(C2_ARTICULATION_ROOT) + np.pi)
    yield from g_rotate_in_place(ctx, post_ret_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
                                 FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, C2_ARTICULATION_ROOT)
    print(f"[APPROACH:RETURN] c2 {POST_RETURN_BACKUP_DISTANCE:.2f}m 후진 + 180 회전 → DOCK 시작")

    ctx.status = "DOCK: 도킹 복귀"
    # [사용자 요청 — 복귀 로직 통일] 원래 여기는 스폰 반대(+180도) 방향으로 주차했는데, 분사
    # 미션 복귀(g_nav_to_dock_approach, DOCK_APPROACH_YAW=스폰과 동일 방향)와 각도가 달라 두 작업
    # 완료 후 자세가 눈에 띄게 달라 보였다 — 두 작업 다 같은 DOCK_APPROACH_XY/YAW·같은 standoff
    # 접근 패턴·같은 정밀 서보(g_precise_dock_approach)로 통일한다.
    dock_dir = np.array([np.cos(DOCK_APPROACH_YAW), np.sin(DOCK_APPROACH_YAW)])
    dock_standoff_xy = DOCK_APPROACH_XY - dock_dir * FINAL_APPROACH_DISTANCE
    yield from g_run_nav_leg(ctx, dock_standoff_xy, DOCK_APPROACH_YAW, DOCK_APPROACH_XY, DOCK_APPROACH_YAW, "DOCK")
    if not simulation_app.is_running():
        return
    yield from g_precise_dock_approach(ctx, DOCK_APPROACH_XY, DOCK_APPROACH_YAW, C2_ARTICULATION_ROOT, "DOCK")
    if not simulation_app.is_running():
        return

    print("[INFO] c2 트래시 미션 완료(파지+덤프+원위치복귀+도킹).")
    ctx.status = "트래시 완료 — IDLE 복귀"
    # [3단계 변경점] 원본은 여기서 "while simulation_app.is_running(): yield"(영원히 유휴)였으나,
    # 작업 선택식 최상위 루프가 유휴를 담당하도록 그냥 return.


# ════════════════════════════════════════════════════════════════════════════
#  J. [4/5 단계] 분사 서브미션 + task_select 최상위 디스패처
# ════════════════════════════════════════════════════════════════════════════
# spray_waypoint_mission.py 가 실사용하던 검증된 좌표 재사용(project 메모리 기준) — WP1 만 우선 구현
# (양쪽 벽 왕복은 WP2 180도 반대 좌표 추가로 확장 가능, 이번 단계 범위 밖).
SPRAY_WP1_XY = np.array([18.8, 8.0])
SPRAY_WP1_YAW = np.radians(90.0)
# 거치대 접근 지점 — [재배치] 거치대(NOZZLE_DOCK_XY)를 다시 carter2 홈에서 떼어놨으므로(위 참고),
# 접근 지점도 더 이상 홈이 아니라 거치대 기준으로 별도 정의한다: 거치대에서 팔이 닿는 검증된 거리
# (TC_DEBUG_CHASSIS_OFFSET_X, 0.35m)만큼 떨어진 지점, 방향은 거치대가 홈 기준 +X 축 위에 있으므로
# 그대로 0도(= carter2 홈과 같은 방향, dock 을 바라봄).
DOCK_APPROACH_YAW = np.radians(C2_START_POSE["yaw_deg"])
DOCK_APPROACH_XY = NOZZLE_DOCK_XY - np.array([np.cos(DOCK_APPROACH_YAW), np.sin(DOCK_APPROACH_YAW)]) * TC_DEBUG_CHASSIS_OFFSET_X


DOCK_APPROACH_SKIP_XY_RADIUS = 0.15   # 이 안이면 "이미 거치대 근처" 로 보고 nav-leg 생략
DOCK_APPROACH_SKIP_YAW_TOL = np.radians(15.0)


def g_nav_to_dock_approach(ctx, label="DOCK_APPROACH"):
    """거치대 팔 작업(파지/반납) 전 챠시를 거치대 근처(DOCK_APPROACH_XY, 팔이 닿는 거리)로 필요할
    때만 nav-leg 이동시킨다. ★버그 발견 이력★(당시엔 DOCK_APPROACH_XY 가 carter2 홈과 같은 지점이라
    스폰 직후 이미 "도착"인 경우가 흔했음) : 이 함수가 "위치·자세 이미 근접"인지 확인 안 하고 매번
    무조건 Nav2 nav-leg(정지점 발행→최종 회전-직진-회전) 를 거쳐 스폰 직후에도 불필요하게 움직였다
    (사용자 실측 지적) → 이미 충분히 가까우면(위치+자세 둘 다) 바로 건너뛰고 팔 작업으로 넘어간다.
    [재배치 후] 지금은 거치대가 다시 홈에서 떨어져 있어(NOZZLE_DOCK_XY 정의부 참고) 스폰 직후엔
    거의 항상 이 스킵 조건을 안 타고 실제 nav-leg 를 타게 된다 — 의도된 동작(분사 작업은 늘 왕복이
    필요하므로)."""
    cur_xy = get_prim_world_position(C2_ARTICULATION_ROOT)[:2]
    cur_yaw = get_chassis_yaw(C2_ARTICULATION_ROOT)
    xy_close = float(np.linalg.norm(cur_xy - DOCK_APPROACH_XY)) < DOCK_APPROACH_SKIP_XY_RADIUS
    yaw_close = abs(wrap_pi(cur_yaw - DOCK_APPROACH_YAW)) < DOCK_APPROACH_SKIP_YAW_TOL
    if xy_close and yaw_close:
        print(f"[NAV:{label}] 이미 거치대 근처(위치·자세 일치, dist={np.linalg.norm(cur_xy - DOCK_APPROACH_XY):.3f}m) "
              f"— nav-leg 생략, 바로 팔 작업")
        sync_rmpflow_base_pose(ctx)
        return

    dock_dir = np.array([np.cos(DOCK_APPROACH_YAW), np.sin(DOCK_APPROACH_YAW)])
    dock_standoff_xy = DOCK_APPROACH_XY - dock_dir * FINAL_APPROACH_DISTANCE
    ctx.status = f"{label}: 거치대 근처로 이동"
    sync_rmpflow_base_pose(ctx)
    yield from g_run_nav_leg(ctx, dock_standoff_xy, DOCK_APPROACH_YAW, DOCK_APPROACH_XY, DOCK_APPROACH_YAW, label)
    if not simulation_app.is_running():
        return
    yield from g_precise_dock_approach(ctx, DOCK_APPROACH_XY, DOCK_APPROACH_YAW, C2_ARTICULATION_ROOT, label)
    if not simulation_app.is_running():
        return
    sync_rmpflow_base_pose(ctx)


def g_spray_mission_body(ctx):
    """분사 서브미션 : 노즐은 이미 파지된 상태로 시작(호출부 보장). 분사 웨이포인트 nav-leg →
    스윕(WIPE/MOVE, FORWARD_DISTANCE) → 거치대 근처로 nav-leg 복귀. 실제 반납(g_tool_change_release,
    팔 동작)은 호출부가 이어서 수행."""
    spray_dir = np.array([np.cos(SPRAY_WP1_YAW), np.sin(SPRAY_WP1_YAW)])
    spray_standoff_xy = SPRAY_WP1_XY - spray_dir * FINAL_APPROACH_DISTANCE
    sync_rmpflow_base_pose(ctx)
    ctx.status = "SPRAY: 웨이포인트로 이동"
    yield from g_run_nav_leg(ctx, spray_standoff_xy, SPRAY_WP1_YAW, SPRAY_WP1_XY, SPRAY_WP1_YAW, "SPRAY_GOTO")
    if not simulation_app.is_running():
        return

    ctx.status = "SPRAY: 스윕 진행 중"
    yield from g_spray_sweep(ctx, forward_distance=FORWARD_DISTANCE)
    if not simulation_app.is_running():
        return

    yield from g_nav_to_dock_approach(ctx, "SPRAY_RETURN")


def g_single_robot_mission(ctx):
    """[4/5 단계] 최상위 작업 선택 루프 — ctx.task_select_state["task"](/carter2/task_select 구독이
    채움, "trash"|"spray")를 기다렸다가 그에 맞는 서브미션 실행 후 다시 대기(IDLE)로 돌아간다.
    "trash" 선택 시 노즐을 들고 있으면 먼저 거치대에 반납, "spray" 선택 시 먼저 파지 후 시작.
    ★버그 발견·수정★ : IDLE 진입 시 곧바로 task_select_state["task"]=None 으로 리셋했었는데, 이게
    "확인하기 직전"이 아니라 제너레이터 재진입 즉시 실행돼서 — 미리 세팅해둔 값(SELFTEST_TASK 나,
    시뮬레이션 시작 극초반에 아주 일찍 도착하는 실제 ROS 메시지)이 검사되기도 전에 지워지는 경쟁
    상태가 있었다(헤드리스 SELFTEST_TASK 로 재현: IDLE 에서 영원히 안 빠져나옴). 리셋을 "값을 읽어
    소비한 직후"로 옮겨 해결."""
    while simulation_app.is_running():
        ctx.status = "IDLE: task_select 대기 (/carter2/task_select)"
        while ctx.task_select_state["task"] is None:
            yield
            if not simulation_app.is_running():
                return

        task = ctx.task_select_state["task"]
        ctx.task_select_state["task"] = None
        print(f"[MISSION] task_select 수신 = '{task}'")

        if task == "trash":
            if ctx.holding_nozzle:
                print("[MISSION] 노즐 보유 중 → 트래시 작업 전 거치대로 이동 후 반납")
                yield from g_nav_to_dock_approach(ctx, "TRASH_PRE_RETURN")
                yield from g_tool_change_release(ctx)
            yield from carter2_mission(ctx)
            print("[MISSION] 트래시 작업 완료 → IDLE 복귀")

        elif task == "spray":
            if not ctx.holding_nozzle:
                yield from g_nav_to_dock_approach(ctx, "SPRAY_PRE_GRASP")
                ok = yield from g_tool_change_grasp(ctx)
                if not ok:
                    print("[MISSION][FAIL] 노즐 파지 실패 — 분사 작업 취소, IDLE 복귀")
                    continue
            yield from g_spray_mission_body(ctx)
            yield from g_tool_change_release(ctx)
            print("[MISSION] 분사 작업 완료 → IDLE 복귀")

        else:
            print(f"[MISSION][WARN] 알 수 없는 task '{task}' — 무시")


def build_common_control(stage, c2_robot, c2_ee_path):
    """RMPflow(트래시 Cartesian 이동용) + ground_plane + Surface Gripper + ToolChangerController —
    디버그 경로와 정식 FSM 경로가 공통으로 필요로 하는 초기화를 한 곳으로 모음."""
    c2_rmpflow = RMPFlowController(name="carter2_cspace", robot_articulation=c2_robot, urdf_path=C2_NO_GRIPPER_URDF)
    sync_base_prim = stage.GetPrimAtPath(f"{C2_ARM_ROOT}/base_link")
    _m = omni.usd.get_world_transform_matrix(sync_base_prim)
    _t = _m.ExtractTranslation(); _q = _m.ExtractRotationQuat(); _im = _q.GetImaginary()
    c2_rmpflow.rmp_flow.set_robot_base_pose(
        robot_position=np.array([_t[0], _t[1], _t[2]]),
        robot_orientation=np.array([_q.GetReal(), _im[0], _im[1], _im[2]]))
    try:
        c2_rmpflow.rmp_flow.add_ground_plane(prim_path=C2_GROUND_PLANE, z_position=0.0)
        print(f"[RMP] ground plane 등록: {C2_GROUND_PLANE}")
    except TypeError:
        try:
            c2_rmpflow.rmp_flow.add_ground_plane()
            print("[RMP] ground plane 등록(무인자)")
        except Exception as e:
            print(f"[RMP][WARN] ground plane 등록 실패({e}) → 스킵")
    except Exception as e:
        print(f"[RMP][WARN] ground plane 등록 실패({e}) → 스킵")

    c2_gripper = SurfaceGripper(end_effector_prim_path=c2_ee_path, surface_gripper_path=C2_SURFACE_GRIPPER)
    c2_gripper.initialize()

    dock_quat = TC_APPROACH_ORIENTATION
    tool_changer = ToolChangerController(
        rg2_fingertip_prim_path=c2_ee_path,
        mop_handle_prim_path=NOZZLE_TOOL_PATH,
        stand_position=np.array([NOZZLE_DOCK_XY[0], NOZZLE_DOCK_XY[1], NOZZLE_DOCK_HEIGHT]),
        stand_orientation=dock_quat,
        approach_orientation=dock_quat,
        fingertip_offset_from_ik_frame=TC_FINGERTIP_OFFSET_FROM_TOOL0,
        rg2_gripper=None,
        surface_gripper_prim_path=C2_SURFACE_GRIPPER,
        auto_create_surface_gripper=False,
    )
    tool_changer.initialize()
    return c2_rmpflow, c2_gripper, tool_changer


# ════════════════════════════════════════════════════════════════════════════
#  Z. main — [1/5] 씬 구성 + [2/5,3/5] 디버그 경로 + [4/5,5/5] 정식 task_select FSM.
# ════════════════════════════════════════════════════════════════════════════
def run_full_mission(my_world, stage, c2_robot, c2_ee_path, c2_tool0_path, c2_dof):
    """[4/5,5/5 단계] 정식 task_select FSM 실행 — 13_multi_robot_integrated.py main() 의 carter2
    초기화/루프 구조와 동일한 패턴(전역 /clock 발행 + rclpy.spin_once + next(gen) 매 스텝).
    carter1 이 없으므로 c1.tick()/next(c2_gen) 인터리빙은 사라지고 next(mission_gen) 단일 호출만 남는다.
    ROS 사이드 신규 작성 불필요(계획 문서) : trash_can_nav_pick_mission.py 가 좌표 무관 제너릭 goal
    포워더라 트래시/분사 어느 leg 든 기존 /carter2/trash_can_nav_goal + /carter2/start_pick 를 그대로 씀.
    /carter2/task_select(String, "trash"|"spray") 만 신규 — 사용자가 ros2 topic pub 으로 트리거."""
    c2_rmpflow, c2_gripper, tool_changer = build_common_control(stage, c2_robot, c2_ee_path)

    rclpy.init()
    ros_node = rclpy.create_node("single_robot_tool_changer_controller")
    clock_pub = ros_node.create_publisher(Clock, "/clock", 10)
    goal_pub = ros_node.create_publisher(PoseStamped, C2_NAV_GOAL, 10)
    cmd_pub = ros_node.create_publisher(Twist, C2_CMD_VEL, 10)

    pick_state = {"start": False}
    ros_node.create_subscription(Bool, C2_START_PICK,
                                 lambda m: pick_state.__setitem__("start", bool(m.data)), 10)
    task_select_state = {"task": None}
    ros_node.create_subscription(String, C2_TASK_SELECT,
                                 lambda m: task_select_state.__setitem__("task", str(m.data).strip().lower()), 10)
    print(f"[ROS] pub {C2_NAV_GOAL} + {C2_CMD_VEL} + /clock, sub {C2_START_PICK} + {C2_TASK_SELECT}")
    print(f"[ROS] 트리거 예시 : ros2 topic pub /{NS_CARTER2}/task_select std_msgs/msg/String "
          "\"data: 'spray'\" --once  (또는 'trash')")

    ctx = C2Ctx(my_world, c2_robot, c2_rmpflow, c2_dof, c2_tool0_path, c2_ee_path, c2_gripper,
               ros_node, goal_pub, cmd_pub, pick_state)
    ctx.tool_changer = tool_changer
    ctx.task_select_state = task_select_state
    mission_gen = g_single_robot_mission(ctx)

    # SELFTEST_TASK=spray|trash 이면 외부 ros2 topic pub 없이 헤드리스로 곧바로 트리거(디버그/검증용).
    _selftest_task = os.environ.get("SELFTEST_TASK", "").strip().lower()
    if _selftest_task in ("spray", "trash"):
        task_select_state["task"] = _selftest_task
        print(f"[SELFTEST] task_select 자동 트리거 = '{_selftest_task}'")

    print("\n[RUN] Play ▶ : carter2 단일로봇 작업선택 미션 대기 중. "
          f"Nav2(carter2 namespace) + trash_can_nav_pick_mission.py(namespace:=carter2) 도 함께 실행하세요.\n")

    hb = 0
    step_i = 0
    mission_done = False
    try:
        while simulation_app.is_running():
            step_i += 1
            my_world.step(render=(step_i % RENDER_EVERY == 0))

            t = float(my_world.current_time)
            cmsg = Clock(); cmsg.clock.sec = int(t); cmsg.clock.nanosec = int(round((t - int(t)) * 1e9))
            clock_pub.publish(cmsg)
            rclpy.spin_once(ros_node, timeout_sec=0.0)

            if not my_world.is_playing():
                continue

            if not mission_done:
                try:
                    next(mission_gen)
                except StopIteration:
                    mission_done = True
                    print("[MISSION] 최상위 제너레이터 종료(비정상 — 통상 IDLE 유휴 루프라 안 끝남)")

            hb += 1
            if hb % 300 == 0:
                print(f"[HB] carter2 = {ctx.status} (노즐 보유={ctx.holding_nozzle})")
    except Exception:
        import traceback
        print("\n[FATAL] main 루프 예외 — 아래 파이썬 트레이스백이 진짜 원인입니다:\n")
        traceback.print_exc()
    finally:
        try:
            cmd_pub.publish(Twist())
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


def main():
    my_world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)

    build_env()
    if not build_carter2():
        simulation_app.close(); return
    if not build_nozzle_dock():
        simulation_app.close(); return

    stage = omni.usd.get_context().get_stage()
    boost_drive_limits(f"{C2_SCOPE}/Nova_Carter_ROS")

    if TC_DEBUG_ONLY or SPRAY_DEBUG_ONLY:
        reposition_carter2_near(
            stage, (NOZZLE_DOCK_XY[0] - TC_DEBUG_CHASSIS_OFFSET_X, NOZZLE_DOCK_XY[1]),
            yaw_deg=float(os.environ.get("TC_DEBUG_YAW_DEG", "0.0")))

    c2_robot, c2_ee_path, c2_tool0_path = build_carter2_control(my_world)
    gripper_path = setup_nozzle_surface_gripper(stage)

    my_world.reset()
    for _ in range(5):
        my_world.step(render=False)

    c2_robot.initialize()
    c2_dof = list(c2_robot.dof_names)
    dp = c2_robot.get_joint_positions()
    for name in ARM_JOINT_NAMES:
        if name in c2_dof:
            dp[c2_dof.index(name)] = 0.0
    c2_robot.set_joint_positions(dp)
    for _ in range(30):
        my_world.step(render=True)

    chassis_pos = get_prim_world_position(C2_ARTICULATION_ROOT)
    ee_pos = get_prim_world_position(c2_ee_path)
    nozzle_pos = get_prim_world_position(NOZZLE_TOOL_PATH)
    print("=" * 64)
    print(f"[CHECK] c2 chassis world = {np.round(chassis_pos, 3)}")
    print(f"[CHECK] c2 link_6 world  = {np.round(ee_pos, 3)}")
    print(f"[CHECK] nozzle dock world = {np.round(nozzle_pos, 3)} "
          f"(목표 {NOZZLE_DOCK_XY.tolist()},{NOZZLE_DOCK_HEIGHT} 대비 드리프트 "
          f"{np.linalg.norm(nozzle_pos - np.array([NOZZLE_DOCK_XY[0], NOZZLE_DOCK_XY[1], NOZZLE_DOCK_HEIGHT])):.4f}m)")
    print(f"[CHECK] chassis-nozzle 거리(XY) = {np.linalg.norm(chassis_pos[:2] - nozzle_pos[:2]):.2f}m")
    print(f"[CHECK] gripper prim path = {gripper_path}")
    print("=" * 64)

    if os.environ.get("ISAAC_HEADLESS", "0") != "1":
        setup_gui_camera_and_light(stage, C2_ARM_ROOT)

    if not (TC_DEBUG_ONLY or SPRAY_DEBUG_ONLY):
        run_full_mission(my_world, stage, c2_robot, c2_ee_path, c2_tool0_path, c2_dof)
        return

    # ── [2/5,3/5] 툴체인지/분사 디버그 경로 ─────────────────────────────────
    c2_rmpflow, c2_gripper, tool_changer = build_common_control(stage, c2_robot, c2_ee_path)

    # SPRAY_DEBUG_ONLY 는 g_spray_sweep 의 MOVE 구간에서 cmd_pub 이 필요 → rclpy 최소 설정.
    ros_node = None; cmd_pub = None
    if SPRAY_DEBUG_ONLY:
        rclpy.init()
        ros_node = rclpy.create_node("tool_changer_spray_debug")
        cmd_pub = ros_node.create_publisher(Twist, C2_CMD_VEL, 10)

    ctx = C2Ctx(my_world, c2_robot, c2_rmpflow, c2_dof, c2_tool0_path, c2_ee_path, c2_gripper,
               ros_node, None, cmd_pub, None)
    ctx.tool_changer = tool_changer

    print("[TOOLCHANGE][DEBUG] 파지 시퀀스 시작")
    grasp_ok = drive_generator_to_completion(g_tool_change_grasp(ctx), my_world)
    print(f"[TOOLCHANGE][DEBUG] 파지 결과 = {grasp_ok}")

    if grasp_ok and SPRAY_DEBUG_ONLY:
        print(f"[SPRAY][DEBUG] 스윕 시퀀스 시작 (max_steps={SPRAY_DEBUG_MAX_STEPS})")
        sweep_ok = drive_generator_to_completion(
            g_spray_sweep(ctx, forward_distance=FORWARD_DISTANCE, max_steps=SPRAY_DEBUG_MAX_STEPS), my_world)
        print(f"[SPRAY][DEBUG] 스윕 결과(True=완주, False=max_steps 조기종료) = {sweep_ok}, "
              f"grip 유지 = {ctx.holding_nozzle}")

    if grasp_ok and ctx.holding_nozzle:
        if SPRAY_DEBUG_ONLY:
            # [GUI 확인 후 추가·수정] 스윕 중 챠시가 MOVE 로 전진해 거치대에서 멀어진 채로 반납을
            # 시도하면, 팔이 거치대까지 못 닿은 채 놓아버리는 문제가 있어 반납 전 챠시를 거치대
            # 근처로 순간이동시킨다(정식 미션 4~5단계는 Nav2 복귀 leg 가 이 역할을 함).
            # ★주의★ : reposition_carter2_near() 는 USD xform 을 직접 덮어쓰는 방식이라 Play 이전
            # (물리 시작 전) 씬 authoring 에만 유효하다 — 물리가 이미 돌고 있는 상태(여기, 파지+스윕
            # 이후)에 그대로 쓰면 물리엔진이 자체 시뮬레이션 pose 를 계속 갖고 있어서 팔(과 붙어있는
            # 노즐)이 실제로는 안 움직이거나 어긋나고, USD 로만 옮겨진 챠시와 시각적으로 분리돼 보이는
            # 버그를 실측으로 확인함("팔이 노바카터·노즐과 떨어져 다른 위치로 이동"). 물리가 살아있는
            # 라이브 articulation 은 반드시 물리 인지 API(SingleManipulator.set_world_pose)로 옮겨야
            # 팔·챠시·그리퍼가 붙잡은 노즐까지 전부 일관되게 같이 이동한다.
            _dock_yaw = 0.0
            _tx = float(NOZZLE_DOCK_XY[0] - TC_DEBUG_CHASSIS_OFFSET_X)
            _ty = float(NOZZLE_DOCK_XY[1])
            c2_robot.set_world_pose(
                position=np.array([_tx, _ty, 0.0]),
                orientation=np.array([np.cos(_dock_yaw / 2.0), 0.0, 0.0, np.sin(_dock_yaw / 2.0)]),
            )
            sync_rmpflow_base_pose(ctx)
            for _ in range(15):
                my_world.step(render=True)
            print(f"[DEBUG] carter2 라이브 재배치(set_world_pose) → ({_tx:.2f},{_ty:.2f})")
        print("[TOOLCHANGE][DEBUG] 반납 시퀀스 시작")
        release_ok = drive_generator_to_completion(g_tool_change_release(ctx), my_world)
        print(f"[TOOLCHANGE][DEBUG] 반납 결과 = {release_ok}")

    print("=" * 64)
    label = "3/5 단계(분사 스윕 제너레이터)" if SPRAY_DEBUG_ONLY else "2/5 단계(툴체인지 제너레이터)"
    print(f"[INFO] {label} 디버그 스모크 완료.")
    print("=" * 64)

    if ros_node is not None:
        try:
            ros_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    if os.environ.get("ISAAC_HEADLESS", "0") == "1":
        _shutdown(); return
    print("[INFO] GUI 모드 — 창을 직접 닫을 때까지 대기합니다.")
    while simulation_app.is_running():
        my_world.step(render=True)
    _shutdown()


def _shutdown():
    """카메라/ROS2 브리지가 활성화된 무거운 씬은 simulation_app.close() 를 곧바로 부르면
    Kit 종료 시퀀스에서 세그폴트가 나는 걸 확인함(1/5 단계 첫 헤드리스 실행에서 재현) — 13_
    multi_robot_integrated.py 의 finally 블록과 동일하게 타임라인을 먼저 정지시키고 닫는다."""
    try:
        omni.timeline.get_timeline_interface().stop()
    except Exception as e:
        print(f"[SHUTDOWN][WARN] 타임라인 정지 실패: {e}")
    simulation_app.close()


if __name__ == "__main__":
    main()
