"""
2_tool_changer_nozzle_demo.py
-----------------------------------
"SG를 이용한 Tool Changer 개발" 설계 문서(2026-07-24) 검증/롤아웃 계획 1단계.

1b_tool_changer_demo_no_gripper.py 를 변형 : placeholder DynamicCuboid mop 대신
실제 m0609_with_nozzle.usd 의 nozzle_base_link 서브트리를 거치대에 놓고, carter2와
동일한 맨몸 m0609 구성(SingleManipulator(gripper=None), link_6 에 Surface Gripper
직접 부착)으로 접근→파지→흔들기(파지 안정성)→반납까지 검증한다.

USD 자산 사전조사(시스템 python3 + pxr, Isaac 미기동 상태로 확인) :
  - tool0_to_nozzle FixedJoint 는 /World/m0609/joints/tool0_to_nozzle 에 있고
    nozzle_base_link 서브트리 "밖"(형제)이다 → primPath="/World/m0609/nozzle_base_link"
    만 참조하면 이 조인트 없이 깨끗하게 가져와진다(설계문서 열린위험 #1 해소).
    새 설계에서는 이 조인트가 필요 없다(carter2 쪽 Surface Gripper D6 조인트로 새로 붙임).
  - nozzle_base_link 는 PhysicsRigidBodyAPI+MassAPI(0.45kg)를 가진 독립 강체이고,
    "로컬 프레임" 기준으로 원점(장착면, z=0)에서 +Z 방향으로 0.142m 뻗은 반지름 0.0315m
    원통형(콜리전 = Cylinder 6개 스택) 형상이다. 자식 nozzle_tcp 가 로컬 (0,0,0.142) =
    분사 팁(NOZZLE_TIP_LOCAL, carter1 상수와 일치 확인).
  - 즉 "원점 = 장착면"이라, carter1에 실제 마운트됐을 때와 마찬가지로 link_6 원점을
    이 원점에 맞추는 것이 자연스러운 파지 목표다.

거치대 설계 : 장착면을 바로 접근시키기엔(원점이 물체 맨 아래) 세워두면 바닥에 닿아
버려 팔이 접근 못 한다 → 눕혀서(원통 옆면이 바닥에 닿는 자세) 배치, 물리로 짧게
settle 시킨 뒤 "실제 정착 위치"를 다시 읽어(핸들 좌표 하드코딩 금지) 그 좌표로
접근한다 — 1_tool_changer_demo.py 의 교훈(선반 모서리 충돌) 대로 평평한 바닥 방식.

파지 성공 여부와 별개로, 파지 직후 + 흔들기 전 구간 동안 nozzle_base_link/nozzle_tcp 의
link_6 기준 상대 pose 를 test_nozzle_attach.py 와 동일한 방식으로 실측·추적한다.
이건 설계문서 3장("분사 조준 수학 — 상수 대신 그때그때 측정")이 요구하는 데이터이자,
Surface Gripper의 컴플라이언트 D6 조인트(스프링, grip_travel~1cm)가 분사 조준
정밀도에 실제로 얼마나 영향을 주는지 답하는 첫 실측치다. test_nozzle_attach.py 와
달리 여기선 PASS/FAIL 판정을 하지 않는다(고정 조인트 기준 5mm/1deg 는 애초에
컴플라이언트 조인트에 적용할 기준이 아님) — 그냥 편차를 데이터로 출력만 한다.

[2단계 추가, run2 실측 반영] 1단계(run2)에서 눕혀서 잡았더니 link_6 기준 오프셋이 Y로 틀어짐을
확인 → 거치 자세를 "매달기"(장착면 위/팁 아래, 임시 hold_joint 고정 후 파지 성공 시 해제)로 변경해
오프셋이 축(Z) 위주가 되도록 했다. 파지 직후 carter1(13_multi_robot_integrated.py)과 동일한
WALL_X/Z_LOW/Z_HIGH/조준 orientation 으로 LulaKinematicsSolver IK를 풀어보고(오프셋만 실측치로
교체), 실제로 q_low/q_high 로 이동시켜 그 자세에서도 파지가 유지되는지까지 확인한다.

실행 방법 (헤드리스 기본 — 자동 검증용. GUI 로 보려면 ISAAC_HEADLESS=0):
  /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
      src/isaac_wiping_task/2_tool_changer_nozzle_demo.py
"""

import os
import sys
from pathlib import Path

from isaacsim import SimulationApp

HEADLESS = os.environ.get("ISAAC_HEADLESS", "1") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.manipulators import SingleManipulator
import isaacsim.robot_motion.motion_generation as mg

_THIS_DIR = Path(__file__).resolve().parent
_WS_ROOT = _THIS_DIR.parents[1]  # cobot3_ws
RMPFLOW_DIR = str(_WS_ROOT / "src" / "integration" / "integration" / "rmpflow")
for p in (str(_THIS_DIR), RMPFLOW_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

import surface_gripper_utils  # noqa: E402
from m0609_rmpflow_controller import RMPFlowController  # noqa: E402
from tool_changer import ToolChangerController  # noqa: E402

# ─────────────────────────────────────────────────────────
# 설정값
# ─────────────────────────────────────────────────────────
# 그리퍼가 없는 순수 M0609 팔 (carter2 와 동일 자산 : C2_NO_GRIPPER_URDF 가 참조하는 것과 같은 usd).
USD_PATH = str(_WS_ROOT / "src" / "doosan-robot2" / "urdf" / "m0609_isaac_sim" / "m0609_isaac_sim.usd")
ROBOT_PRIM_PATH = "/m0609"
EE_LINK_NAME = "link_6"
RG2_FINGERTIP_LINK_NAME = "link_6"  # carter2의 C2_SURFACE_GRIPPER = .../link_6/mop_surface_gripper 와 동일 부착점

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8

# 그리퍼가 없으니 fingertip(link_6)과 IK 목표 프레임(tool0)이 사실상 같은 위치 (1b 와 동일 결론).
FINGERTIP_OFFSET_FROM_TOOL0 = np.zeros(3)

# ── 노즐 자산 ──
NOZZLE_USD = str(_WS_ROOT / "src" / "integration" / "integration" / "m0609_with_nozzle.usd")
NOZZLE_SOURCE_PRIMPATH = "/World/m0609/nozzle_base_link"
NOZZLE_TOOL_PATH = "/World/NozzleDock/nozzle_tool"
NOZZLE_TCP_PATH = f"{NOZZLE_TOOL_PATH}/nozzle_tcp"
NOZZLE_RADIUS = 0.0315  # 로컬 bbox 실측(pxr 오프라인 조사) — 눕힌 자세 정착 높이 추정에 사용
NOZZLE_LENGTH = 0.142

# 접근 orientation — identity(1,0,0,0)는 M0609 손목 특이점 근처라 쓰면 안 됨(교훈 #3).
# 1b_tool_changer_demo_no_gripper.py 가 동일 팔 자산·유사 거치 높이에서 이미 검증한 값 재사용.
APPROACH_ORIENTATION = np.array([0.0, 1.0, 0.0, 0.0])
# 거치대 반납 목표 자세도 접근과 동일 orientation 사용.
STAND_ORIENTATION = APPROACH_ORIENTATION

# ── 2단계(run2 결과 반영) : 눕혀서 거치 → "매달아" 거치로 변경 ──
# 1단계 실측(2_tool_changer_nozzle_demo.py run2 로그)에서, 눕혀서 잡았더니 link_6 기준
# 노즐 오프셋이 축(Z)이 아니라 Y로 틀어졌다 — Surface Gripper는 방향과 무관하게 거리만
# 보는 "자석"이라 거치 자세 그대로 잡히기 때문. carter1의 aim_link6_world() 는 "장착면→팁이
# link_6 로컬 Z와 나란함"을 전제하므로, 이번엔 그 조건에 맞게 거치 자세를 바꾼다.
# APPROACH_ORIENTATION=(0,1,0,0)(X축 180도 회전)을 로컬 Z=(0,0,1)에 적용하면 world (0,0,-1)
# 이 나온다 — 즉 이 접근 orientation에서 link_6 로컬 Z축은 world -Z(아래)를 향한다. 노즐도
# 로컬 +Z(장착면→팁)가 world -Z를 향하도록 세워 두면(=동일 orientation 적용) 두 로컬 Z축이
# 나란해져 파지 시 link_6 기준 오프셋이 (이상적으로는) 순수 Z축 성분이 된다.
# 이 자세는 "장착면(원점)이 위, 팁이 아래로 매달림" 형태라 팁이 바닥에 닿지 않게 띄워야 한다.
DOCK_HOLD_ORIENTATION = APPROACH_ORIENTATION
DOCK_XY = np.array([0.5, 0.0])
DOCK_HEIGHT = 0.25  # 장착면(원점) 높이. 팁(로컬 0.142m 아래) = 0.25-0.142 = 0.108m, 바닥 위 여유 있음.
# 실제 소켓/지그처럼, 접근 전까지는 이 자세로 "임시 거치 조인트"에 고정해 둔다(원통이라 눕혀두면
# 콜리전이 실린더 스택이라 굴러갈 위험도 있음 — 교훈 #2 확장: 평평한 면이든 매달기든, 예측 가능한
# 고정 자세로 두는 게 접근/파지 정합성에 유리). 파지 성공 확인 직후 이 조인트를 비활성화한다.
NOZZLE_HOLD_JOINT_PATH = "/World/NozzleDock/hold_joint"

EE_OFFSET = np.array([0.0, 0.0, 0.15])
GRASP_APPROACH_CLEARANCE = np.zeros(3)

PHYSICS_DT = 1.0 / 60.0
# [GUI 확인 후 수정] 원래 0.03(1b 의 헐거운 floor-mop 기준 재사용) 이었는데, 하강 이동이
# "settle step=0"(램프 종료 직후 첫 판정)에 3cm 이내라는 이유만으로 즉시 "도달"로 끝나버려
# 실제로는 목표보다 2.15cm 못 미친 채(z=0.2715 vs 목표 0.25) 파지가 걸렸다 — 측정된 gap(2.03cm)과
# 정확히 일치. Surface Gripper 파라미터 문제가 아니라 이 도달판정 오차였다. 5mm로 조인다.
POSITION_TOLERANCE = 0.005
MAX_APPROACH_STEPS = 400

MAX_EE_LINEAR_SPEED = 0.10
MIN_INTERP_STEPS = 60
MAX_INTERP_STEPS = 600

SHAKE_MAX_LINEAR_SPEED = 0.08
SHAKE_HOLD_STEPS = 30

SETTLE_STEPS_AFTER_RESET = 30  # hold_joint 로 고정되므로 드리프트는 없음 — 물리 초기화 안정화용 소량만.

# Surface Gripper 튜닝값 — "강한 자석" 쪽으로 여유를 준 1차 시도값. 기본(0.02/5000/100)보다
# 붙잡는 범위를 넓혀(0.04) 첫 시도에서 위치 오차로 인한 grip 실패 가능성을 줄인다.
# 실측 후 필요하면 좁혀도 된다(설계문서 3장: 실측 기반으로 오차 흡수).
MAX_GRIP_DISTANCE = 0.04
GRIP_DRIVE_STIFFNESS = 5000.0
GRIP_DRIVE_DAMPING = 100.0
# [2단계 GUI 확인 후 튜닝] run4 실측 : link_6-nozzle_base_link 파지 후 간격 ≈ 2.03cm.
# setup_mop_surface_gripper() 기본값 clearance_offset=0.008 + grip_travel=0.01(합 0.018m)이
# 원인으로 보여 둘 다 줄인다. clearance_offset 은 "완전히 붙어도 남기는 최소 여유", grip_travel 은
# D6 조인트가 물체를 끌어당길 수 있는 최대 이동범위 — MAX_GRIP_DISTANCE(붙잡는 탐지범위, 0.04)와는
# 별개다. 0으로 완전히 죽이지 않는 이유: 약간의 컴플라이언스가 있어야 접근좌표가 완벽하지 않아도
# grip 시 튕겨나가지 않는다(surface_gripper_utils.py 상단 docstring 참고).
CLEARANCE_OFFSET = 0.002
GRIP_TRAVEL = 0.004

# ── 2단계 : 조준 IK(LulaKinematicsSolver) 검증 — 설계문서 검증계획 항목 2 ──
# WALL_X/AIM_Y/Z_LOW/Z_HIGH 는 carter1(13_multi_robot_integrated.py)이 이미 라이브로 검증한 값을
# 그대로 재사용한다 — "같은 팔 모델, 같은 base 높이에서 carter1과 동일한 조준 범위(벽까지 거리·
# 분사 상/하한 높이)가 carter2(Surface Gripper로 잡은 노즐)에서도 풀리는가"를 직접 비교 가능하게
# 하기 위함. IK 는 URDF 기구학 체인에 노즐을 반영하지 않고(맨몸 URDF) link_6 기준 오프셋만큼
# 목표를 되돌려 계산 — carter1은 이 오프셋이 고정상수(NOZZLE_OFFSET)였지만, 여기선 1단계에서
# 실측한 tcp_rel_pos0(파지 직후 link_6 기준 nozzle_tcp 상대위치, 3축 전체)를 그대로 쓴다
# (aim_link6_world_from_offset — carter1의 aim_link6_world 를 스칼라 Z 오프셋에서 3축 벡터로 일반화).
IK_URDF_PATH = str(_WS_ROOT / "isaacpjt" / "M0609" / "rmpflow" / "m0609_isaac_sim.urdf")
IK_DESCRIPTION_PATH = str(_WS_ROOT / "isaacpjt" / "M0609" / "rmpflow" / "m0609_description.yaml")
WALL_X = 0.575
AIM_Y = 0.0
Z_LOW = 0.12
Z_HIGH = 0.80
IK_POSITION_TOLERANCE = 0.005
IK_ORIENTATION_TOLERANCE = 0.05
AIM_MOVE_SETTLE_STEPS = 90  # q_low/q_high 로 실제 이동 후 파지 유지를 확인할 정착 시간


# ─────────────────────────────────────────────────────────
# 헬퍼
# ─────────────────────────────────────────────────────────
def find_prim_path_by_name(root_path: str, name: str):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def get_prim_world_position(prim_path: str) -> np.ndarray:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    matrix = omni.usd.get_world_transform_matrix(prim)
    translation = matrix.ExtractTranslation()
    return np.array([translation[0], translation[1], translation[2]])


def read_world_pose(prim_path: str):
    """test_nozzle_attach.py 와 동일 방식(위치 np3, 쿼터니언 wxyz np4)."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = Gf.Transform(m)
    tr = t.GetTranslation()
    q = t.GetRotation().GetQuat()
    pos = np.array([tr[0], tr[1], tr[2]])
    quat = np.array([q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]])
    return pos, quat


def quat_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def relative_pose(parent_path: str, child_path: str):
    """child 를 parent 프레임에서 본 (상대위치 np3, 상대회전행렬 3x3). test_nozzle_attach.py 와 동일."""
    p_pos, p_quat = read_world_pose(parent_path)
    c_pos, c_quat = read_world_pose(child_path)
    R_p = quat_to_matrix(p_quat)
    rel_pos = R_p.T @ (c_pos - p_pos)
    rel_R = R_p.T @ quat_to_matrix(c_quat)
    return rel_pos, rel_R


def rot_angle_deg(R):
    ang = np.arccos(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    return np.degrees(ang)


def matrix_to_quat_wxyz(R):
    """13_multi_robot_integrated.py 와 동일 구현."""
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


def spray_orientation_quat():
    """carter1(13_multi_robot_integrated.py)과 동일한 조준 EE orientation."""
    R = np.array([[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]])
    return matrix_to_quat_wxyz(R)


def local_to_world(base_pos, base_quat, p_local):
    return base_pos + quat_to_matrix(base_quat) @ np.asarray(p_local)


def aim_link6_world_from_offset(base_pos, base_quat, ori_quat, z, tip_offset_link6_frame):
    """carter1의 aim_link6_world() 를 스칼라 Z 오프셋(NOZZLE_OFFSET)에서 3축 벡터로 일반화한 버전.
    tip_offset_link6_frame 은 파지 직후 실측한 link_6 기준 노즐팁 상대위치(3축)."""
    world_offset = quat_to_matrix(ori_quat) @ np.asarray(tip_offset_link6_frame)
    return local_to_world(base_pos, base_quat, [WALL_X, AIM_Y, z]) - world_offset


def spawn_nozzle_tool(stage):
    """m0609_with_nozzle.usd 의 nozzle_base_link 서브트리만 거치대 경로에 참조.
    tool0_to_nozzle 조인트는 이 서브트리 밖(형제)이라 자동으로 딸려오지 않는다.

    DOCK_HOLD_ORIENTATION 자세로 매달아 배치하고, world 에 고정하는 임시
    FixedJoint(NOZZLE_HOLD_JOINT_PATH)를 함께 authoring 한다 — 파지 전까지는 이 조인트가
    자세를 유지하고, 파지 성공 확인 직후 release_hold_joint() 로 비활성화한다."""
    UsdGeom.Xform.Define(stage, "/World/NozzleDock")
    tool_prim = stage.DefinePrim(NOZZLE_TOOL_PATH, "Xform")
    tool_prim.GetReferences().AddReference(
        Sdf.Reference(assetPath=NOZZLE_USD, primPath=NOZZLE_SOURCE_PRIMPATH)
    )
    for _ in range(20):
        simulation_app.update()
    if not stage.GetPrimAtPath(NOZZLE_TOOL_PATH).IsValid():
        raise RuntimeError(f"{NOZZLE_TOOL_PATH} 로드 실패 — NOZZLE_USD/primPath 확인 필요")

    # 매달린 자세로 배치 (원본 xformOps 는 로봇 마운트 시 rest pose 기준이라 무의미 → 덮어씀).
    xf = UsdGeom.Xformable(tool_prim)
    xf.ClearXformOpOrder()
    rot = Gf.Rotation(Gf.Quatd(float(DOCK_HOLD_ORIENTATION[0]),
                                Gf.Vec3d(*DOCK_HOLD_ORIENTATION[1:])))
    m = Gf.Matrix4d().SetRotate(rot).SetTranslateOnly(
        Gf.Vec3d(float(DOCK_XY[0]), float(DOCK_XY[1]), float(DOCK_HEIGHT))
    )
    xf.AddTransformOp().Set(m)
    print(f"[INFO] 노즐 거치대 배치 = ({DOCK_XY[0]:.3f},{DOCK_XY[1]:.3f},{DOCK_HEIGHT:.3f}), "
          f"매달린 자세(장착면 위, 팁 아래)")

    # 임시 거치 조인트 : 정적 anchor prim(RigidBodyAPI 없음 = PhysX 가 world-fixed 로 취급)을
    # body0 으로 명시 지정 — body0 을 아예 비워두면 joint frame0 이 world 원점(0,0,0)으로
    # 취급돼(joint 자체의 xformOp 이 아니라 localPos0/localRot0 만으로 정의되는데 둘 다 기본
    # identity) 실제 배치 위치(DOCK_XY, DOCK_HEIGHT)와 충돌 → 헤드리스로 직접 확인: 첫 스텝부터
    # 좌표가 1e6 단위로 발산(물리 폭발)했다. anchor 를 명시하고 localPos0=0/localRot0=identity 로
    # "anchor 의 world pose 자체가 곧 joint frame" 이 되게 하면 안전하다.
    anchor_path = "/World/NozzleDock/hold_anchor"
    anchor_prim = stage.DefinePrim(anchor_path, "Xform")
    anchor_xf = UsdGeom.Xformable(anchor_prim)
    anchor_xf.ClearXformOpOrder()
    anchor_xf.AddTransformOp().Set(m)  # m = 위에서 계산한 노즐과 동일한 배치 행렬

    hold_joint = UsdPhysics.FixedJoint.Define(stage, NOZZLE_HOLD_JOINT_PATH)
    hold_joint.CreateBody0Rel().SetTargets([anchor_path])
    hold_joint.CreateBody1Rel().SetTargets([NOZZLE_TOOL_PATH])
    hold_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    hold_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    hold_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    hold_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    hold_joint.CreateExcludeFromArticulationAttr().Set(True)
    hold_joint.CreateJointEnabledAttr().Set(True)
    print(f"[INFO] 임시 거치 조인트 authoring = {NOZZLE_HOLD_JOINT_PATH} (anchor={anchor_path})")
    return NOZZLE_TOOL_PATH


def release_hold_joint(stage):
    """파지 성공 확인 후 임시 거치 조인트를 비활성화 — 이후로는 Surface Gripper D6 조인트만
    노즐을 붙잡는다(진짜 소켓/지그에서 뽑아내는 것과 동일한 개념)."""
    prim = stage.GetPrimAtPath(NOZZLE_HOLD_JOINT_PATH)
    UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(False)
    print(f"[INFO] 임시 거치 조인트 비활성화 = {NOZZLE_HOLD_JOINT_PATH}")


def engage_hold_joint(stage):
    """재파지 사이클 테스트용 — 반납 직후(자유낙하 시간이 짧을 때) 다시 활성화해 노즐을
    anchor 자세(DOCK_XY, DOCK_HEIGHT)로 되돌린다. 낙하 시간이 길면(수백 ms 이상) 큰 위치
    오차를 한 번에 강체 조인트로 보정하게 돼 불안정할 수 있으므로, 호출부에서 release
    직후 짧게(수 스텝)만 지나고 나서 불러야 한다."""
    prim = stage.GetPrimAtPath(NOZZLE_HOLD_JOINT_PATH)
    UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(True)
    print(f"[INFO] 임시 거치 조인트 재활성화 = {NOZZLE_HOLD_JOINT_PATH}")


def set_drive_gains(stage, root_path: str):
    drive_count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        for dof_type in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, dof_type)
            if drive:
                drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                drive.GetDampingAttr().Set(DRIVE_DAMPING)
                drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                drive_count += 1
    print(f"[INFO] 관절 드라이브 강성/댐핑 설정 완료: {drive_count}개")


def _smoothstep(alpha: float) -> float:
    return alpha * alpha * (3.0 - 2.0 * alpha)


def _ramp_steps_for_distance(distance: float, max_linear_speed: float) -> int:
    raw_steps = distance / max_linear_speed / PHYSICS_DT
    return int(np.clip(round(raw_steps), MIN_INTERP_STEPS, MAX_INTERP_STEPS))


def _ramp_ee_target(world, robot, rmpflow, tool0_path, target_position, target_orientation, ramp_steps: int):
    start_position = get_prim_world_position(tool0_path)
    for step in range(ramp_steps):
        world.step(render=True)
        alpha = (step + 1) / ramp_steps
        waypoint = start_position + _smoothstep(alpha) * (target_position - start_position)
        action = rmpflow.forward(
            target_end_effector_position=waypoint,
            target_end_effector_orientation=target_orientation,
        )
        robot.apply_action(action)


def move_to_pose(
    world, robot, rmpflow, tool0_path, target_position, target_orientation, label: str,
    max_linear_speed: float = MAX_EE_LINEAR_SPEED,
):
    start_position = get_prim_world_position(tool0_path)
    distance = float(np.linalg.norm(target_position - start_position))
    ramp_steps = _ramp_steps_for_distance(distance, max_linear_speed)
    print(
        f"[INFO] {label} 목표 {target_position} 로 이동 시작 "
        f"(start={start_position}, dist={distance:.3f}m, ramp_steps={ramp_steps})..."
    )
    _ramp_ee_target(world, robot, rmpflow, tool0_path, target_position, target_orientation, ramp_steps)

    for step in range(MAX_APPROACH_STEPS):
        world.step(render=True)
        ee_position = get_prim_world_position(tool0_path)
        action = rmpflow.forward(
            target_end_effector_position=target_position,
            target_end_effector_orientation=target_orientation,
        )
        robot.apply_action(action)
        if ToolChangerController.is_at_pose(ee_position, target_position, POSITION_TOLERANCE):
            print(f"[INFO] {label} 도달 (settle step={step}, ee={ee_position})")
            return True
    final_pos = get_prim_world_position(tool0_path)
    print(
        f"[WARN] {label} 이동이 {MAX_APPROACH_STEPS} step 내에 목표 허용오차 이내로 수렴하지 못했습니다. "
        f"final_ee={final_pos} dist={np.linalg.norm(final_pos - target_position):.4f}"
    )
    return False


def shake_test(world, robot, rmpflow, tool0_path, tool_changer, center_position, orientation,
                link6_path, rel_pos0, rel_R0):
    """파지 후 노즐이 안 떨어지는지 + link_6 기준 상대 pose 가 얼마나 흔들리는지(=Surface
    Gripper 컴플라이언스가 조준 정밀도에 주는 영향의 실측치) 함께 기록한다."""
    offsets = [
        np.array([0.06, 0.0, 0.0]),
        np.array([-0.06, 0.0, 0.0]),
        np.array([0.0, 0.06, 0.0]),
        np.array([0.0, -0.06, 0.0]),
        np.array([0.0, 0.0, 0.0]),
    ]
    max_pos_dev = 0.0
    max_ang_dev = 0.0
    for i, offset in enumerate(offsets):
        target = center_position + offset
        start_position = get_prim_world_position(tool0_path)
        distance = float(np.linalg.norm(target - start_position))
        ramp_steps = _ramp_steps_for_distance(distance, SHAKE_MAX_LINEAR_SPEED)
        print(f"[INFO] 흔들기 {i + 1}/{len(offsets)} -> {target} (ramp_steps={ramp_steps})")
        _ramp_ee_target(world, robot, rmpflow, tool0_path, target, orientation, ramp_steps)

        for _ in range(SHAKE_HOLD_STEPS):
            world.step(render=True)
            action = rmpflow.forward(target_end_effector_position=target, target_end_effector_orientation=orientation)
            robot.apply_action(action)

        still_closed = tool_changer.surface_gripper.is_closed()
        rel_pos, rel_R = relative_pose(link6_path, NOZZLE_TOOL_PATH)
        pos_dev = float(np.linalg.norm(rel_pos - rel_pos0))
        ang_dev = float(rot_angle_deg(rel_R0.T @ rel_R))
        max_pos_dev = max(max_pos_dev, pos_dev)
        max_ang_dev = max(max_ang_dev, ang_dev)
        print(f"[CHECKPOINT] 흔들기 {i + 1} 후 is_closed()={still_closed}, "
              f"link6 기준 노즐 상대pose 편차 = {pos_dev*1000:.2f}mm / {ang_dev:.3f}deg "
              f"(기준 대비 누적최대 {max_pos_dev*1000:.2f}mm / {max_ang_dev:.3f}deg)")
    return max_pos_dev, max_ang_dev


def main():
    stage = omni.usd.get_context().get_stage()
    stage.GetRootLayer().subLayerPaths.append(USD_PATH)
    for _ in range(30):
        simulation_app.update()

    world = World(physics_dt=PHYSICS_DT)
    world.scene.add_default_ground_plane()

    set_drive_gains(stage, ROBOT_PRIM_PATH)

    ee_prim_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
    if ee_prim_path is None:
        raise RuntimeError(f"'{EE_LINK_NAME}' 링크를 {ROBOT_PRIM_PATH} 하위에서 찾지 못했습니다.")
    print(f"[INFO] End Effector rigid body ({EE_LINK_NAME}) = {ee_prim_path}")

    tool0_path = find_prim_path_by_name(ROBOT_PRIM_PATH, "tool0")
    if tool0_path is None:
        raise RuntimeError(f"'tool0' 프레임을 {ROBOT_PRIM_PATH} 하위에서 찾지 못했습니다.")
    print(f"[INFO] IK 목표 프레임(tool0) = {tool0_path}")

    fingertip_path = find_prim_path_by_name(ROBOT_PRIM_PATH, RG2_FINGERTIP_LINK_NAME)
    if fingertip_path is None:
        raise RuntimeError(f"'{RG2_FINGERTIP_LINK_NAME}' 링크를 {ROBOT_PRIM_PATH} 하위에서 찾지 못했습니다.")
    print(f"[INFO] Surface Gripper 부착점 = {fingertip_path}")

    # world.reset()/robot.initialize() 이전에 Surface Gripper D6 조인트 authoring (교훈 #1).
    surface_gripper_prim_path = surface_gripper_utils.setup_mop_surface_gripper(
        stage, fingertip_prim_path=fingertip_path,
        max_grip_distance=MAX_GRIP_DISTANCE,
        grip_drive_stiffness=GRIP_DRIVE_STIFFNESS,
        grip_drive_damping=GRIP_DRIVE_DAMPING,
        clearance_offset=CLEARANCE_OFFSET,
        grip_travel=GRIP_TRAVEL,
    )

    robot = world.scene.add(
        SingleManipulator(
            prim_path=ROBOT_PRIM_PATH,
            name="m0609_robot",
            end_effector_prim_path=ee_prim_path,
            gripper=None,
        )
    )

    # 노즐도 world.reset() 이전에 씬에 추가 (교훈 #5: 동적 추가 시 브로드페이즈 깨짐).
    spawn_nozzle_tool(stage)

    world.reset()
    robot.initialize()

    robot.set_joint_positions(np.zeros(robot.num_dof))
    for _ in range(10):
        world.step(render=True)

    # hold_joint 로 고정돼 있어 물리 초기화만 잠깐 안정화시킨다(드리프트 없어야 정상).
    for _ in range(SETTLE_STEPS_AFTER_RESET):
        world.step(render=True)
    settled_pos = get_prim_world_position(NOZZLE_TOOL_PATH)
    _dock_target = np.array([DOCK_XY[0], DOCK_XY[1], DOCK_HEIGHT])
    print(f"[INFO] 노즐 world position(hold_joint 고정) = {settled_pos} "
          f"(배치 목표 대비 드리프트 {np.linalg.norm(settled_pos - _dock_target):.4f}m — 0에 가까워야 정상)")

    rmpflow = RMPFlowController(
        name="tool_changer_cspace_controller",
        robot_articulation=robot,
    )

    tool_changer = ToolChangerController(
        rg2_fingertip_prim_path=fingertip_path,
        mop_handle_prim_path=NOZZLE_TOOL_PATH,  # nozzle_base_link 원점 = 장착면 = 자연스러운 파지 목표
        stand_position=np.array([DOCK_XY[0], DOCK_XY[1], DOCK_HEIGHT]),
        stand_orientation=STAND_ORIENTATION,
        approach_orientation=APPROACH_ORIENTATION,
        fingertip_offset_from_ik_frame=FINGERTIP_OFFSET_FROM_TOOL0,
        rg2_gripper=None,
        surface_gripper_prim_path=surface_gripper_prim_path,
        auto_create_surface_gripper=False,
    )
    tool_changer.initialize()

    for _ in range(30):
        world.step(render=True)

    # 1) 노즐 위 여유 공간으로 접근 후 하강 (실측 정착 위치 기준, 하드코딩 아님)
    handle_position, handle_orientation = tool_changer.approach_tool_stand()
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        handle_position + EE_OFFSET, handle_orientation, "노즐 상공 접근",
    )
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        handle_position + GRASP_APPROACH_CLEARANCE, handle_orientation, "노즐 하강",
    )

    # 2) 파지
    tool_changer.grasp_mop()
    for _ in range(30):
        world.step(render=True)

    grasp_ok = tool_changer.surface_gripper.is_closed()
    if not grasp_ok:
        print("[FAIL] 파지 실패 — 흔들기/실측 단계를 건너뜁니다. "
              "MAX_GRIP_DISTANCE·접근좌표·거치 자세를 조정해 재시도하세요.")
        simulation_app.close()
        return

    # 파지 성공 확인됨 — 임시 거치 조인트 해제(이후로는 Surface Gripper만 노즐을 붙잡는다).
    release_hold_joint(stage)
    for _ in range(15):
        world.step(render=True)

    # 3) 파지 직후 실측 : link_6 기준 노즐 상대 pose (설계문서 3장의 "그때그때 측정" 데이터)
    rel_pos0, rel_R0 = relative_pose(ee_prim_path, NOZZLE_TOOL_PATH)
    tcp_rel_pos0, tcp_rel_R0 = relative_pose(ee_prim_path, NOZZLE_TCP_PATH)
    print("=" * 64)
    print(f"[MEASURED] link_6 기준 nozzle_base_link 상대위치 = {np.round(rel_pos0, 4)} m "
          f"(|rel|={np.linalg.norm(rel_pos0):.4f})")
    print(f"[MEASURED] link_6 기준 nozzle_tcp(분사팁) 상대위치 = {np.round(tcp_rel_pos0, 4)} m "
          f"(|rel|={np.linalg.norm(tcp_rel_pos0):.4f}) "
          f"— carter1 고정상수(NOZZLE_OFFSET=0.1392, NOZZLE_TIP_LOCAL=0.142)와 비교 참고용")
    print("=" * 64)

    # 3.5) 조준 IK 검증 (설계문서 검증계획 2단계) : carter1과 동일 WALL_X/Z_LOW/Z_HIGH·조준
    # orientation, 오프셋만 위에서 실측한 tcp_rel_pos0(3축)로 교체.
    base_pos, base_quat = read_world_pose(f"{ROBOT_PRIM_PATH}/base_link")
    ori = spray_orientation_quat()
    ik = mg.LulaKinematicsSolver(robot_description_path=IK_DESCRIPTION_PATH, urdf_path=IK_URDF_PATH)
    ik.set_robot_base_pose(base_pos, base_quat)
    q_low, ok_lo = ik.compute_inverse_kinematics(
        EE_LINK_NAME, aim_link6_world_from_offset(base_pos, base_quat, ori, Z_LOW, tcp_rel_pos0), ori,
        position_tolerance=IK_POSITION_TOLERANCE, orientation_tolerance=IK_ORIENTATION_TOLERANCE)
    q_high, ok_hi = ik.compute_inverse_kinematics(
        EE_LINK_NAME, aim_link6_world_from_offset(base_pos, base_quat, ori, Z_HIGH, tcp_rel_pos0), ori,
        warm_start=(q_low if ok_lo else None),
        position_tolerance=IK_POSITION_TOLERANCE, orientation_tolerance=IK_ORIENTATION_TOLERANCE)
    print("=" * 64)
    print(f"[IK] carter1과 동일 WALL_X={WALL_X} Z_LOW={Z_LOW} Z_HIGH={Z_HIGH} (오프셋만 실측치로 교체)")
    print(f"[IK] q_low  ok={ok_lo}" + (f"  q={np.round(np.asarray(q_low[:6]), 4)}" if ok_lo else ""))
    print(f"[IK] q_high ok={ok_hi}" + (f"  q={np.round(np.asarray(q_high[:6]), 4)}" if ok_hi else ""))
    ik_solved = bool(ok_lo and ok_hi)
    if not ik_solved:
        print("[WARN] 조준 IK 미해결 — 거치 자세/오프셋/WALL_X 재확인 필요.")

    ik_hold = {"low": None, "high": None}
    ik_dev = {"low": None, "high": None}
    if ik_solved:
        for label, q_target in (("low", np.asarray(q_low[:6])), ("high", np.asarray(q_high[:6]))):
            action = ArticulationAction(joint_positions=q_target)
            for _ in range(AIM_MOVE_SETTLE_STEPS):
                world.step(render=True)
                robot.apply_action(action)
            still_closed = tool_changer.surface_gripper.is_closed()
            rel_pos, rel_R = relative_pose(ee_prim_path, NOZZLE_TOOL_PATH)
            pos_dev = float(np.linalg.norm(rel_pos - rel_pos0))
            ang_dev = float(rot_angle_deg(rel_R0.T @ rel_R))
            ik_hold[label] = still_closed
            ik_dev[label] = (pos_dev, ang_dev)
            print(f"[CHECKPOINT] q_{label} 자세 이동 후 is_closed()={still_closed}, "
                  f"link6 기준 노즐 상대pose 편차 = {pos_dev*1000:.2f}mm / {ang_dev:.3f}deg")
    print("=" * 64)

    # 4) 흔들기 전, 상공으로 물러난다.
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        handle_position + EE_OFFSET, handle_orientation, "파지 후 흔들기 대기 위치로 상승",
    )

    # 5) 흔들기 + 컴플라이언스 편차 실측
    max_pos_dev, max_ang_dev = shake_test(
        world, robot, rmpflow, tool0_path, tool_changer,
        center_position=handle_position + EE_OFFSET, orientation=handle_orientation,
        link6_path=ee_prim_path, rel_pos0=rel_pos0, rel_R0=rel_R0,
    )
    held_after_shake = tool_changer.surface_gripper.is_closed()

    # 6) 거치대로 복귀 + 반납
    stand_position, stand_orientation = tool_changer.stand_return_target()
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        stand_position + EE_OFFSET, stand_orientation, "거치대 상공 복귀",
    )
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        stand_position + GRASP_APPROACH_CLEARANCE, stand_orientation, "거치대 하강",
    )
    tool_changer.release_mop_to_stand()
    # 재도킹까지 자유낙하 시간을 짧게 유지(engage_hold_joint 안전 여유) — 60스텝(1s)이면 낙하가
    # 너무 커진다(교훈: 큰 오차를 강체 조인트로 한번에 보정하면 hold_joint 최초버전 폭발과 동일한
    # 위험). 15스텝(0.25s, 자유낙하 약 3cm)이면 재고정 시 안전하다.
    for _ in range(15):
        world.step(render=True)
    release_ok = not tool_changer.surface_gripper.is_closed()

    # ── 재파지 사이클 테스트 (2회차) ── 반납한 노즐을 다시 도킹시켜 두 번째로 접근→파지가
    # 되는지 확인한다. 실제 미션은 "쓰레기 수거↔소독"을 반복하며 노즐을 여러 번 집었다 놓을
    # 것이므로, 1회성 파지만으로는 부족하다.
    engage_hold_joint(stage)
    for _ in range(20):
        world.step(render=True)
    redock_pos = get_prim_world_position(NOZZLE_TOOL_PATH)
    redock_target = np.array([DOCK_XY[0], DOCK_XY[1], DOCK_HEIGHT])
    print(f"[INFO] 재도킹 후 노즐 위치 = {redock_pos} "
          f"(목표 대비 드리프트 {np.linalg.norm(redock_pos - redock_target):.4f}m)")

    print("[INFO] ===== 2회차 파지 사이클 시작 =====")
    handle_position2, handle_orientation2 = tool_changer.approach_tool_stand()
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        handle_position2 + EE_OFFSET, handle_orientation2, "2회차: 노즐 상공 접근",
    )
    move_to_pose(
        world, robot, rmpflow, tool0_path,
        handle_position2 + GRASP_APPROACH_CLEARANCE, handle_orientation2, "2회차: 노즐 하강",
    )
    tool_changer.grasp_mop()
    for _ in range(30):
        world.step(render=True)
    grasp_ok2 = tool_changer.surface_gripper.is_closed()

    rel_pos0_2 = None
    if grasp_ok2:
        release_hold_joint(stage)
        for _ in range(15):
            world.step(render=True)
        rel_pos0_2, _ = relative_pose(ee_prim_path, NOZZLE_TOOL_PATH)
        print(f"[MEASURED] 2회차 link_6 기준 nozzle_base_link 상대위치 = {np.round(rel_pos0_2, 4)} m "
              f"(1회차 {np.round(rel_pos0, 4)} m 대비 재현성 참고)")
        # 정리 : 다시 반납해 스크립트를 처음과 동일한(파지 해제) 상태로 마친다.
        move_to_pose(
            world, robot, rmpflow, tool0_path,
            handle_position2 + EE_OFFSET, handle_orientation2, "2회차: 반납 전 상승",
        )
        move_to_pose(
            world, robot, rmpflow, tool0_path,
            handle_position2 + GRASP_APPROACH_CLEARANCE, handle_orientation2, "2회차: 거치대 하강",
        )
        tool_changer.release_mop_to_stand()
        for _ in range(30):
            world.step(render=True)
    else:
        print("[FAIL] 2회차 파지 실패 — 재도킹 좌표/자세가 1회차와 어긋났을 가능성.")

    print("=" * 64)
    print("[SUMMARY]")
    print(f"  파지 성공          : {grasp_ok}")
    print(f"  조준 IK(q_low/q_high) 해결 : {ik_solved} (low={ok_lo}, high={ok_hi})")
    if ik_solved:
        print(f"  q_low  이동 후 파지 유지 : {ik_hold['low']}  편차 {ik_dev['low'][0]*1000:.2f}mm/{ik_dev['low'][1]:.3f}deg")
        print(f"  q_high 이동 후 파지 유지 : {ik_hold['high']}  편차 {ik_dev['high'][0]*1000:.2f}mm/{ik_dev['high'][1]:.3f}deg")
    print(f"  흔들기 중 유지      : {held_after_shake if grasp_ok else 'N/A (파지실패)'}")
    print(f"  최대 위치 편차(흔들기): {max_pos_dev*1000:.2f} mm")
    print(f"  최대 자세 편차(흔들기): {max_ang_dev:.3f} deg")
    print(f"  반납(해제) 성공      : {release_ok}")
    print(f"  [재파지] 2회차 파지 성공 : {grasp_ok2}")
    if grasp_ok2:
        print(f"  [재파지] 2회차 gap  : {np.linalg.norm(rel_pos0_2)*1000:.2f} mm "
              f"(1회차 {np.linalg.norm(rel_pos0)*1000:.2f} mm)")
    print("  (참고: 여기엔 PASS/FAIL 임계값이 없음 — Surface Gripper 는 컴플라이언트 조인트라 "
          "test_nozzle_attach.py 의 5mm/1deg 고정조인트 기준이 애초에 안 맞음. 이 수치가 "
          "분사 조준에 허용 가능한 수준인지는 실제 WIPE 궤적 요구 정밀도와 비교해 판단할 것.)")
    print("=" * 64)

    if not HEADLESS:
        print("[INFO] GUI 모드 — 창을 직접 닫을 때까지 대기합니다.")
        while simulation_app.is_running():
            world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
