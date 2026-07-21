"""Isaac Sim 5.1 Surface Gripper 저작(authoring) 유틸리티.

``isaacsim.robot.manipulators.grippers.surface_gripper.SurfaceGripper`` 는
스테이지에 "이미 존재하는" Surface Gripper Prim(IsaacSurfaceGripper 스키마)에 대한
얇은 런타임 래퍼일 뿐, 프림 자체를 만들어주지 않는다. (Isaac Sim 5.1 기준,
end_effector_prim_path / surface_gripper_path 두 인자만 받는다.)

물리적으로 동작하는 Surface Gripper가 되려면 스테이지에 아래 두 가지가
authoring 되어 있어야 한다.

  1. Attachment Point 역할을 하는 D6 PhysicsJoint
     - body0 = 그리퍼가 부착되는 rigid body (여기서는 RG2 fingertip)
     - body1 = 같은 로봇 내의 다른 rigid body (placeholder). 실제로 무엇을
       붙잡을지는 PhysX SurfaceGripper 플러그인이 런타임에 body1을 갈아
       끼우며 결정하므로, 여기 지정하는 body1은 "잡을 대상"이 아니다.
  2. ``IsaacSurfaceGripper`` 스키마 프림 - attachmentPoints 관계로 위 조인트를
     참조하고, maxGripDistance / coaxialForceLimit / shearForceLimit /
     retryInterval 등을 속성으로 갖는다.

이 모듈의 필드 값과 스키마 구성은 로컬에 설치된 Isaac Sim 5.1의
``isaacsim.robot.surface_gripper`` 확장이 제공하는 공식 데모
(``data/SurfaceGripper_gantry.usda``)에 저작되어 있는 D6 조인트를 그대로
참고하여 동일하게 구성한 것이다 (임의 추정이 아님).
"""
from __future__ import annotations

import logging
from typing import Sequence

try:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    from usd.schema.isaac import robot_schema

    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False

logger = logging.getLogger(__name__)

# float32 최댓값. USD Physics의 breakForce/breakTorque "사실상 무한대" 관례값.
_FLT_MAX = 3.4028235e38


def create_attachment_point_joint(
    stage: "Usd.Stage",
    joint_path: str,
    body0_path: str,
    body1_path: str,
    local_pos0: Sequence[float] = (0.0, 0.0, 0.0),
    local_rot0: Sequence[float] = (1.0, 0.0, 0.0, 0.0),  # (w, x, y, z)
    local_pos1: Sequence[float] = (0.0, 0.0, 0.0),
    local_rot1: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    forward_axis: str = "Z",
    clearance_offset: float = 0.008,
    grip_travel: float = 0.01,
    grip_drive_stiffness: float = 5000.0,
    grip_drive_damping: float = 100.0,
    rot_drive_stiffness: float = 100.0,
    rot_z_drive_stiffness: float = 10000.0,
):
    """Surface Gripper의 Attachment Point로 쓰이는 D6 PhysicsJoint를 생성한다.

    forward_axis 방향의 translate 자유도만 0~grip_travel 범위로 열어 두고,
    나머지 자유도는 잠근다 (Isaac Sim 5.1 SurfaceGripper_gantry.usda 데모와 동일 구성).

    Args:
        stage: 대상 USD 스테이지.
        joint_path: 생성할 조인트 prim 경로.
        body0_path: 그리퍼 쪽 rigid body (예: RG2 fingertip) prim 경로.
        body1_path: placeholder rigid body prim 경로 (실제 파지 대상이 아님).
        local_pos0: body0 프레임 기준 조인트 로컬 위치.
        local_rot0: body0 프레임 기준 조인트 로컬 회전 (w, x, y, z).
        local_pos1: body1 프레임 기준 조인트 로컬 위치.
        local_rot1: body1 프레임 기준 조인트 로컬 회전 (w, x, y, z).
        forward_axis: 파지(접근) 방향 축. "X" | "Y" | "Z".
        clearance_offset: isaac:clearanceOffset (그리퍼 표면 여유 거리, m).
        grip_travel: 접근 축 자유도의 이동 허용 범위(m). 표면에 닿아 물체를
            "밀어 넣는" 정도의 여유값.
        grip_drive_stiffness / grip_drive_damping: 접근 축을 따라 물체를 attachment
            point 쪽으로 끌어당기는 드라이브 강성/댐핑. NVIDIA 공식 데모 기본값
            (5000/100)을 그대로 쓴다 — 그립 시점 정렬 오차가 크면(예: attachment
            point가 목표 지점에서 많이 벗어난 채로 grip을 시도하면) 이 값이 높아도
            낮아도 결국 "튕겨나가는" 것처럼 보일 수 있으니, 우선 접근 위치/충돌을
            바로잡는 게 먼저다.
        rot_drive_stiffness / rot_z_drive_stiffness: rotX/rotY 및 rotZ 자유도의
            드라이브 강성. NVIDIA 공식 데모 기본값(100 / 10000)을 그대로 쓴다.

    Returns:
        생성된 조인트의 Usd.Prim.
    """
    joint = UsdPhysics.Joint.Define(stage, joint_path)
    joint_prim = joint.GetPrim()

    joint.CreateBody0Rel().SetTargets([body0_path])
    joint.CreateBody1Rel().SetTargets([body1_path])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(local_rot0[0]), Gf.Vec3f(*local_rot0[1:])))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*local_pos1))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(float(local_rot1[0]), Gf.Vec3f(*local_rot1[1:])))
    joint.CreateBreakForceAttr().Set(_FLT_MAX)
    joint.CreateBreakTorqueAttr().Set(_FLT_MAX)
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateJointEnabledAttr().Set(True)

    # 접근(grip) 축을 제외한 모든 translate 자유도는 완전히 잠근다.
    # (Isaac Sim 데모 관례: low > high 로 두면 해당 축이 잠긴 것으로 취급된다)
    all_trans_axes = {"X", "Y", "Z"}
    grip_axis_letter = forward_axis.upper()
    for axis_letter in all_trans_axes - {grip_axis_letter}:
        limit = UsdPhysics.LimitAPI.Apply(joint_prim, f"trans{axis_letter}")
        limit.CreateLowAttr().Set(1.0)
        limit.CreateHighAttr().Set(-1.0)

    grip_axis = f"trans{grip_axis_letter}"
    grip_limit = UsdPhysics.LimitAPI.Apply(joint_prim, grip_axis)
    grip_limit.CreateLowAttr().Set(0.0)
    grip_limit.CreateHighAttr().Set(grip_travel)
    grip_drive = UsdPhysics.DriveAPI.Apply(joint_prim, grip_axis)
    grip_drive.CreateStiffnessAttr().Set(grip_drive_stiffness)
    grip_drive.CreateDampingAttr().Set(grip_drive_damping)

    # NVIDIA Isaac Sim 5.1 공식 데모(SurfaceGripper_gantry.usda) 기본값으로 복원:
    # grip축 stiffness=5000/damping=100, rotZ stiffness=10000, rotX/rotY stiffness=100.
    # (한때 "튕겨나가는" 문제 때문에 이 값들을 낮췄었는데, 실제 원인은 드라이브가
    # 아니라 접근 목표 지점의 오차/충돌이었음을 확인해서 원래 값으로 되돌린다.)
    for axis in ("rotX", "rotY", "rotZ"):
        limit = UsdPhysics.LimitAPI.Apply(joint_prim, axis)
        limit.CreateLowAttr().Set(-3.0)
        limit.CreateHighAttr().Set(3.0)
        drive = UsdPhysics.DriveAPI.Apply(joint_prim, axis)
        drive.CreateStiffnessAttr().Set(rot_z_drive_stiffness if axis == "rotZ" else rot_drive_stiffness)

    robot_schema.ApplyAttachmentPointAPI(joint_prim)
    joint_prim.GetAttribute(robot_schema.Attributes.FORWARD_AXIS.name).Set(grip_axis_letter)
    joint_prim.GetAttribute(robot_schema.Attributes.CLEARANCE_OFFSET.name).Set(clearance_offset)

    logger.info(f"[SurfaceGripper] Attachment point joint 생성: {joint_path} (body0={body0_path})")
    return joint_prim


def create_surface_gripper(
    stage: "Usd.Stage",
    gripper_prim_path: str,
    attachment_joint_paths: Sequence[str],
    max_grip_distance: float = 0.02,
    coaxial_force_limit: float = -1.0,
    shear_force_limit: float = -1.0,
    retry_interval: float = 1.0,
):
    """``IsaacSurfaceGripper`` 스키마 프림을 생성하고 attachment point들을 연결한다.

    coaxial_force_limit / shear_force_limit 는 -1이면 제한 없음 (USD 스키마 기본값).
    """
    gripper_prim = robot_schema.CreateSurfaceGripper(stage, gripper_prim_path)

    attachment_points_rel = gripper_prim.GetRelationship(robot_schema.Relations.ATTACHMENT_POINTS.name)
    attachment_points_rel.SetTargets(list(attachment_joint_paths))

    gripper_prim.GetAttribute(robot_schema.Attributes.MAX_GRIP_DISTANCE.name).Set(max_grip_distance)
    gripper_prim.GetAttribute(robot_schema.Attributes.COAXIAL_FORCE_LIMIT.name).Set(coaxial_force_limit)
    gripper_prim.GetAttribute(robot_schema.Attributes.SHEAR_FORCE_LIMIT.name).Set(shear_force_limit)
    gripper_prim.GetAttribute(robot_schema.Attributes.RETRY_INTERVAL.name).Set(retry_interval)

    logger.info(
        f"[SurfaceGripper] '{gripper_prim_path}' 생성 완료. attachment_points={list(attachment_joint_paths)}"
    )
    return gripper_prim


def setup_mop_surface_gripper(
    stage: "Usd.Stage",
    fingertip_prim_path: str,
    gripper_prim_path: str | None = None,
    joints_scope_path: str | None = None,
    body1_path: str | None = None,
    local_pos0: Sequence[float] = (0.0, 0.0, 0.0),
    forward_axis: str = "Z",
    max_grip_distance: float = 0.02,
    clearance_offset: float = 0.008,
    grip_travel: float = 0.01,
    grip_drive_stiffness: float = 5000.0,
    grip_drive_damping: float = 100.0,
    rot_drive_stiffness: float = 100.0,
    rot_z_drive_stiffness: float = 10000.0,
) -> str:
    """RG2 fingertip prim에 부착되는 Surface Gripper 세트(조인트 1개 + 스키마 프림)를
    한 번에 생성하는 편의 함수. 이미 존재하면 재생성하지 않고 경로만 반환한다.

    TODO: local_pos0 / forward_axis / max_grip_distance / clearance_offset 은
    실제 RG2 + 걸레 USD 자산 형상을 GUI에서 눈으로 확인하며 튜닝해야 하는
    placeholder 값이다. (기본값은 Isaac Sim 5.1 공식 데모 값)

    grip_drive_stiffness/damping, rot_drive_stiffness/rot_z_drive_stiffness,
    grip_travel, clearance_offset: 전부 NVIDIA 공식 데모 기본값으로 되돌려져 있다.
    (한때 "튕겨나가는" 문제 때문에 이 값들을 낮췄었는데, 실제 원인은 접근 목표
    지점의 오차/충돌이었음을 확인해서 원래 값으로 복원했다.)

    Returns:
        생성된(또는 기존의) Surface Gripper 프림 경로.
    """
    fingertip_prim = stage.GetPrimAtPath(fingertip_prim_path)
    if not fingertip_prim.IsValid():
        raise ValueError(f"fingertip prim이 유효하지 않습니다: {fingertip_prim_path}")

    if gripper_prim_path is None:
        gripper_prim_path = f"{fingertip_prim_path}/mop_surface_gripper"
    if joints_scope_path is None:
        joints_scope_path = f"{fingertip_prim_path}/mop_surface_gripper_joints"
    if body1_path is None:
        parent_prim = fingertip_prim.GetParent()
        body1_path = str(parent_prim.GetPath()) if parent_prim and parent_prim.IsValid() else fingertip_prim_path

    existing = stage.GetPrimAtPath(gripper_prim_path)
    if existing.IsValid():
        logger.info(f"[SurfaceGripper] 이미 존재하는 프림을 재사용합니다: {gripper_prim_path}")
        return gripper_prim_path

    UsdGeom.Scope.Define(stage, joints_scope_path)
    joint_path = f"{joints_scope_path}/mop_attachment_joint"
    create_attachment_point_joint(
        stage,
        joint_path=joint_path,
        body0_path=fingertip_prim_path,
        body1_path=body1_path,
        local_pos0=local_pos0,
        forward_axis=forward_axis,
        clearance_offset=clearance_offset,
        grip_travel=grip_travel,
        grip_drive_stiffness=grip_drive_stiffness,
        grip_drive_damping=grip_drive_damping,
        rot_drive_stiffness=rot_drive_stiffness,
        rot_z_drive_stiffness=rot_z_drive_stiffness,
    )

    create_surface_gripper(
        stage,
        gripper_prim_path=gripper_prim_path,
        attachment_joint_paths=[joint_path],
        max_grip_distance=max_grip_distance,
    )

    return gripper_prim_path
