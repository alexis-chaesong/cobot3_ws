"""
floor_wipe_motion.py
---------------------
MINIMAL FEATURE ONLY: grasp the mop, then move it back and forth (zigzag) a few
strokes just above the floor. No coverage tracking, no revisit avoidance, no
impedance/force control - just "follow a waypoint list in order".

Depends on mop_grasp_control.py (grasp poses, gripper control, mop setup) and
reuses the RMPFlowController move pattern from test_rmpflow.py / run_lift_drop_test.

Usage (Script Editor):
  1. Open mobile_manipulator_v2.usd and Press Play (>)
  2. Paste this whole file and Run
     -> calls run_mop_grasp_and_wipe_demo() at the bottom
"""

import sys
import numpy as np
import omni.usd
import omni.timeline
import omni.kit.app

# ── Script Editor에 붙여넣기로 실행할 때는 이 파일이 속한 폴더가 sys.path에
#    없을 수 있으므로, mop_grasp_control.py를 import하기 전에 직접 추가 ──
MOBILE_MANIPULATOR_DIR = "/home/rokey/cobot3_ws/src/integration/integration/mobile_manipulater"
if MOBILE_MANIPULATOR_DIR not in sys.path:
    sys.path.insert(0, MOBILE_MANIPULATOR_DIR)
import mop_grasp_control as mgc

RMPFLOW_DIR = "/home/rokey/cobot3_ws/isaacpjt/M0609/rmpflow"
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
from m0609_rmpflow_controller import RMPFlowController

from isaacsim.core.prims import SingleArticulation

# ────────────────────────────────────────────────────────────────
# 설정값
# ────────────────────────────────────────────────────────────────
WIPE_HEIGHT_ABOVE_FLOOR = 0.025   # 대걸레 "헤드"가 바닥에서 뜨는 높이 [m] (2~3cm)
# 주의: 절차적 mop은 손잡이가 길어서(0.7m) tool0는 바닥이 아니라 grasp 높이 근처에
# 있어야 헤드가 바닥을 스친다. 실제 waypoint z는 아래 run_..._demo에서
# grasp_pos_z + WIPE_HEIGHT_ABOVE_FLOOR 로 계산한다 (grasp 시점에 헤드가 바닥에
# 닿아 있으므로 grasp z가 곧 "헤드가 바닥에 닿는 tool0 높이"의 기준이 됨).
WIPE_STROKE_LENGTH = 0.15         # 스트로크 1회 이동 거리(전/후 축) [m] (리치 한계 감안해 짧게)
WIPE_LATERAL_STEP = 0.08          # 스트로크 사이 옆으로 이동하는 거리 [m]
WIPE_NUM_STROKES = 3              # 왕복 스트로크 수
WIPE_STEPS_PER_WAYPOINT = 90      # 웨이포인트 하나당 진행할 physics step 수

WIPE_FORWARD_AXIS = np.array([1.0, 0.0, 0.0])   # 로봇 앞쪽(월드 +X 가정)
WIPE_LATERAL_AXIS = np.array([0.0, 1.0, 0.0])   # 옆 방향(월드 +Y 가정)
WIPE_ORIENTATION_WXYZ = np.array([0.0, 1.0, 0.0, 0.0])  # top-down, compute_mop_grasp_poses와 동일

LIFT_AFTER_GRASP_HEIGHT = 0.05    # 파지 후 바닥과의 간섭을 피하기 위해 먼저 살짝 들어올리는 높이 [m]
LIFT_AFTER_GRASP_STEPS = 200

# diagnose_grasp.py로 확인: RMPFlowController가 90 step(1.5초)로는 수렴이 덜 끝난
# 상태였고(어떤 목표는 300 step까지도 계속 줄어듦), 300 step 정도로 늘려야 대부분의
# 목표에서 안정적으로 수렴한다. (단, base_link 바로 아래처럼 자체 충돌회피 영역에
# 너무 가까운 목표는 step을 늘려도 수렴이 ~0.2m에서 멈추는 경우가 있었음 - 이 경우는
# MOP_SPAWN_POSITION/GRASP_DROP_FROM_TOP을 조정해 목표를 base_link 몸체에서 더
# 떨어뜨려야 함)
PRE_GRASP_STEPS = 300
GRASP_APPROACH_STEPS = 300

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8
# ────────────────────────────────────────────────────────────────


def generate_floor_wipe_waypoints(
    start_xy,
    height_above_floor: float = WIPE_HEIGHT_ABOVE_FLOOR,
    stroke_length: float = WIPE_STROKE_LENGTH,
    lateral_step: float = WIPE_LATERAL_STEP,
    num_strokes: int = WIPE_NUM_STROKES,
    forward_axis=WIPE_FORWARD_AXIS,
    lateral_axis=WIPE_LATERAL_AXIS,
):
    """
    start_xy(대걸레 헤드의 현재 x,y)를 기준으로, 로봇 앞쪽 좁은 영역을 지그재그로
    왕복하는 3D 웨이포인트 리스트를 만든다. 각 스트로크는 forward_axis 방향으로
    stroke_length만큼 전진/후진하고, 스트로크가 끝날 때마다 lateral_axis 방향으로
    lateral_step만큼 이동한다. 높이는 height_above_floor로 전 구간 고정.
    """
    start_xy = np.array(start_xy, dtype=float)
    waypoints = []

    current_xy = start_xy.copy()
    going_forward = True
    for stroke in range(num_strokes):
        direction = forward_axis if going_forward else -forward_axis
        end_xy = current_xy + direction[:2] * stroke_length

        start_pt = np.array([current_xy[0], current_xy[1], height_above_floor])
        end_pt = np.array([end_xy[0], end_xy[1], height_above_floor])
        waypoints.append(start_pt)
        waypoints.append(end_pt)

        current_xy = end_xy + lateral_axis[:2] * lateral_step
        going_forward = not going_forward

    print(f"[INFO] generated {len(waypoints)} wipe waypoints ({num_strokes} strokes)")
    for i, wp in enumerate(waypoints):
        print(f"  waypoint[{i}] = {wp}")
    return waypoints


def follow_waypoints(
    arm: SingleArticulation,
    controller: RMPFlowController,
    waypoints,
    orientation_wxyz=WIPE_ORIENTATION_WXYZ,
    steps_per_waypoint: int = WIPE_STEPS_PER_WAYPOINT,
):
    """RMPFlowController로 waypoints를 순서대로 따라간다. 각 웨이포인트당 steps_per_waypoint 만큼 진행."""
    app = omni.kit.app.get_app()

    for i, target_pos in enumerate(waypoints):
        print(f"[WIPE] waypoint {i + 1}/{len(waypoints)} -> {target_pos}")
        for _ in range(steps_per_waypoint):
            action = controller.forward(
                target_end_effector_position=np.array(target_pos),
                target_end_effector_orientation=np.array(orientation_wxyz),
            )
            arm.apply_action(action)
            app.update()

    print("[WIPE] all waypoints reached")


# ==================================================================
# 전체 시퀀스 진입 함수
# ==================================================================
def run_mop_grasp_and_wipe_demo():
    """
    pre-grasp 이동 -> grasp 이동 -> 그리퍼 닫기 -> confirm_grasp_by_effort ->
    짧게 들어올리기 -> 바닥 와이핑 웨이포인트 순회, 순서로 전체를 실행한다.

    MINIMAL FEATURE ONLY (see module docstring): coverage/revisit/impedance 없음.
    """
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        print("[ERROR] Press Play before running this!")
        return False

    app = omni.kit.app.get_app()

    print("[STEP 0] setting up mop in scene (no-op if already present)")
    mgc.setup_mop_in_scene()
    mgc.bind_high_friction_grip_material()

    print("[STEP 1] connecting to arm articulation:", mgc.ARM_PRIM_PATH)
    arm = SingleArticulation(prim_path=mgc.ARM_PRIM_PATH, name="m0609_wipe_demo")
    arm.initialize()
    mgc.apply_arm_drive_settings(
        arm_prim_path=mgc.ARM_PRIM_PATH,
        stiffness=DRIVE_STIFFNESS,
        damping=DRIVE_DAMPING,
        max_force=DRIVE_MAX_FORCE,
    )

    controller = RMPFlowController(
        name="rmpflow_wipe_demo_ctrl",
        robot_articulation=arm,
        physics_dt=1.0 / 60.0,
    )
    controller.reset()
    mgc.sync_rmpflow_base_pose(controller)

    print("[STEP 2] computing pre-grasp / grasp poses")
    poses = mgc.compute_mop_grasp_poses()

    print("[STEP 3] moving to pre-grasp pose")
    for _ in range(PRE_GRASP_STEPS):
        action = controller.forward(
            target_end_effector_position=poses["pre_grasp_pos"],
            target_end_effector_orientation=poses["orientation_wxyz"],
        )
        arm.apply_action(action)
        app.update()

    print("[STEP 4] moving to grasp pose")
    for _ in range(GRASP_APPROACH_STEPS):
        action = controller.forward(
            target_end_effector_position=poses["grasp_pos"],
            target_end_effector_orientation=poses["orientation_wxyz"],
        )
        arm.apply_action(action)
        app.update()

    print("[STEP 5] closing gripper")
    mgc.close_gripper_to_width(arm, mgc.GRASP_TARGET_WIDTH)
    for _ in range(mgc.SETTLE_STEPS):
        app.update()

    grasp_ok = mgc.confirm_grasp_by_effort(arm)
    if not grasp_ok:
        print("[WARNING] confirm_grasp_by_effort reports FAIL, continuing anyway (minimal demo)")

    print("[STEP 6] lifting mop slightly")
    lift_target = poses["grasp_pos"] + mgc.HANDLE_AXIS_WORLD * LIFT_AFTER_GRASP_HEIGHT
    for _ in range(LIFT_AFTER_GRASP_STEPS):
        action = controller.forward(
            target_end_effector_position=lift_target,
            target_end_effector_orientation=poses["orientation_wxyz"],
        )
        arm.apply_action(action)
        app.update()

    print("[STEP 7] generating floor wipe waypoints")
    # grasp 시점에 mop 헤드가 바닥에 닿아 있으므로 grasp z가 "헤드=바닥" 기준 높이.
    # 거기에 헤드 클리어런스만 더해 tool0 순회 높이로 사용한다.
    wipe_tool0_z = poses["grasp_pos"][2] + WIPE_HEIGHT_ABOVE_FLOOR
    waypoints = generate_floor_wipe_waypoints(
        start_xy=lift_target[:2],
        height_above_floor=wipe_tool0_z,
    )

    print("[STEP 8] wiping floor")
    follow_waypoints(arm, controller, waypoints)

    print("[DONE] run_mop_grasp_and_wipe_demo finished")
    return True


if __name__ == "__main__":
    run_mop_grasp_and_wipe_demo()
