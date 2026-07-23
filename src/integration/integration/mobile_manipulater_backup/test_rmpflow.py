"""
test_rmpflow.py
---------------
Isaac Sim Script Editor용 RMPflow 테스트 스크립트.

사용법:
  1. Isaac Sim에서 mobile_manipulator_v2.usd 씬 열기
  2. Play 버튼(▶) 누르기
  3. Window → Script Editor 열기
  4. 이 스크립트 전체 붙여넣기 후 Run 클릭
  5. 로봇 팔이 순서대로 각 목표 위치로 이동하는지 확인
"""

import sys
import numpy as np
import omni.usd
import omni.timeline
import omni.kit.app

# ── rmpflow 폴더 경로 추가 ──
RMPFLOW_DIR = "/home/rokey/cobot3_ws/isaacpjt/M0609/rmpflow"
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

from isaacsim.core.prims import SingleArticulation
from m0609_rmpflow_controller import RMPFlowController

# ────────────────────────────
# 설정값
# ────────────────────────────
ARM_PRIM_PATH     = "/World/m0609_with_gripper"
PHYSICS_DT        = 1.0 / 60.0
STEPS_PER_TARGET  = 240   # 각 목표 위치 유지 스텝 수 (240 = 약 4초)

# 이동할 목표 위치 목록 (월드 좌표, 단위: m)
TARGET_POSITIONS = [
    np.array([0.3,  0.0,  0.9]),   # 정면
    np.array([0.0,  0.3,  0.9]),   # 좌측
    np.array([0.0, -0.3,  0.9]),   # 우측
    np.array([0.3,  0.0,  1.1]),   # 위
]
TARGET_ORIENTATION = np.array([0.0, 1.0, 0.0, 0.0])

# ────────────────────────────
# 타임라인 확인
# ────────────────────────────
timeline = omni.timeline.get_timeline_interface()
if not timeline.is_playing():
    print("[ERROR] Play 버튼(▶)을 먼저 누른 후 스크립트를 실행하세요!")
else:
    app = omni.kit.app.get_app()

    # ── 아티큘레이션 연결 ──
    print(f"[INFO] 아티큘레이션 연결: {ARM_PRIM_PATH}")
    robot = SingleArticulation(prim_path=ARM_PRIM_PATH, name="m0609_test")
    robot.initialize()
    print(f"[INFO] 조인트 수: {robot.num_dof}")
    print(f"[INFO] 조인트 이름: {robot.dof_names}")

    # ── RMPflow 초기화 ──
    print("[INFO] RMPFlowController 초기화 중...")
    controller = RMPFlowController(
        name="rmpflow_test_ctrl",
        robot_articulation=robot,
        physics_dt=PHYSICS_DT,
    )
    controller.reset()
    print("[INFO] 초기화 완료! 이동 시작...\n")

    # ── 각 목표 위치로 순차 이동 ──
    for i, target_pos in enumerate(TARGET_POSITIONS):
        print(f"[TARGET {i+1}/{len(TARGET_POSITIONS)}] 이동 중 → {target_pos}")

        for step in range(STEPS_PER_TARGET):
            # RMPflow로 액션 계산 후 적용
            action = controller.forward(
                target_end_effector_position=target_pos,
                target_end_effector_orientation=TARGET_ORIENTATION,
            )
            robot.apply_action(action)

            # ★ 핵심: 실제로 물리 시뮬레이션 1프레임 진행
            app.update()

            # 60 스텝마다 현재 조인트 각도 출력
            if step % 60 == 0:
                joint_pos = robot.get_joint_positions()
                degs = np.degrees(joint_pos[:6])
                print(f"  step {step:3d}/{STEPS_PER_TARGET} | "
                      f"joints(deg): [{', '.join(f'{d:6.1f}' for d in degs)}]")

        print(f"  → [TARGET {i+1}] 도달 완료!")

    print("\n[INFO] 모든 테스트 완료!")
