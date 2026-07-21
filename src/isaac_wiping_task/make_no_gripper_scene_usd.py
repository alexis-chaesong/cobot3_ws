"""
make_no_gripper_scene_usd.py
-----------------------------
1b_tool_changer_demo_no_gripper.py가 매 실행마다 코드로 조립하던 "초기 씬"
(그리퍼 없는 M0609 팔 + 관절 드라이브 게인 + 바닥 + 걸레 placeholder)을
한 번만 구워서 독립 USD 파일(m0609_no_gripper_scene.usd)로 저장하는 스크립트.

동작(IK로 움직이기/파지/흔들기 등)은 이 USD에 들어가지 않는다 — 이 파일은
어디까지나 "씬 구성"(정적 배치 + 물리 프로퍼티) 스냅샷이고, 실제 동작은
여전히 1b_tool_changer_demo_no_gripper.py의 RMPflow/Surface Gripper 루프가
담당한다. 이 스크립트는 그 스크립트의 main() 앞부분(씬 조립 로직)만 떼어내어
1회 실행 후 결과를 디스크에 저장하는 용도다.

USD_PATH(그리퍼 없는 M0609 팔 자산)는 이 파일에 통째로 복사되지 않고
subLayer로만 참조된다 — 원본 로봇 자산을 중복 저장하지 않기 위함이며,
1b_tool_changer_demo_no_gripper.py의 `stage.GetRootLayer().subLayerPaths.append(...)`
와 동일한 방식이다.

실행 방법:
  /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh make_no_gripper_scene_usd.py
"""

from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

_THIS_DIR = Path(__file__).resolve().parent
_WS_ROOT = _THIS_DIR.parents[1]  # cobot3_ws
M0609_DIR = _WS_ROOT / "isaacpjt" / "M0609"

# 그리퍼가 없는 순수 M0609 팔 자산 (1b_tool_changer_demo_no_gripper.py와 동일).
ROBOT_USD_PATH = str(_WS_ROOT / "src" / "doosan-robot2" / "urdf" / "m0609_isaac_sim" / "m0609_isaac_sim.usd")  # src/doosan-robot2로 이동됨
ROBOT_PRIM_PATH = "/m0609"

OUTPUT_USD_PATH = str(_THIS_DIR / "m0609_no_gripper_scene.usd")

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8

# 1b_tool_changer_demo_no_gripper.py와 동일한 값 (거치대 선반 대신 바닥에 걸레 배치).
STAND_POSITION = np.array([0.5, 0.0, 0.0])


def set_drive_gains(stage, root_path: str) -> None:
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


def create_placeholder_mop(stage, world) -> str:
    mop_pad_path = "/World/Mop/pad"
    pad_half_height = 0.01
    pad_position = STAND_POSITION + np.array([0.0, 0.0, pad_half_height])
    world.scene.add(
        DynamicCuboid(
            prim_path=mop_pad_path,
            name="mop_pad",
            position=pad_position,
            scale=np.array([0.15, 0.25, pad_half_height * 2.0]),
            color=np.array([0.6, 0.6, 0.9]),
            mass=0.2,
        )
    )
    handle_path = f"{mop_pad_path}/handle"
    handle_xform = UsdGeom.Xform.Define(stage, handle_path)
    handle_xform.AddTranslateOp().Set((0.0, 0.0, pad_half_height))
    return handle_path


def main():
    stage = omni.usd.get_context().get_stage()
    stage.GetRootLayer().subLayerPaths.append(ROBOT_USD_PATH)
    for _ in range(30):
        simulation_app.update()

    world = World()
    world.scene.add_default_ground_plane()

    set_drive_gains(stage, ROBOT_PRIM_PATH)
    create_placeholder_mop(stage, world)

    for _ in range(5):
        simulation_app.update()

    stage.GetRootLayer().Export(OUTPUT_USD_PATH)
    print(f"[INFO] 씬을 저장했습니다: {OUTPUT_USD_PATH}")


if __name__ == "__main__":
    main()
    simulation_app.close()
