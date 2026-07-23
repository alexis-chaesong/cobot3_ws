
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
enable_extension("omni.physx.ui")               # Property 패널의 Physics 섹션(+Add 포함) 표시
enable_extension("omni.kit.property.physx")     # 위 확장이 의존하는 Physics 프로퍼티 위젯
enable_extension("omni.kit.window.script_editor")  # Window > Script Editor - 실행 중에 직접 천을 추가할 때 사용
simulation_app.update()

from pathlib import Path
import sys
import time

import numpy as np
import omni.usd
import omni.kit.app
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

em = omni.kit.app.get_app().get_extension_manager()
print(f"  [EXT] omni.physx.ui enabled = {em.is_extension_enabled('omni.physx.ui')}")
print(f"  [EXT] omni.kit.property.physx enabled = {em.is_extension_enabled('omni.kit.property.physx')}")
print(f"  [EXT] omni.kit.window.script_editor enabled = {em.is_extension_enabled('omni.kit.window.script_editor')}")

_THIS_DIR = Path(__file__).resolve().parent
_WS_SRC_DIR = (_THIS_DIR / "../../src").resolve()  # cobot3_ws/src

# rmpflow 인프라 폴더 경로 등록 (인프라 파일 내부 import가 그대로 동작)
RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

# ClothAsset 패키지 경로 등록. 이 스크립트는 천을 직접 만들지 않고, 사용자가 Script Editor에서
# create_cloth_asset.py를 열어 직접 실행하는 것을 전제로 한다 - 그때 `import create_cloth_asset`이
# 바로 되도록 경로만 미리 등록해 둔다.
CLOTH_ASSET_DIR = str(_WS_SRC_DIR / "ClothAsset/scripts")
if CLOTH_ASSET_DIR not in sys.path:
    sys.path.insert(0, CLOTH_ASSET_DIR)

from m0609_pick_place_controller import PickPlaceController

# ╔══════════════════════════════════════════════════════════════╗
# ║  A. Task 파라미터 (4_pick_place.py / 6_pick_place_color.py와 동일) ║
# ╚══════════════════════════════════════════════════════════════╝
USD_PATH        = str(_WS_SRC_DIR / "assets/robots/Collected_m0609_camera/m0609_camera.usd")
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"
GRIPPER_JOINTS  = ["finger_joint", "right_inner_knuckle_joint"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

GRIPPER_OPEN    = [0.0, 0.0]
GRIPPER_CLOSE   = [0.5, 0.5]
GRIPPER_DELTA   = [-0.5, -0.5]

FINGER_STATIC   = 1.8
FINGER_DYNAMIC  = 1.4

# ╔══════════════════════════════════════════════════════════════╗
# ║  B. Controller 파라미터                                        ║
# ╚══════════════════════════════════════════════════════════════╝
M0609_URDF_PATH           = str(_WS_SRC_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH    = str(_THIS_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(_THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml")

GOAL_POS  = np.array([0.55, -0.35, 0.0])
EE_OFFSET = np.array([0.0, 0.0, 0.2])

# 10단계 타이밍 (4_pick_place.py와 동일)
EVENTS_DT = [
    0.008,   # 0. 접근 이동
    0.005,   # 1. 하강
    0.02,    # 2. 그리퍼 닫기 대기
    0.1,     # 3. 그리퍼 닫힘 유지
    0.0025,  # 4. 들어올리기
    0.01,    # 5. Place 위치로 이동
    0.0025,  # 6. 하강
    1,       # 7. 그리퍼 열기 대기
    0.008,   # 8. 상승
    0.08,    # 9. 복귀
]


# ============================================================
# 유틸
# ============================================================
def find_prim_path_by_name(root_path: str, name: str):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def initialize_robot(robot, world):
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    robot.set_joint_positions(np.zeros(robot.num_dof))


def find_cloth_mesh_prim():
    """씬 전체를 훑어 PhysxParticleClothAPI가 붙은 UsdGeom.Mesh를 찾는다.
    천을 이 스크립트가 만들지 않고 사용자가 Script Editor 등에서 별도로 추가하기
    때문에, 경로를 미리 알 수 없어 매번 스캔해서 '감지'하는 방식으로 동작한다."""
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh) and prim.HasAPI(PhysxSchema.PhysxParticleClothAPI):
            return prim
    return None


def get_cloth_world_centroid(mesh_prim):
    """천 메시의 현재 시뮬레이션 포인트를 월드 좌표로 변환해 중심점을 구한다.
    큐브처럼 get_world_pose() 한 번으로 위치를 알 수 없는, 매 스텝 형태가 변하는
    오브젝트이므로 이 중심점 계산이 곧 '천 감지(detect)'에 해당한다."""
    if mesh_prim is None or not mesh_prim.IsValid():
        return None
    mesh = UsdGeom.Mesh(mesh_prim)
    points = mesh.GetPointsAttr().Get()
    if not points:
        return None
    world_matrix = UsdGeom.XformCache().GetLocalToWorldTransform(mesh_prim)
    world_points = np.array([world_matrix.Transform(p) for p in points])
    return np.mean(world_points, axis=0)


# ============================================================
# Task — M0609ClothTask
# ============================================================
class M0609ClothTask(BaseTask):

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._task_achieved = False
        self._cloth_mesh_prim = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._discover_links()
        self._setup_physics()
        self._register_robot(scene)
        self._create_scene(scene)
        print("\n  [완료] 씬 구성 성공!\n")

    def _load_usd(self):
        print("\n" + "=" * 60)
        print("[1.LOAD] USD 로드")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()
        print(f"  [OK] {USD_PATH}")

    def _discover_links(self):
        print("\n" + "=" * 60)
        print("[2.DISCOVER] 링크 경로 탐색")
        print("=" * 60)
        self._ee_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if self._ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found")
        print(f"  EE ({EE_LINK_NAME}) = {self._ee_path}")
        for jn in GRIPPER_JOINTS:
            print(f"  {jn:<35} = {find_prim_path_by_name(ROBOT_PRIM_PATH, jn)}")

    def _setup_physics(self):
        print("\n" + "=" * 60)
        print("[3.PHYSICS] 물리 설정")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()

        drive_count = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            for dt in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, dt)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    drive_count += 1
        print(f"  [OK] drive updated: {drive_count}")

    def _register_robot(self, scene):
        print("\n" + "=" * 60)
        print("[4.REGISTER] 로봇 등록")
        print("=" * 60)
        gripper = ParallelGripper(
            end_effector_prim_path=self._ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array(GRIPPER_OPEN),
            joint_closed_positions=np.array(GRIPPER_CLOSE),
            action_deltas=np.array(GRIPPER_DELTA),
        )
        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
                end_effector_prim_path=self._ee_path,
                gripper=gripper,
            )
        )
        print(f"  [OK] SingleManipulator: {ROBOT_PRIM_PATH}")

    def _create_scene(self, scene):
        print("\n" + "=" * 60)
        print("[5.SCENE] 작업 환경 구성 (목표 마커 - 천은 사용자가 직접 추가)")
        print("=" * 60)
        scene.add(
            VisualCuboid(
                prim_path="/World/goal_marker",
                name="goal_marker",
                position=GOAL_POS,
                scale=np.array([0.06, 0.06, 0.001]),
                color=np.array([0.0, 1.0, 0.0]),
            )
        )
        print(f"  [OK] goal @ {GOAL_POS}")

        finger_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/finger_material",
            static_friction=FINGER_STATIC,
            dynamic_friction=FINGER_DYNAMIC,
            restitution=0.0,
        )
        for link_name in ["left_inner_finger", "right_inner_finger"]:
            link_path = find_prim_path_by_name(ROBOT_PRIM_PATH, link_name)
            if link_path:
                SingleGeometryPrim(
                    prim_path=link_path,
                    name=f"{link_name}_geom",
                ).apply_physics_material(finger_material)
                print(f"  [OK] friction: {link_path}")

        print("  [천] 이 스크립트는 천을 생성하지 않습니다 - Script Editor에서")
        print("       create_cloth_asset.py를 열어 직접 실행해 천을 추가해주세요.")
        print("       (PhysxParticleClothAPI가 붙은 Mesh를 자동으로 찾아 집습니다)")

    def get_observations(self):
        if self._cloth_mesh_prim is None or not self._cloth_mesh_prim.IsValid():
            self._cloth_mesh_prim = find_cloth_mesh_prim()
        cloth_pos = get_cloth_world_centroid(self._cloth_mesh_prim)
        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),
            },
            "cloth": {
                "position": cloth_pos,  # 아직 천이 없으면 None
                "goal_position": GOAL_POS,
            },
        }

    def pre_step(self, control_index, simulation_time):
        cloth_pos = get_cloth_world_centroid(self._cloth_mesh_prim)
        if cloth_pos is not None and not self._task_achieved and np.mean(np.abs(GOAL_POS - cloth_pos)) < 0.05:
            self._task_achieved = True

    def post_reset(self):
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )
        # 천은 사용자가 직접 관리하므로 여기서 지우거나 다시 만들지 않고,
        # 다음 get_observations() 호출 때 다시 스캔하도록 캐시만 비운다.
        self._cloth_mesh_prim = None
        self._task_achieved = False


# ╔══════════════════════════════════════════════════════════════╗
# ║  C. 메인 — Controller 생성 및 실행                              ║
# ╚══════════════════════════════════════════════════════════════╝

def main():
    my_world = World(stage_units_in_meters=1.0)
    task = M0609ClothTask(name="m0609_cloth_task")
    my_world.add_task(task)
    my_world.reset()

    robot = my_world.scene.get_object("m0609_robot")
    initialize_robot(robot, my_world)

    # 홈 포지션 안정화 대기
    for _ in range(30):
        my_world.step(render=True)

    print("\n" + "=" * 60)
    print("[C-1] PickPlaceController 생성")
    print("=" * 60)
    print(f"  URDF        = {M0609_URDF_PATH}")
    print(f"  description = {M0609_DESCRIPTION_PATH}")
    print(f"  rmpflow     = {M0609_RMPFLOW_CONFIG_PATH}")
    print(f"  events_dt   = {EVENTS_DT}")
    print(f"  EE frame    = {EE_LINK_NAME}")

    controller = PickPlaceController(
        name="m0609_pick_place_controller",
        gripper=robot.gripper,
        robot_articulation=robot,
        end_effector_initial_height=0.30,
        events_dt=EVENTS_DT,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )
    print("  [OK] Controller 생성 완료")

    ee_pos, _ = robot.end_effector.get_world_pose()
    print(f"\n  EE 초기 위치 = {ee_pos}")
    print(f"  목표 위치    = {GOAL_POS}")
    print("\n[대기] Window > Script Editor 에서 천을 추가해주세요. 감지되면 자동으로 Pick & Place를 시작합니다.\n")

    was_playing = False
    task_done = False
    cloth_waiting_announced = False

    while simulation_app.is_running():
        my_world.step(render=True)
        time.sleep(0.01)
        is_playing = my_world.is_playing()

        # Play 시작 감지 → 리셋
        if is_playing and not was_playing:
            my_world.reset()
            initialize_robot(robot, my_world)
            controller.reset()
            task_done = False
            cloth_waiting_announced = False

        # 매 스텝 제어
        if is_playing and not task_done:
            # (1) 관측 데이터 수집 — 천 감지 (없으면 None)
            obs = task.get_observations()
            cloth_position  = obs["cloth"]["position"]
            current_joints  = obs["m0609_robot"]["joint_positions"]

            if cloth_position is None:
                if not cloth_waiting_announced:
                    print("  [대기] 씬에서 천을 아직 찾지 못했습니다. Script Editor로 추가해주세요...")
                    cloth_waiting_announced = True
            else:
                cloth_waiting_announced = False

                # (2) Controller에 목표 전달 → 관절 명령 생성
                actions = controller.forward(
                    picking_position=cloth_position,
                    placing_position=GOAL_POS,
                    current_joint_positions=current_joints,
                    end_effector_offset=EE_OFFSET,
                )

                # (3) 로봇에 적용
                robot.apply_action(actions)

                # (4) 완료 확인
                if controller.is_done():
                    print("[완료] 천 Pick & Place 성공!")
                    task_done = True
                    my_world.pause()

                # 디버그 출력
                event = controller.get_current_event()
                ee_pos, _ = robot.end_effector.get_world_pose()
                print(f"  [event={event}] cloth={cloth_position}  ee_z={ee_pos[2]:.4f}")

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()
