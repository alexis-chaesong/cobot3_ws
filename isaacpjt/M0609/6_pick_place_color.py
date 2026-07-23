
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# ── ROS2 관련 모듈은 반드시 enable_extension("isaacsim.ros2.bridge") 아래에서 import ──
# (확장이 활성화되기 전에는 rclpy / ROS2 OmniGraph 노드 타입을 사용할 수 없다)
import omni.graph.core as og
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from pathlib import Path
import random
import sys
import time

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent

# rmpflow 인프라 폴더 경로 등록 (인프라 파일 내부 import가 그대로 동작)
RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

from m0609_pick_place_controller import PickPlaceController

# ╔══════════════════════════════════════════════════════════════╗
# ║  A. Task 파라미터 (이전 장과 동일)                              ║
# ╚══════════════════════════════════════════════════════════════╝
USD_PATH        = str(_THIS_DIR / "Collected_m0609_camera/m0609_camera.usd")
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
CUBE_STATIC     = 1.2
CUBE_DYNAMIC    = 1.0


# ╔══════════════════════════════════════════════════════════════╗
# ║  B. Controller 파라미터 (이전 장과 동일)                        ║
# ╚══════════════════════════════════════════════════════════════╝

# ── B-1. 인프라 파일 경로 (RMPFlow가 참조) ────────────────────
M0609_URDF_PATH           = str(_THIS_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH    = str(_THIS_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(_THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml")

# ── B-2. Pick & Place 동작 파라미터 ───────────────────────────
MARKER_POS = {
    "blue":  np.array([0.55, -0.35, 0.0]),
    "green": np.array([0.55,  0.35, 0.0]),
}

# ★ pick 영역: 두 마커의 중간 지점을 "중심"으로 삼고, 매 라운드 이 중심에서
#   x/y를 ±CUBE_INIT_RANGE_XY 범위 내에서 무작위로 흔들어 실제 pick 위치를 정한다
#   (기존에는 CUBE_INIT_POS 라는 고정 좌표 하나였음 — 매번 정확히 같은 자리에
#   떨어져서 다양성이 없었음).
CUBE_INIT_CENTER = np.array([
    (MARKER_POS["blue"][0] + MARKER_POS["green"][0]) / 2,
    (MARKER_POS["blue"][1] + MARKER_POS["green"][1]) / 2,
    0.0515 / 2.0
])

# 중심에서 ±8cm 범위. 두 마커까지의 y방향 거리(0.35m)에 비해 충분히 작아
# 마커 위치와 절대 겹치지 않고(최악의 경우도 0.27m 여유), 로봇 도달범위 안에서
# 안전하게 움직일 수 있는 보수적인 값이다. 필요하면 이 값만 조정하면 된다.
CUBE_INIT_RANGE_XY = 0.08

EE_OFFSET     = np.array([0.0, 0.0, 0.2])               # 접근 높이

# ★ 비활성(선택되지 않은) 큐브를 치워두는 "파킹" 위치.
#   로봇 작업공간(대략 x<0.6, y 는 -0.4~0.6, z<0.4)에서 완전히 벗어난 좌표라
#   시각적으로 안 보이는 것과 별개로, 혹시라도 물리 비활성화가 실패하더라도
#   그리퍼/활성 큐브와 물리적으로 겹칠 일이 없도록 하는 이중 안전장치다.
#   비활성 큐브는 항상 1개뿐이므로 색상별로 굳이 나눌 필요는 없지만,
#   디버깅 시 어떤 색이 파킹됐는지 로그로 구분하기 쉽도록 색상별 좌표를 둔다.
CUBE_PARK_POS = {
    "blue":  np.array([-5.0, -5.0, -1.0]),
    "green": np.array([-5.0,  5.0, -1.0]),
}

CUBE_COLOR_RGB = {
    "blue":  np.array([0.0, 0.0, 1.0]),
    "green": np.array([0.0, 1.0, 0.0]),
}

# color_id 매핑: 외부 색상 감지 노드가 /color_id 로 보내는 값 (std_msgs/Int32)
COLOR_ID_TO_NAME = {1: "blue", 2: "green"}

# ── B-3. 10단계 타이밍 (작을수록 빠름) ────────────────────────
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

# ── B-4. Wrist Camera → /rgb 퍼블리셔 설정 ────────────────────
# 실제 프림 이름을 100% 확신할 수 없으므로(USD 참조/페이로드로 조립된 RealSense 자산),
# UsdGeom.Camera 타입으로 스캔한 뒤 이름에 아래 힌트가 들어간 것을 우선 선택한다.
# 팀원 확인 결과 실제 카메라 프림 이름이 다르면 이 힌트만 바꾸면 된다.
CAMERA_NAME_HINT = "rgb"
RGB_TOPIC_NAME   = "/rgb"
CAMERA_FRAME_ID  = "wrist_camera"
CAMERA_RESOLUTION = (640, 480)

# ── B-5. /color_id 하드코딩 테스트 플래그 ─────────────────────
# 색상 감지 노드(다른 PC)가 아직 안 붙어 있어도 동작을 확인할 수 있도록,
# True 로 켜면 실제 /color_id 구독값 대신 FORCE_COLOR_ID_VALUE 를 강제로 사용한다.
# 실제 통합 테스트 시에는 반드시 False 로 되돌릴 것.
FORCE_COLOR_ID_FOR_TEST = False
FORCE_COLOR_ID_VALUE    = 1   # 1=파랑, 2=초록


# ============================================================
# 유틸 (이전 장과 동일)
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


def find_camera_prim_path(root_path: str, name_hint: str = "rgb"):
    """로봇 프림 트리 안에서 UsdGeom.Camera 타입 프림을 찾는다.
    RealSense 등 외부 자산은 참조/페이로드로 조립되므로 정확한 프림 이름을 미리
    알기 어렵다. 그래서 이름 매칭 대신 '타입'으로 찾고, 후보가 여러 개면
    name_hint(기본 "rgb")가 경로에 포함된 것을 우선한다."""
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None

    candidates = [
        str(prim.GetPath())
        for prim in Usd.PrimRange(root_prim)
        if prim.IsA(UsdGeom.Camera)
    ]
    if not candidates:
        return None

    print(f"  [CAMERA] 발견된 Camera 프림 후보: {candidates}")
    for path in candidates:
        if name_hint.lower() in path.lower():
            return path
    return candidates[0]


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


def setup_ros2_camera_publisher(camera_prim_path, topic_name, frame_id, resolution):
    """isaacsim.ros2.bridge 의 ROS2 Camera Helper OmniGraph를 코드로 구성해
    Wrist Camera 이미지를 sensor_msgs/Image 로 topic_name(/rgb)에 발행한다.
    OmniGraph 에디터에서 수동으로 만드는 대신 og.Controller.edit()로 동일한
    노드 그래프(OnTick → CreateRenderProduct → ROS2CameraHelper)를 생성한다."""
    keys = og.Controller.Keys
    graph_path = "/World/ROS2CameraGraph"
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
    return graph


class ColorIdSubscriber(Node):
    """/color_id (std_msgs/Int32) 를 구독하는 경량 노드.
    콜백은 값을 인스턴스 변수에 저장만 하고, 실제 사용은 메인 루프에서
    rclpy.spin_once()로 폴링하는 방식으로 처리한다 (시뮬레이션 스텝을 막지 않기 위함)."""

    def __init__(self):
        super().__init__("color_id_subscriber")
        self.latest_color_id = None  # 아직 한 번도 못 받았으면 None
        self.create_subscription(Int32, "/color_id", self._on_color_id, 10)

    def _on_color_id(self, msg):
        # ★ 콜백이 실제로 호출되는지 자체를 눈으로 확인하기 위한 로그.
        #   이 로그가 안 찍히면 "구독이 안 되고 있는" 상황(도메인ID 불일치, 퍼블리셔
        #   미실행 등)이고, 이 로그는 찍히는데 메인 루프의 color_id가 계속 None이면
        #   저장/전달 쪽 로직 버그로 원인을 좁힐 수 있다.
        print(f"  [/color_id 수신] value={msg.data}")
        self.latest_color_id = int(msg.data)


# ============================================================
# Task — M0609Task (파랑/초록 2-큐브 버전으로 확장)
# ============================================================
class M0609Task(BaseTask):

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._task_achieved = False
        # 아직 reset이 한 번도 안 됐을 때 get_observations가 호출돼도
        # 안전하도록 기본값을 미리 잡아둔다.
        self._active_cube_color = "blue"
        self._active_cube_name = "blue_cube"

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
        print("[5.SCENE] 작업 환경 구성 (파랑/초록 큐브 2개)")
        print("=" * 60)
        cube_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/cube_material",
            static_friction=CUBE_STATIC,
            dynamic_friction=CUBE_DYNAMIC,
            restitution=0.0,
        )

        # ★ 큐브 1개 → 파랑/초록 2개로 확장. 생성 시점에는 둘 다 "활성"이 아직 정해지지
        #   않았으므로, 일단 파킹 위치에 만들어두고 즉시 시각적으로 숨긴다.
        #   물리(rigid body) on/off는 physics sim view가 준비된 뒤(post_reset 시점)에만
        #   안전하게 호출할 수 있어서 여기서는 건드리지 않는다 — 실제 활성/비활성
        #   전환(가시성 + 물리 토글)은 _randomize_active_cube()가 전담한다.
        self._cubes = {}
        for color in ("blue", "green"):
            cube = scene.add(
                DynamicCuboid(
                    prim_path=f"/World/{color}_cube",
                    name=f"{color}_cube",
                    position=CUBE_PARK_POS[color],
                    scale=np.array([0.05, 0.05, 0.05]),
                    color=CUBE_COLOR_RGB[color],
                    mass=0.05,
                    physics_material=cube_material,
                )
            )
            cube.set_visibility(visible=False)
            self._cubes[color] = cube
            print(f"  [OK] {color}_cube 생성 @ park {CUBE_PARK_POS[color]} (초기 상태: 숨김)")

        # ★ goal_marker 1개 → blue_marker/green_marker 2개로 확장 (색상별 목표 지점)
        self._markers = {}
        for color in ("blue", "green"):
            marker = scene.add(
                VisualCuboid(
                    prim_path=f"/World/{color}_marker",
                    name=f"{color}_marker",
                    position=MARKER_POS[color],
                    scale=np.array([0.06, 0.06, 0.001]),
                    color=CUBE_COLOR_RGB[color],
                )
            )
            self._markers[color] = marker
            print(f"  [OK] {color}_marker @ {MARKER_POS[color]}")

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

    def _randomize_active_cube(self):
        """파랑/초록 큐브 중 하나를 무작위로 골라, pick 영역 중심(CUBE_INIT_CENTER)
        근처의 무작위 위치에 배치하고, 나머지 하나는 "존재하지 않는 것처럼" 완전히
        숨긴다. reset마다(post_reset에서) 호출되어 매 시나리오 시작 시 색상과
        위치 모두에 랜덤성을 부여한다.

        비활성 큐브에 대해 시각적 숨김(visibility) 하나만 적용하지 않고,
        아래 두 가지를 함께 적용한다:
          1) disable_rigid_body_physics() — 중력/충돌 시뮬레이션에서 완전히 제외.
             그렇지 않으면 안 보이는 큐브가 여전히 물리적으로는 존재해서, 그리퍼나
             활성 큐브와 "보이지 않는 충돌"을 일으킬 수 있다(예: 낙하하다가 활성
             큐브를 밀쳐내는 등). 시각적으로만 숨기는 것은 이 문제를 막지 못한다.
          2) CUBE_PARK_POS로 원거리 이동 — 물리가 비활성화되어도 좌표 자체는 그대로
             남아있으므로(카메라 프러스텀 컬링, 바운딩 볼륨 쿼리 등 다른 시스템이
             transform을 참조할 가능성 대비) 작업공간과 완전히 분리된 곳으로 치워
             이중으로 안전하게 만든다.
        결과적으로 "시각적 숨김 + 물리 비활성화 + 원거리 격리" 3중 처리로, 삭제하지
        않고도(다음 라운드에 다시 꺼내 쓸 수 있도록) 완전히 안 보이고 안 부딪히게 한다."""
        self._active_cube_color = random.choice(["blue", "green"])
        # ★ 진단용 로그: random.choice() 원시 결과 자체를 매 호출마다 그대로 찍는다.
        #   이 값이 매번 편향되게 나온다면 random 모듈/시드 쪽 문제, 여기까지는 정상인데
        #   화면/물리에서 여전히 파랑만 보인다면 아래 for 루프(visibility/physics 반영)
        #   쪽 문제로 원인을 좁힐 수 있다.
        print(f"  [RANDOM] random.choice 결과 = {self._active_cube_color}")
        self._active_cube_name = self._cubes[self._active_cube_color].name

        # ★ pick 위치도 매 라운드 랜덤화: CUBE_INIT_CENTER를 기준으로 x/y를 각각
        #   [-CUBE_INIT_RANGE_XY, +CUBE_INIT_RANGE_XY] 범위에서 균등 샘플링한다.
        #   z는 큐브가 바닥에 닿는 고정 높이(큐브 반높이)라 랜덤화 대상이 아니다.
        offset_x = random.uniform(-CUBE_INIT_RANGE_XY, CUBE_INIT_RANGE_XY)
        offset_y = random.uniform(-CUBE_INIT_RANGE_XY, CUBE_INIT_RANGE_XY)
        pick_pos = CUBE_INIT_CENTER + np.array([offset_x, offset_y, 0.0])
        print(f"  [RANDOM] 이번 라운드 pick 위치 = {pick_pos}")

        for color, cube in self._cubes.items():
            if color == self._active_cube_color:
                cube.enable_rigid_body_physics()
                cube.set_world_pose(position=pick_pos)
                cube.set_visibility(visible=True)
            else:
                cube.disable_rigid_body_physics()
                cube.set_world_pose(position=CUBE_PARK_POS[color])
                cube.set_visibility(visible=False)
            # 순간 이동 직후 잔여 속도가 남아있으면 물리 스텝에서 튀어나갈 수 있으므로 0으로 초기화
            cube.set_linear_velocity(np.zeros(3))
            cube.set_angular_velocity(np.zeros(3))

        print(
            f"  [RANDOM] 이번 라운드: {self._active_cube_color} 큐브만 활성화, "
            f"나머지는 숨김 처리됨 (active={self._active_cube_name})"
        )

    def get_observations(self):
        active_cube = self._cubes[self._active_cube_color]
        active_pos, _ = active_cube.get_world_pose()
        blue_marker_pos, _ = self._markers["blue"].get_world_pose()
        green_marker_pos, _ = self._markers["green"].get_world_pose()
        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),
            },
            # ★ 기존 target_cube 단일 항목 대신, 현재 pick 대상인 "활성 큐브" 정보를 반환
            "active_cube": {
                "name": self._active_cube_name,
                "color": self._active_cube_color,
                "position": active_pos,
            },
            "blue_marker": {"position": blue_marker_pos},
            "green_marker": {"position": green_marker_pos},
        }

    def pre_step(self, control_index, simulation_time):
        # ★ 완료 판정 기준: 활성 큐브 위치가 "자기 색상과 일치하는" 마커 위치에 도달했는가
        active_cube = self._cubes[self._active_cube_color]
        target_marker = self._markers[self._active_cube_color]
        cube_pos, _ = active_cube.get_world_pose()
        marker_pos, _ = target_marker.get_world_pose()
        if not self._task_achieved and np.mean(np.abs(marker_pos - cube_pos)) < 0.02:
            self._task_achieved = True

    def post_reset(self):
        # ★ 진단용 로그: Task.post_reset()이 reset/Play마다 실제로 호출되는지 확인.
        #   (World.reset()은 scene.post_reset() → task.post_reset() 순서로 호출하므로,
        #   여기서 하는 위치/가시성/물리 설정이 scene의 기본상태 복원보다 항상 나중에
        #   적용되어 최종적으로 반영된다.)
        print("  [POST_RESET] M0609Task.post_reset() 호출됨")
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )
        # ★ reset 시점마다 파랑/초록 중 하나를 랜덤하게 pick 영역으로 이동
        self._randomize_active_cube()
        self._task_achieved = False


# ╔══════════════════════════════════════════════════════════════╗
# ║  C. 메인 — Controller 생성 및 실행                              ║
# ╚══════════════════════════════════════════════════════════════╝

def main():
    # ── C-1. World + Task (이전 장과 동일) ────────────────────
    my_world = World(stage_units_in_meters=1.0)
    task = M0609Task(name="m0609_task")
    my_world.add_task(task)
    my_world.reset()

    robot = my_world.scene.get_object("m0609_robot")
    initialize_robot(robot, my_world)

    # 홈 포지션 안정화 대기
    for _ in range(30):
        my_world.step(render=True)

    # ── C-1b. ROS2 카메라 퍼블리셔 (/rgb) 셋업 ─────────────────
    # USD가 완전히 로드되고 my_world.reset()이 끝난 뒤에야 카메라 프림을 찾을 수 있으므로
    # 여기(월드 리셋 이후)에서 설정한다. enable_extension("isaacsim.ros2.bridge")은
    # 파일 최상단에서 이미 호출됨.
    print("\n" + "=" * 60)
    print("[C-1b] ROS2 Wrist Camera 퍼블리셔 설정")
    print("=" * 60)
    camera_prim_path = find_camera_prim_path(ROBOT_PRIM_PATH, name_hint=CAMERA_NAME_HINT)
    if camera_prim_path is None:
        print("  [WARN] Camera 프림을 찾지 못했습니다. /rgb 퍼블리셔를 건너뜁니다. "
              "(USD 안에 카메라가 있는지, ROBOT_PRIM_PATH 아래에 있는지 확인 필요)")
    else:
        setup_ros2_camera_publisher(
            camera_prim_path=camera_prim_path,
            topic_name=RGB_TOPIC_NAME,
            frame_id=CAMERA_FRAME_ID,
            resolution=CAMERA_RESOLUTION,
        )
        simulation_app.update()
        print(f"  [OK] {camera_prim_path} → {RGB_TOPIC_NAME}")

    # ── C-1c. ROS2 /color_id 구독자 셋업 ───────────────────────
    rclpy.init()
    color_sub_node = ColorIdSubscriber()
    print(f"  [OK] /color_id 구독 시작 (FORCE_COLOR_ID_FOR_TEST={FORCE_COLOR_ID_FOR_TEST})")

    # ── C-2. Controller 생성 (initialize 이후에만 가능) ───────
    print("\n" + "=" * 60)
    print("[C-2] PickPlaceController 생성")
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

    # ── C-3. 초기 상태 진단 ───────────────────────────────────
    ee_pos, _ = robot.end_effector.get_world_pose()
    print(f"\n  EE 초기 위치 = {ee_pos}")
    # ★ pick 위치는 이제 라운드마다 달라지므로, 여기서는 중심/범위만 참고용으로 출력한다.
    #   실제 이번 라운드 pick 좌표는 _randomize_active_cube()의 "[RANDOM] 이번 라운드
    #   pick 위치 = ..." 로그에서 확인할 것.
    print(f"  pick 영역 중심 = {CUBE_INIT_CENTER}  (±{CUBE_INIT_RANGE_XY}m 랜덤)")
    print(f"  blue marker  = {MARKER_POS['blue']}")
    print(f"  green marker = {MARKER_POS['green']}")

    # ── C-4. Controller 실행 루프 ─────────────────────────────
    print("\n[Pick & Place 시작]\n")
    was_playing = False
    task_done = False

    while simulation_app.is_running():
        my_world.step(render=True)
        time.sleep(0.01)
        is_playing = my_world.is_playing()

        # /color_id 콜백 처리 (블로킹 없이 큐에 쌓인 메시지만 즉시 소비)
        rclpy.spin_once(color_sub_node, timeout_sec=0.0)

        # Play 시작 감지 → 리셋
        if is_playing and not was_playing:
            my_world.reset()
            initialize_robot(robot, my_world)
            controller.reset()
            task_done = False

        # 매 스텝 제어
        if is_playing and not task_done:
            # (1) 관측 데이터 수집
            obs = task.get_observations()
            cube_position   = obs["active_cube"]["position"]
            active_color    = obs["active_cube"]["color"]
            current_joints  = obs["m0609_robot"]["joint_positions"]

            # (2) color_id 결정: 하드코딩 테스트 플래그가 켜져 있으면 강제값 사용,
            #     아니면 /color_id 구독으로 받은 최신 값을 사용
            if FORCE_COLOR_ID_FOR_TEST:
                color_id = FORCE_COLOR_ID_VALUE
            else:
                color_id = color_sub_node.latest_color_id

            if color_id is None:
                # /color_id 를 아직 한 번도 못 받은 상태 → 목표(place) 위치를 정할 수 없으므로
                # 이번 스텝은 로봇에 명령을 내리지 않고 대기한다 (컨트롤러 내부 상태는 그대로 유지됨).
                # ★ FORCE_COLOR_ID_FOR_TEST=False 인 상태에서 이 로그가 뜨는 것 자체는
                #   버그가 아니라 "아직 색상 감지 노드가 /color_id를 발행하지 않았다"는
                #   정상적인 대기 상태다. 콜백이 실제로 호출되는지는 위 ColorIdSubscriber
                #   의 "[/color_id 수신] value=..." 로그로 별도 확인할 것.
                print(
                    f"  [정상 대기] /color_id 발행 노드가 아직 연결되지 않음 "
                    f"(색상 감지 노드 실행 대기 중) — active_cube={active_color}"
                )
            else:
                place_color = COLOR_ID_TO_NAME.get(color_id)
                if place_color is None:
                    # 정의되지 않은 color_id 값(1/2 외)이 들어온 경우의 안전 폴백:
                    # 활성 큐브 자신의 색상 마커를 기본 목표로 사용한다.
                    place_color = active_color
                placing_position = MARKER_POS[place_color]

                # (3) Controller에 목표 전달 → 관절 명령 생성
                actions = controller.forward(
                    picking_position=cube_position,
                    placing_position=placing_position,
                    current_joint_positions=current_joints,
                    end_effector_offset=EE_OFFSET,
                )

                # (4) 로봇에 적용
                robot.apply_action(actions)

                # (5) 완료 확인
                if controller.is_done():
                    print("[완료] Pick & Place 성공!")
                    task_done = True
                    my_world.pause()

                # 디버그 출력
                event = controller.get_current_event()
                ee_pos, _ = robot.end_effector.get_world_pose()
                print(
                    f"  [event={event}] active={active_color} color_id={color_id} "
                    f"place={place_color}{placing_position} cube_z={cube_position[2]:.4f} ee_z={ee_pos[2]:.4f}"
                )

        was_playing = is_playing

    color_sub_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
