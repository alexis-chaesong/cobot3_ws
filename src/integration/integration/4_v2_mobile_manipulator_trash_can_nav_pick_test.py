"""
4_v2_mobile_manipulator_trash_can_nav_pick_test.py
--------------------------------------------------
4_mobile_manipulator_trash_can_nav_pick_test.py 의 v2.

기존 4_ 미션(Nav2 접근 → 파지 → big_trash 덤프 → 원위치 복귀 → 도킹)을 그대로
수행하되, 파지 직전에 RealSense D455 + YOLOv11s 가중치로 쓰레기통을 실시간
인식해서 게이트를 통과한 뒤에만 기존 파지 시퀀스를 실행한다.

추가된 흐름 (PICK Nav2 도착 후, TARGET_JOINTS_DEG 파지 전):
  1) link_5 에 RealSense 가 없으면 로컬 rsd455.usd 를 런타임 attach
  2) SURVEY_JOINTS_DEG 로 카메라 관측 자세 이동
  3) YOLO 실시간 추론 (여러 프레임) — small_trash_can / COCO 폴백
  4) 검출 성공 시에만 기존 4_ 파지 시퀀스 진행 (--force-pick 으로 우회 가능)

실행 방법 (총 3개 터미널) ───────────────────────────────────────────
  ★ carter_navigation 은 cobot3_ws 가 아니라
    ~/IsaacSim-ros_workspaces/humble_ws 에 있다. 아래 source 순서를 지킬 것.

  1) Nav2 bringup :
     source /opt/ros/humble/setup.bash
     source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
     source ~/cobot3_ws/install/setup.bash
     ros2 launch carter_navigation carter_navigation.launch.py \
         map:=~/cobot3_ws/src/assets/map/modified_hospital_map.yaml \
         params_file:=~/IsaacSim-ros_workspaces/humble_ws/install/carter_navigation/share/carter_navigation/params/carter_navigation_params_hospital.yaml
  2) 이 스크립트 (ros_set=/opt/ros/humble source 금지, Isaac python 만) :
     cd ~/cobot3_ws && isaac_ros   # 또는 bridge lib 를 LD_LIBRARY_PATH 앞에
     ~/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
         src/integration/integration/4_v2_mobile_manipulator_trash_can_nav_pick_test.py
  3) Nav2 커맨더 v2 (원본 trash_can_nav_pick_mission 은 수정하지 않음) :
     ros2 run commander trash_can_nav_pick_mission_v2

  상세: src/integration/integration/run_4_v2_nav_pick_terminals.md

YOLO/카메라만 로컬 검증 (Nav2 없이):
  .../python.sh .../4_v2_...py --skip-nav --pick-only

옵션:
  --headless / --skip-nav / --pick-only / --force-pick
  --weights PATH / --yolo-conf FLOAT / --yolo-frames N

★ Isaac Sim 실행 중 만난 문제와 해결 (개발 로그) ──────────────────────
  [P1] move_tash_can.usd 에는 RealSense 가 없음
       → link_5 에 로컬 rsd455.usd 를 런타임 reference + 알려진 mount xform 적용
  [P2] 커스텀 가중치 conf 가 낮음(sim close-up)
       → 기본 conf=0.05 + COCO yolo11s.pt 폴백(cup/bottle 등)
  [P3] Nav2 없이 전체 미션을 돌리면 /start_pick 대기에서 멈춤
       → --skip-nav/--pick-only 로 YOLO 게이트+파지만 단독 검증
  [P4] 카메라 RGB 가 초기 몇 프레임 공백
       → initialize 후 warm-up step + capture 재시도
  [P5] Isaac Sim python.sh(3.11) + 시스템 /opt/ros/humble rclpy(3.10) import 충돌
       → 증상: ModuleNotFoundError: rclpy._rclpy_pybind11 후 segfault
       → 원인: 터미널에서 `source /opt/ros/humble` (ros_set) 하면 PYTHONPATH 에
         python3.10 site-packages 가 먼저 들어가 Isaac python3.11 과 불일치
       → 해결: --skip-nav 는 rclpy 생략. Nav2 경로에서는 import 직전에
         /opt/ros 를 PYTHONPATH/LD_LIBRARY_PATH 에서 제거하고
         isaacsim.ros2.bridge/humble 의 py3.11 rclpy+lib 을 앞에 넣음.
         (터미널 1/3 Nav2·commander 는 계속 /opt/ros/humble 을 source)
  [P6] 런타임 AddReference 만으로는 RealSense RGB 가 안 나옴
       → CreateReference + mount xform + Camera 를 world.reset() 이전에 준비
  [P7] raw rsd455.usd 의 nested PhysicsRigidBodyAPI 가 link_5 articulation 과 충돌
       → PhysX: Incompatible size of velocity tensor (expected 6, received 12 (2,6))
         후 프로세스 Killed 되면 /clock 이 끊겨 터미널3 amcl/get_state 대기 고착
       → attach + world.reset 직후 rigidBodyEnabled=False / API 제거를 이중 적용
  [P9] 터미널3 `amcl/get_state service not available`
       → Nav2 lifecycle 이 active 되기 전에 commander 가 waitUntilNav2Active()
       → 순서: (1) Nav2 bringup (2) Isaac 4_v2 가 /clock·ROS 발행 시작
                (3) 그 다음 commander. T2 가 죽으면 amcl 대기는 정상 증상
  [P8] --skip-nav 검증 시 hospital 미병합이면 장면이 어두울 수 있음
       → RGB mean 이 너무 낮으면 DomeLight 를 약하게 보강 (과노출 방지 intensity)

아래부터는 원본 4_ docstring (기하/Nav2 handoff 설명) 을 유지한다.
--------------------------------------------------
move_tash_can.usd(Nova Carter + m0609 + Surface Gripper)에 modified_hospital.usd
(병원 환경, modified_hospital_map.yaml/png 를 뽑아낸 그 씬)를 합치고, 쓰레기통을
미리 골라둔 후보 좌표 중 하나에 무작위로 스폰시킨 뒤, Nav2로 그 앞까지 주행해서
3_mobile_manipulator_trash_can_pick_test.py 와 동일한 고정 관절각(TARGET_JOINTS_DEG)
방식으로 집어 올리는 테스트 스크립트.

★ 반드시 이해하고 쓸 것 (기하 가정) ─────────────────────────────────────
TARGET_JOINTS_DEG는 move_tash_can.usd 원본에서 "chassis_link 가 정확히
CARTER_START_POSE, 쓰레기통이 정확히 그 원래 스폰 좌표"인 상대 기하에서만 GUI로
검증된 고정 자세다. 쓰레기통을 다른 좌표에 스폰시켜도 이 자세가 그대로 통하려면
Nav2가 "쓰레기통 스폰 좌표 - OFFSET_TRASH_FROM_CHASSIS" 위치에 (원래와 같은 yaw=0
으로) 도착해야 한다 (OFFSET_TRASH_FROM_CHASSIS 값은 move_tash_can.usd 를 실제로
정착시켜 측정한 값, 아래 상수 참고).

문제는 carter_navigation_params_hospital.yaml 의 기본 goal tolerance
(xy=0.25 m, yaw=0.25 rad) 가 이 요구조건보다 훨씬 느슨하고, 게다가 쓰레기통 자체가
코스트맵 obstacle layer 에 장애물로 잡혀서 inflation 반경(코스트맵 튜닝 후에도
0.5m) 때문에 Nav2 가 chassis_goal_xy 근처까지 아예 못 다가간다 — 그래서 세 가지를
추가했다:
  1) Nav2 목표를 chassis_goal_xy 가 아니라 그보다 FINAL_APPROACH_DISTANCE 만큼 뒤로
     뺀 standoff 지점(inflation/footprint 충돌 밖, 안전하게 도달 가능)으로 잡고, 도착
     후 이 스크립트가 /cmd_vel 로 직접 마저 접근한다. Nav2 가 정확히 standoff_xy/
     standoff_yaw 에 도착한다고 가정하지 않고, 도착 직후 chassis 의 실제 위치를 다시
     재서 "지금 여기서 chassis_goal_xy 까지 가장 가까운 방향/거리"를 그때그때 계산
     한다(고정된 approach_dir 로 진입방향을 강제하지 않음) — 제자리 회전으로 그 방향을
     본 뒤 직진 → 도착 후 TARGET_JOINTS_DEG 가 요구하는 yaw=0 으로 다시 제자리 회전
     (differential-drive 라 방향 전환 없인 대각선 이동이 안 되므로 회전→직진→회전 3
     단계가 필요하다). Nav2 는 이 시점에 이미 완료 상태라 /cmd_vel 을 직접 써도 충돌
     없다.
  2) sync_rmpflow_base_pose(): 주행으로 chassis(=arm base)가 이동한 뒤에는 RMPflow 에
     최신 base pose 를 다시 알려줘야 한다 (RMPflow 는 world 프레임 타겟을 base pose
     기준으로 풀기 때문에, 이거 안 하면 이후 모든 IK 계산이 스폰 시점 기준으로 스테일
     하게 계산된다 — Isaac Sim RMPflow 문서 참고).
  3) TARGET_JOINTS_DEG 로 일단 접근한 뒤, 실측 쓰레기통 위치를 기준으로 creep 전진축과
     "수직인" 성분(좌우/높이 오차, creep 이 못 흡수하는 바로 그 방향)만 RMPflow IK 로
     한 번 보정한다(LATERAL_CORRECTION_MIN/MAX). 깊이 방향은 여전히 creep 에 맡긴다.

씬/Nav2 아키텍처 (10_carter_hospital_spray_nav.py 의 handoff 구조와 동일한 이유) ──
Nav2 클라이언트(BasicNavigator)의 blocking 호출을 이 스크립트 안에서 직접 하면,
그 blocking 동안 이 스크립트가 발행해야 할 /clock 이 멈춘다 → use_sim_time Nav2
노드들이 그 /clock 을 기다리며 멈춘다 → 즉 이 스크립트 자신이 만든 데드락에
걸린다. 그래서 Nav2 주행은 별도 프로세스(commander/trash_can_nav_pick_mission.py)
가 맡고, 이 스크립트는 /clock 발행 + 목표 퍼블리시 + /start_pick 대기(non-blocking)
만 한다.

다단계 미션 (소형 쓰레기통 파지 → big_trash 로 이동해 비우기 → 원위치 복귀해서 내려놓기
→ docking_station 복귀) ── 위 "Nav2 목표 발행 → /start_pick 대기 → 실제위치 기준
회전→직진→회전" 패턴을 run_nav_leg() 로 함수화해서 네 구간(PICK/DUMP/RETURN/DOCK)이
전부 재사용한다. 커맨더 쪽(trash_can_nav_pick_mission.py)도 같은 토픽(/trash_can_nav_goal,
/start_pick)으로 여러 구간을 반복 처리하도록 loop 로 되어있다. big_trash 접근은
4-진입점 계산 없이 항상 +Y 쪽(벽에서 1.55m 여유, -Y 쪽은 0m=완전히 막힘)에서 진입하는
단일 고정 방향이고, BIG_TRASH_FINAL_DISTANCE 는 GUI 검증 전 추정치라 실제로 돌려보면서
튜닝이 필요하다. 원위치 복귀(RETURN)는 파지 때와 같은 chassis_goal_xy/yaw 를 재사용하고,
dump 로 뒤틀린 손목에서 바로 Cartesian 이동하면 carter 차체(RMPflow 장애물 미등록)와
부딪힐 위험이 있어 TARGET_JOINTS_DEG 로 먼저 안전 복귀한 뒤 grasp_position/orientation
으로 내려가 Surface Gripper 를 연다. 마지막 DOCK 구간은 물체를 안 다루니 standoff 를
목표 지점과 동일하게 둬서(FINAL_APPROACH_DISTANCE 미사용) Nav2 가 바로 그 지점을
향하게 하고, 남는 미세 오차만 run_nav_leg 의 회전→직진→회전으로 보정한다.

실행 방법 (총 2개 터미널) ───────────────────────────────────────────
  1) Nav2 bringup :
     ros2 launch carter_navigation carter_navigation.launch.py \\
         map:=/home/rokey/cobot3_ws/src/assets/map/modified_hospital_map.yaml

  2) 이 스크립트 (Play 는 스크립트가 자동 진행) :
     /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \\
         src/integration/integration/4_mobile_manipulator_trash_can_nav_pick_test.py

  3) Nav2 커맨더 (초기 pose 설정 + 목표 수신 + 주행 + /start_pick 발행) :
     ros2 run commander trash_can_nav_pick_mission
--------------------------------------------------
"""

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

_THIS_DIR_EARLY = Path(__file__).resolve().parent
_WS_ROOT_EARLY = _THIS_DIR_EARLY.parents[2]
_DEFAULT_WEIGHTS = str(_WS_ROOT_EARLY / "src" / "perception" / "models" / "small_trash_can_yolo11s.pt")
_FALLBACK_WEIGHTS = str(_WS_ROOT_EARLY / "src" / "perception" / "models" / "yolo11s.pt")
_YOLO_PREVIEW_DIR = _WS_ROOT_EARLY / "src" / "perception" / "datasets" / "nav_pick_yolo_preview"

_PARSER = argparse.ArgumentParser(description="4_v2 Nav2 trash-can pick with YOLO gate")
_PARSER.add_argument("--headless", action="store_true")
_PARSER.add_argument("--skip-nav", action="store_true", help="Nav2 생략(로컬 YOLO+파지 검증용)")
_PARSER.add_argument("--pick-only", action="store_true", help="파지/들기/tuck 까지만 수행")
_PARSER.add_argument("--force-pick", action="store_true", help="YOLO 미검출이어도 파지 진행")
_PARSER.add_argument("--weights", type=str, default=_DEFAULT_WEIGHTS)
_PARSER.add_argument("--yolo-conf", type=float, default=0.05)
_PARSER.add_argument("--yolo-frames", type=int, default=12)
_PARSER.add_argument("--yolo-min-hits", type=int, default=2)
_ARGS, _UNKNOWN = _PARSER.parse_known_args()


def _reexec_with_isaac_ros_env_if_needed() -> None:
    """Nav2 경로에서 /opt/ros/humble 이 섞여 있으면 LD_LIBRARY_PATH 가 프로세스
    시작 시점에 고정되어 mid-run 수정이 소용없다. SimulationApp 전에 환경을
    정리하고 자기 자신을 한 번 re-exec 한다 (P5)."""
    import os

    if _ARGS.skip_nav:
        return
    if os.environ.get("ISAAC_P5_ENV_READY") == "1":
        return

    release = Path.home() / "dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
    env_root = os.environ.get("ISAAC_SIM_PATH") or os.environ.get("ISAACSIM_PATH")
    if env_root:
        release = Path(env_root)
    # python.sh 경유 시 executable 기준 release 추정
    exe = Path(sys.executable).resolve()
    if len(exe.parents) >= 4 and (exe.parents[3] / "exts" / "isaacsim.ros2.bridge").is_dir():
        release = exe.parents[3]

    bridge = release / "exts" / "isaacsim.ros2.bridge" / "humble"
    rclpy_so = bridge / "rclpy" / "rclpy" / "_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so"
    if not rclpy_so.is_file():
        print(f"[ROS][WARN] Isaac bridge rclpy not found at {rclpy_so} — P5 re-exec skipped", flush=True)
        return

    def scrub(path_str: str) -> list[str]:
        return [p for p in (path_str or "").split(":") if p and "/opt/ros/" not in p]

    py = os.environ.get("PYTHONPATH", "")
    ld = os.environ.get("LD_LIBRARY_PATH", "")
    needs = ("/opt/ros/" in py) or ("/opt/ros/" in ld) or (str(bridge / "rclpy") not in py.split(":"))
    if not needs and os.environ.get("ROS_DISTRO") == "humble":
        # 이미 깨끗해도 bridge 를 앞에 보장
        pass

    new_env = os.environ.copy()
    new_env["PYTHONPATH"] = ":".join([str(bridge / "rclpy")] + scrub(py))
    new_env["LD_LIBRARY_PATH"] = ":".join([str(bridge / "lib")] + scrub(ld))
    new_env["ROS_DISTRO"] = new_env.get("ROS_DISTRO") or "humble"
    new_env["ISAAC_P5_ENV_READY"] = "1"
    # ament 가 시스템 humble 을 다시 끌어오지 않도록
    for key in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "CMAKE_PREFIX_PATH"):
        if key in new_env and "/opt/ros/" in new_env[key]:
            new_env[key] = ":".join(scrub(new_env[key]))

    print("[ROS] P5: re-exec with Isaac bridge py3.11 rclpy (scrub /opt/ros from env)", flush=True)
    os.execve(sys.executable, [sys.executable, *sys.argv], new_env)


_reexec_with_isaac_ros_env_if_needed()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": _ARGS.headless})

# ROS2 bridge 확장 활성화 (Carter 내장 그래프 발행 + Nav2 구동에 필수)
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
from isaacsim.sensors.camera import Camera

import cv2
from ultralytics import YOLO

# Isaac Sim python(3.11) 과 시스템 ROS Humble rclpy(3.10) 가 충돌할 수 있어
# Nav2 경로에서만 지연 import 한다 (--skip-nav 로컬 검증은 ROS 없이 동작).
rclpy = None
Bool = PoseStamped = Twist = Clock = None


def prepare_isaac_ros_python_env() -> str:
    """Isaac python.sh(3.11) 에서 시스템 /opt/ros/humble(3.10) rclpy 가 잡히면
    `_rclpy_pybind11` 를 못 찾고 크래시한다 (P5). bridge 번들 py3.11 rclpy 를
    PYTHONPATH/LD_LIBRARY_PATH 맨 앞에 두고 /opt/ros 항목을 제거한다.
    Returns: bridge humble root path (logging용).
    """
    import os
    from pathlib import Path as _Path

    # python.sh → .../release/python.sh 기준 release 루트
    release_root = _Path(__file__).resolve()
    # 스크립트는 cobot3_ws 쪽이라, Isaac release 는 알려진 경로/환경변수에서 찾는다.
    candidates = []
    env_root = os.environ.get("ISAAC_SIM_PATH") or os.environ.get("ISAACSIM_PATH")
    if env_root:
        candidates.append(_Path(env_root))
    # python.sh 로 실행 중이면 sys.executable 이 kit/python/bin/python3
    import sys as _sys
    exe = _Path(_sys.executable).resolve()
    # .../release/kit/python/bin/python3 → parents[3] == release
    if len(exe.parents) >= 4 and exe.parents[0].name == "bin":
        candidates.append(exe.parents[3])
    candidates.append(_Path.home() / "dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")

    bridge_humble = None
    for root in candidates:
        cand = root / "exts" / "isaacsim.ros2.bridge" / "humble"
        if (cand / "rclpy" / "rclpy" / "_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so").is_file():
            bridge_humble = cand
            break
        # egg 레이아웃 변형
        if (cand / "rclpy").is_dir() and (cand / "lib").is_dir():
            bridge_humble = cand
            break
    if bridge_humble is None:
        raise RuntimeError(
            "isaacsim.ros2.bridge/humble 을 찾지 못했습니다. "
            "터미널 2 에서는 `source /opt/ros/humble` 를 쓰지 말고, "
            "Isaac python.sh 로만 실행하세요 (P5)."
        )

    def _scrub(path_str: str) -> list[str]:
        out = []
        for p in (path_str or "").split(":"):
            if not p:
                continue
            if "/opt/ros/" in p:
                continue
            out.append(p)
        return out

    py_parts = [str(bridge_humble / "rclpy")] + _scrub(os.environ.get("PYTHONPATH", ""))
    # sys.path 에서도 /opt/ros 를 제거한 뒤 bridge 를 맨 앞
    scrubbed_sys = [p for p in _sys.path if p and "/opt/ros/" not in p]
    _sys.path[:] = [str(bridge_humble / "rclpy")] + [
        p for p in scrubbed_sys if p != str(bridge_humble / "rclpy")
    ]
    os.environ["PYTHONPATH"] = ":".join(py_parts)

    ld_parts = [str(bridge_humble / "lib")] + _scrub(os.environ.get("LD_LIBRARY_PATH", ""))
    os.environ["LD_LIBRARY_PATH"] = ":".join(ld_parts)
    os.environ.setdefault("ROS_DISTRO", "humble")

    print(f"[ROS] P5 guard: using Isaac bridge rclpy at {bridge_humble / 'rclpy'}", flush=True)
    print("[ROS] P5 guard: scrubbed /opt/ros from PYTHONPATH/LD_LIBRARY_PATH for this process", flush=True)
    return str(bridge_humble)

_THIS_DIR = Path(__file__).resolve().parent  # src/integration/integration
_WS_ROOT = _THIS_DIR.parents[2]  # cobot3_ws
ASSETS_DIR = _WS_ROOT / "src" / "assets"
RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

from m0609_rmpflow_controller import RMPFlowController  # noqa: E402

# ─────────────────────────────────────────────────────────
# 설정값
# ─────────────────────────────────────────────────────────
USD_PATH = str(ASSETS_DIR / "scenes" / "move_tash_can.usd")
HOSPITAL_USD = str(ASSETS_DIR / "map" / "modified_hospital.usd")
ARTICULATION_ROOT_PATH = "/World/Nova_Carter_ROS/chassis_link"
ARM_USD_ROOT = "/World/m0609"
EE_LINK_NAME = "link_6"
TRASH_CAN_PRIM_PATH = "/World/small_trash_can_body"
SURFACE_GRIPPER_PATH = f"{ARM_USD_ROOT}/{EE_LINK_NAME}/mop_surface_gripper"
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]

# GUI에서 직접 찾은, 쓰레기통 옆면(x축 방향 면)을 잡을 수 있고 Carter도 넘어가지
# 않는 것으로 검증된 자세 (degree). 3_mobile_manipulator_trash_can_pick_test.py 와 동일.
TARGET_JOINTS_DEG = [-90.0, 101.0, 50.0, -94.0, 91.8, -1.1]
# YOLO 관측용(파지 직전). 파지 자세보다 물러선 각도라 쓰레기통 전체가 더 잘 들어온다.
SURVEY_JOINTS_DEG = [-90.0, 70.0, 65.0, -110.0, 90.0, 0.0]
REALSENSE_PRIM_PATH = "/World/m0609/link_5/realsense_d455"
LOCAL_RSD455_USD = str(
    _WS_ROOT_EARLY
    / "src/assets/robots/Collected_m0609_camera"
    / "omniverse-content-production.s3-us-west-2.amazonaws.com"
    / "Assets/Isaac/5.1/Isaac/Sensors/Intel/RealSense/rsd455.usd"
)
CAMERA_RESOLUTION = (640, 480)
YOLO_TARGET_CLASS = "small_trash_can"
# Nav2 RViz(/scan·맵) 와 구분되는 YOLO 시각화 토픽 — RViz Image 디스플레이로 구독
YOLO_IMAGE_TOPIC = "/yolo/annotated"
YOLO_STATUS_TOPIC = "/yolo/status"
YOLO_WINDOW_NAME = "YOLO Trash-Can Detection (v2)"
# 이동(주행) 중에는 j1=0도로 유지 — big_trash 에서 비울 때도 그대로 0도 유지(DUMP_J1_DEG
# 와 동일값이라 dump 단계에서 j1 은 사실상 그대로, j6 만 실제로 움직인다).
TUCK_J1_DEG = 0.0

# big_trash(8.010, -2.169, 0.060) 앞에 도착했다고 가정하고 쓰레기통을 기울여 비우는
# 자세. j1 은 TUCK_J1_DEG 와 동일(0도, 이동 중과 동일 자세 유지), j6 만 tuck 이후
# 유지되던 현재값에서 상대 회전(손목 뒤집기)시켜 비운다. 180도/-180도 둘 다 최종
# 각도는 같지만(wrap), ramp_to_joint_positions 는 그 사이를 그냥 선형보간하므로 부호가
# 회전 "방향"을 결정한다 — 시계방향으로 돌도록 음수를 사용.
DUMP_J1_DEG = TUCK_J1_DEG
DUMP_J6_ROTATE_DEG = -180.0
# dump 는 j6 손목 하나만 크게 움직이는 단순 동작이라, 파지처럼 신중해야 할 이유가 없다.
# JOINT_RAMP_STEPS(300=5초)를 그대로 쓰면 불필요하게 오래 걸려서 dump 전용으로 더
# 빠른 스텝 수를 따로 둔다.
DUMP_RAMP_STEPS = 90

# ★ 튜닝 포인트: 덤프 직후 곧바로 Nav2(RETURN 구간)를 켜면 big_trash 바로 앞이라
# 경로계획/코스트맵이 애매해질 수 있어서, 먼저 이 거리만큼 뒤로 물러난 뒤에 Nav2 목표를
# 발행한다(drive_straight_open_loop 의 reverse=True 재사용, 회전 없이 heading 유지).
POST_DUMP_BACKUP_DISTANCE = 0.6  # [m]

# ── big_trash 로 이동하는 두 번째 Nav2 구간. 4-진입점 계산은 안 씀 — 씬 상 거의 직선
# 으로 갈 수 있고, +Y 쪽(북쪽)이 벽에서 1.55m 로 제일 트여있음(맵 자동 스캔으로 확인,
# -Y 쪽은 0m=완전히 막힘). 그래서 항상 +Y 쪽에서 -Y(남쪽)를 보고 진입한다.
# ★ 튜닝 포인트: BIG_TRASH_FINAL_DISTANCE 는 GUI 로 검증된 값이 아니라 추정치라, 실제로
# 덤프 자세(DUMP_J1_DEG/DUMP_J6_ROTATE_DEG)가 big_trash 위로 잘 오는지 보면서 조정할 것.
BIG_TRASH_POSITION_XY = np.array([8.010060678957633, -2.169276378671552])
BIG_TRASH_APPROACH_DIR = np.array([0.0, 1.0])       # big_trash -> standoff 방향 (+Y)
BIG_TRASH_STANDOFF_DISTANCE = 1.0     # Nav2 목표(standoff)를 big_trash 에서 이만큼 +Y 로 뺀 거리 [m]
BIG_TRASH_FINAL_DISTANCE = 0.5        # standoff 도착 후 cmd_vel 로 마저 접근할 최종 거리 [m] (추정치, 튜닝 필요)

NO_GRIPPER_URDF_PATH = str(_WS_ROOT / "src" / "doosan-robot2" / "urdf" / "m0609_isaac_sim.urdf")

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING = 1e4
DRIVE_MAX_FORCE = 1e8
PHYSICS_DT = 1.0 / 60.0

LIFT_OFFSET = np.array([0.0, 0.0, 0.75])  # big_trash 위로 넘기기엔 0.60m 로는 낮아서 상향

POSITION_TOLERANCE = 0.03
MAX_APPROACH_STEPS = 400
MAX_EE_LINEAR_SPEED = 0.10
MIN_INTERP_STEPS = 60
MAX_INTERP_STEPS = 600
SETTLE_STEPS = 60
GRASP_HOLD_STEPS = 60
JOINT_RAMP_STEPS = 300

CREEP_STEP_SIZE = 0.005
# 좌우/높이 보정이 IK 도달 한계 때문에 완전히 수렴 못하고 몇 cm~십수 cm 남은 채 끝나는
# 경우가 있어(바닥 충돌이 아니라 그 자세에서의 IK local minimum 으로 보임), creep 여유를
# 0.2m -> 0.3m 로 늘려 "간신히 파지" 상황의 안전마진을 키운다.
CREEP_MAX_STEPS = 60
CREEP_SETTLE_STEPS = 5

# Nav2 도착 오차 보정(전진축과 수직인 성분)의 적용 범위. 1cm 미만은 잡음이라 보정 생략,
# 0.4m 초과는 Nav2 goal tolerance(기본 xy=0.25m) 로 설명되지 않는 수준이라 다른 문제(도착
# 실패 등)일 가능성이 높아 보정 없이 그대로 진행(로그로 확인 가능하게 경고만 출력).
LATERAL_CORRECTION_MIN = 0.01
LATERAL_CORRECTION_MAX = 0.4

# ── 이전 랜덤 스폰 후보 (modified_hospital_map.png free space 자동 스캔, 참고용으로 유지).
#    지금은 쓰레기통을 큰 쓰레기통(big_trash)에 비우고 원위치에 돌려놓는 다단계 작업을
#    만드는 중이라, 재현 가능한 테스트를 위해 아래 TRASH_SPAWN_FIXED 로 고정해서 쓴다 ──
TRASH_SPAWN_CANDIDATES = [
    (8.025, 6.575),
    (17.225, 5.775),
    (17.425, 3.175),
    (9.625, 6.975),
    (6.625, 7.175),
]
TRASH_SPAWN_FIXED = (7.5, 7.5)  # 벽에서 2.75m 여유 확인됨
TRASH_SPAWN_Z = 0.0786364536328308  # move_tash_can.usd 원본 스폰 높이(낙하 후 바닥에 정착)

# move_tash_can.usd 를 실제로 physics 정착시켜 측정한 chassis_link ↔ 쓰레기통의
# 상대 위치 (probe 스크립트로 측정, TARGET_JOINTS_DEG 가 검증된 그 기하).
# trash_world_pos = chassis_world_pos + OFFSET_TRASH_FROM_CHASSIS
OFFSET_TRASH_FROM_CHASSIS = np.array([-0.750368208, -0.758035257])

# move_tash_can.usd 원본 스폰 pose (chassis_link, yaw 는 거의 0) — modified_hospital.usd
# 의 docking_station_02 마커(x=6.9807, y=0) 위치로 맞춤. Nav2 커맨더 쪽
# trash_can_nav_pick_mission.py 의 CARTER_START_POSE 와 반드시 일치시킬 것
# (AMCL 초기위치 ≠ 실제 스폰 위치면 이후 모든 위치추정이 틀어진다).
CARTER_START_POSE = dict(x=16.66290495232035, y=-0.0029517927591273807, yaw_deg=0.0)

NAV_GOAL_TOPIC = "/trash_can_nav_goal"
START_PICK_TOPIC = "/start_pick"
CMD_VEL_TOPIC = "/cmd_vel"

# ── 코스트맵 옵션 B: Nav2는 쓰레기통에서 안전거리(inflation 밖)까지만 데려다주고,
# 그 이후 chassis_goal_xy(검증된 상대기하 지점)까지 남은 구간은 이 스크립트가 Nav2/
# 코스트맵과 무관하게 직접 /cmd_vel 로 접근한다(Nav2 는 이 시점에 이미 완료 상태라
# 충돌 없음, 3_mobile_manipulator_trash_can_pick_test.py 의 drive_straight() 와 같은
# 발상이나 heading-hold 로 직진 정밀도를 높임). Differential-drive 라 방향 전환 없이
# 대각선 이동은 못하므로: Nav2 도착 시 이미 접근 방향(approach yaw)을 보게 만들고
# → 그 방향으로 직진 → 도착 후 TARGET_JOINTS_DEG 가 요구하는 yaw=0 으로 제자리 회전.
FINAL_APPROACH_DISTANCE = 0.4    # standoff -> chassis_goal_xy 거리 [m] (inflation_radius=0.5 보다 여유있게)
FINAL_APPROACH_SPEED = 0.15       # [m/s]
# Nova Carter 뒷쪽 자유 스위블 캐스터는 "이동거리"에 비례해서 정렬되지(속도와 무관) —
# 그래서 속도 ramp 만으로는 못 없애고, 아래 DRIFT 게인으로 직진 중 헤딩을 계속 미세
# 보정해야 한다. ramp 자체는 급발진만 완화하는 부수 효과라 남겨둔다.
FINAL_APPROACH_RAMP_TIME = 0.5    # [s]
# 회전(rotate_in_place)으로 큰 방향은 이미 맞춘 뒤라 오차가 작으므로, 아주 낮은 게인
# 으로 직진 중 헤딩만 미세 보정해서 캐스터 드리프트를 상쇄한다. 코너 진입처럼 오차가
# 큰 상황에서 진동났던 예전 heading-hold(KP=2.0 급)와 달리, 여기선 오차가 이미 작아서
# 낮은 게인으로도 충분하고 오버슈트 위험도 작다.
# ※ 캐스터 교란은 "일시적"(초반에 생겼다 사라짐)인데 KD 가 KP 대비 너무 약하면, 보정이
# 목표 헤딩을 지나쳐 반대로 튕겨나가는 오버슈트가 생긴다(실측으로 확인됨) — 그래서 KP
# 는 낮추고 KD(감쇠) 비중을 크게 올렸다.
FINAL_APPROACH_DRIFT_KP = 0.15
FINAL_APPROACH_DRIFT_KD = 0.4
FINAL_APPROACH_DRIFT_W_MAX = 0.15
FINAL_APPROACH_DRIFT_MAX_W_STEP = 0.01
# 큰 방향 전환(회전)과 직진을 동시에 명령하면, 코너 진입처럼 회전량이 큰 상황에서
# chassis 의 회전 관성 때문에 "물고기처럼" 좌우로 진동한다. 그래서 큰 회전은 절대
# 직진과 동시에 명령하지 않는다: 제자리 회전(각속도만, 방향 정렬) -> 직진(낮은 게인의
# 미세 헤딩 보정만 동반, 위 FINAL_APPROACH_DRIFT_* 참고) -> 반복 없이 한 번씩만.
# 진입점이 4개(0/90/180/270도) 중 하나라 최악의 경우 270도(4.7rad)까지 돌아야 하는데,
# 예전 W_MAX=0.4rad/s 로는 그것만 12초 가까이 걸렸다. KD/rate-limit 로 이미 진동은
# 억제하고 있으니 최고 각속도와 그 각속도로의 가속(rate limit) 자체를 올려서 단축한다.
FINAL_ROTATE_KP = 2.0
FINAL_ROTATE_KD = 0.5
FINAL_ROTATE_W_MAX = 1.0
FINAL_ROTATE_MAX_W_STEP = 0.05
FINAL_ROTATE_TOLERANCE_RAD = 0.02
FINAL_MOVE_SETTLE_STEPS = 30

# ★ 튜닝 포인트: chassis_goal_yaw 로 최종 회전을 마친 뒤, chassis/tool0 위치를 다시
# 확인하고 이 거리만큼 한 번 더 살짝 전진한다(open-loop, drive_straight_open_loop 재사용).
# 값을 늘리면 더 깊이 들어가고, 0으로 두면 추가 전진 없이 예전과 동일하게 동작한다.
FINAL_NUDGE_DISTANCE = 0.00  # [m]

# ★ 튜닝 포인트: 위 FINAL_NUDGE_DISTANCE 는 최종 회전 "이후" 전진인데, 이건 반대로
# entry_yaw 방향 직진(= chassis_goal_yaw 로 최종 회전하기 "이전") 거리에 더해지는 값.
# standoff/진입점 계산(FINAL_APPROACH_DISTANCE)은 그대로 두고, 최종 회전 전에만 더
# 들어가고 싶을 때 이 값을 늘린다.
PRE_ROTATE_NUDGE_DISTANCE = 0.25  # [m]



def resolve_yolo_weights(path: str) -> str:
    p = Path(path)
    if p.is_file():
        return str(p)
    if Path(_FALLBACK_WEIGHTS).is_file():
        print(f"[YOLO][WARN] weights missing ({path}) → fallback {_FALLBACK_WEIGHTS}", flush=True)
        return _FALLBACK_WEIGHTS
    raise FileNotFoundError(f"YOLO weights not found: {path}")


def _strip_nested_rigid_bodies(stage, root_path: str) -> int:
    """rsd455.usd 기본 에셋은 RSD455 루트에 PhysicsRigidBodyAPI 가 있다.
    link_5(이미 articulation rigid body) 아래에 붙이면 nested RB 로 PhysX tensor
    가 깨진다 (expected 6, received 12 shape (2,6)) — P7.
    reference composition 에서는 RemoveAPI 만으로 안 지워지는 경우가 있어
    physics:rigidBodyEnabled=False / collisionEnabled=False 도 함께 박는다."""
    from pxr import Sdf

    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return 0
    touched = 0
    strip_names = (
        "PhysicsRigidBodyAPI",
        "PhysxRigidBodyAPI",
        "PhysicsCollisionAPI",
        "PhysxCollisionAPI",
    )
    for prim in Usd.PrimRange(root):
        schemas = list(prim.GetAppliedSchemas())
        has_rb = prim.HasAPI(UsdPhysics.RigidBodyAPI) or any("RigidBody" in s for s in schemas)
        has_col = prim.HasAPI(UsdPhysics.CollisionAPI) or any("Collision" in s for s in schemas)
        if not (has_rb or has_col):
            continue

        # 1) reference 위에서도 먹는 확실한 비활성화
        if has_rb:
            rb = UsdPhysics.RigidBodyAPI.Apply(prim)
            rb.CreateRigidBodyEnabledAttr(False)
            touched += 1
        if has_col:
            col = UsdPhysics.CollisionAPI.Apply(prim)
            col.CreateCollisionEnabledAttr(False)
            touched += 1

        # 2) 가능하면 API 자체 제거 (YOLO 씬과 동일)
        try:
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                touched += 1
        except Exception:
            pass
        try:
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                prim.RemoveAPI(UsdPhysics.CollisionAPI)
                touched += 1
        except Exception:
            pass
        for schema in schemas:
            if schema in strip_names:
                try:
                    prim.RemoveAppliedSchema(schema)
                    touched += 1
                except Exception:
                    pass

        # 3) 속성 직접 오버라이드 (composition 최후 수단)
        for name, val in (
            ("physics:rigidBodyEnabled", False),
            ("physics:collisionEnabled", False),
        ):
            attr = prim.GetAttribute(name)
            if not attr:
                attr = prim.CreateAttribute(name, Sdf.ValueTypeNames.Bool)
            if attr:
                attr.Set(val)
                touched += 1
    return touched


def ensure_realsense_on_link5(stage) -> str:
    """move_tash_can.usd 에는 RealSense 가 없으므로 link_5 에 로컬 rsd455 를 attach.
    mobile_manipulator_with_yolov11s_v1.usd 에서 확인한 mount xform 을 그대로 사용.
    ★ world.reset() 이전에 호출할 것 (P6/P7)."""
    import omni.kit.commands
    root = REALSENSE_PRIM_PATH
    if not stage.GetPrimAtPath(root).IsValid():
        if not Path(LOCAL_RSD455_USD).is_file():
            raise FileNotFoundError(f"local RealSense USD missing: {LOCAL_RSD455_USD}")
        # CreateReference 가 payload composition 에 더 안정적 (AddReference 만으로
        # Camera render product 가 안 뜨는 경우가 있었음 — P6).
        omni.kit.commands.execute(
            "CreateReference",
            usd_context=omni.usd.get_context(),
            path_to=root,
            asset_path=LOCAL_RSD455_USD,
        )
        xform = UsdGeom.Xformable(stage.GetPrimAtPath(root))
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(0.00003807931889507343, 0.000038103651953891315, -0.07))
        xform.AddRotateZYXOp().Set(Gf.Vec3f(0.0, 0.0, 270.0))
        xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
        n_strip = _strip_nested_rigid_bodies(stage, root)
        print(
            f"[YOLO] RealSense attached at {root} from {LOCAL_RSD455_USD} "
            f"(stripped nested RB/collision ops={n_strip})",
            flush=True,
        )
        for _ in range(60):
            simulation_app.update()
    else:
        n_strip = _strip_nested_rigid_bodies(stage, root)
        print(f"[YOLO] RealSense already present: {root} (strip ops={n_strip})", flush=True)

    preferred = [
        f"{root}/RSD455/Camera_OmniVision_OV9782_Color",
        f"{root}/RSD455/Visual/Camera_OmniVision_OV9782_Color",
    ]
    for p in preferred:
        if stage.GetPrimAtPath(p).IsValid():
            print(f"[YOLO] color camera prim = {p}", flush=True)
            return p
    cands = []
    rs = stage.GetPrimAtPath(root)
    for prim in Usd.PrimRange(rs):
        if prim.GetTypeName() == "Camera":
            name = prim.GetName().lower()
            score = (("color" in name) * 3) + (("ov9782" in name))
            if any(k in name for k in ("depth", "left", "right", "pseudo")):
                score -= 2
            cands.append((score, str(prim.GetPath())))
    if not cands:
        # dump tree for debug
        print("[YOLO][ERROR] cameras under realsense:", flush=True)
        for prim in Usd.PrimRange(rs):
            print(f"  {prim.GetPath()} ({prim.GetTypeName()})", flush=True)
        raise RuntimeError("RealSense color Camera prim not found after attach")
    cands.sort(reverse=True)
    print(f"[YOLO] using camera {cands[0][1]}", flush=True)
    return cands[0][1]


def ensure_min_scene_light(stage, intensity: float = 400.0) -> None:
    """hospital 미병합(--skip-nav) 장면에서 카메라가 거의 검정이면 DomeLight 보강 (P8)."""
    from pxr import UsdLux
    path = "/World/YoloAssistDomeLight"
    if stage.GetPrimAtPath(path).IsValid():
        return
    dome = UsdLux.DomeLight.Define(stage, path)
    dome.CreateIntensityAttr(float(intensity))
    print(f"[YOLO] added assist DomeLight intensity={intensity}", flush=True)


def make_realsense_camera(cam_path: str) -> Camera:
    """World.reset() 전에 Camera 객체를 만들고, reset 이후 initialize 한다 (정상 YOLO 씬과 동일)."""
    return Camera(prim_path=cam_path, name="realsense_color", resolution=CAMERA_RESOLUTION, frequency=30)


def warm_realsense_camera(camera: Camera, world, stage=None) -> np.ndarray:
    """initialize + warm-up. 유효 RGB(mean>=1) 한 장을 반환. 너무 어두우면 DomeLight 보강."""
    camera.initialize()
    for _ in range(60):
        world.step(render=True)
    rgb = capture_rgb(camera, world, tries=80)
    if rgb is None or float(rgb.mean()) < 1.0:
        if stage is not None:
            ensure_min_scene_light(stage, intensity=400.0)
            for _ in range(30):
                world.step(render=True)
        print("[YOLO][WARN] camera RGB weak/empty — re-initialize", flush=True)
        camera.initialize()
        for _ in range(60):
            world.step(render=True)
        rgb = capture_rgb(camera, world, tries=80)
    if rgb is None:
        raise RuntimeError(
            f"RealSense RGB unavailable at {camera.prim_path}. "
            "Check P6/P7 (CreateReference + strip nested RigidBody)."
        )
    print(f"[YOLO] camera RGB OK shape={rgb.shape} brightness={float(rgb.mean()):.1f}", flush=True)
    return rgb


def capture_rgb(camera: Camera, world, tries: int = 40):
    for _ in range(tries):
        world.step(render=True)
        rgb = camera.get_rgb()
        if rgb is None:
            continue
        arr = np.asarray(rgb)
        if arr.ndim == 3 and arr.size and np.max(arr) > 0:
            return np.clip(arr, 0, 255).astype(np.uint8) if arr.dtype != np.uint8 else arr
    return None


def _pick_best_det(result, conf: float):
    best = None
    for box in result.boxes:
        cls_id = int(box.cls.item())
        name = result.names.get(cls_id, str(cls_id))
        score = float(box.conf.item())
        if score < conf:
            continue
        xyxy = [float(v) for v in box.xyxy[0].tolist()]
        ok = (name == YOLO_TARGET_CLASS) or (name in {"cup", "bottle", "bowl", "vase", "toilet"})
        if not ok:
            continue
        area = max(0.0, xyxy[2] - xyxy[0]) * max(0.0, xyxy[3] - xyxy[1])
        cand = {"class_name": name, "confidence": score, "xyxy": xyxy, "class_id": cls_id, "area": area}
        if best is None or (cand["confidence"], cand["area"]) > (best["confidence"], best["area"]):
            best = cand
    return best


def detect_trash_can(model: YOLO, bgr, conf: float, fallback_model=None):
    result = model.predict(source=bgr, conf=conf, verbose=False)[0]
    best = _pick_best_det(result, conf)
    annotated = result.plot()
    if best is None and fallback_model is not None:
        fb = fallback_model.predict(source=bgr, conf=max(conf, 0.15), verbose=False)[0]
        best = _pick_best_det(fb, max(conf, 0.15))
        annotated = fb.plot()
        result = fb
    return best, annotated


def draw_yolo_hud(ann_bgr, *, frame_i: int, frames: int, hits: int, min_hits: int,
                  det, conf_thresh: float):
    """Nav2(라이다·맵)와 구분되도록 YOLO 전용 HUD/박스를 크게 그린다."""
    img = ann_bgr.copy()
    h, w = img.shape[:2]
    # 상단 배너
    cv2.rectangle(img, (0, 0), (w, 56), (40, 20, 0), -1)
    cv2.putText(
        img, "YOLO GATE (camera)  !=  Nav2 (lidar/map)",
        (12, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 220, 255), 2, cv2.LINE_AA,
    )
    status = f"frame {frame_i + 1}/{frames}  hits={hits}/{min_hits}  conf>={conf_thresh:.2f}"
    cv2.putText(img, status, (12, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    if det is not None:
        x1, y1, x2, y2 = [int(v) for v in det["xyxy"]]
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
        label = f"HIT {det['class_name']} {det['confidence']:.2f}"
        cv2.putText(img, label, (x1, max(20, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        badge = "DETECT"
        color = (0, 200, 0)
    else:
        badge = "MISS"
        color = (0, 0, 220)
        cv2.putText(img, "no trash can in frame", (12, 130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
    cv2.rectangle(img, (w - 170, 64), (w - 12, 110), color, -1)
    cv2.putText(img, badge, (w - 155, 98), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
    return img


def bgr_to_ros_image(bgr, stamp=None, frame_id: str = "realsense_color"):
    """cv_bridge 없이 sensor_msgs/Image 생성 (Isaac py3.11 환경용)."""
    from sensor_msgs.msg import Image as RosImage
    msg = RosImage()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = int(bgr.shape[0]), int(bgr.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(bgr, dtype=np.uint8).tobytes()
    return msg


def publish_yolo_viz(ros_node, image_pub, status_pub, ann_bgr, status_text: str):
    if ros_node is None or image_pub is None:
        return
    try:
        stamp = ros_node.get_clock().now().to_msg()
    except Exception:
        stamp = None
    image_pub.publish(bgr_to_ros_image(ann_bgr, stamp=stamp))
    if status_pub is not None:
        from std_msgs.msg import String
        s = String()
        s.data = status_text
        status_pub.publish(s)


def show_yolo_window(ann_bgr):
    """headless 가 아니면 OpenCV 창으로 YOLO 결과를 실시간 표시."""
    if _ARGS.headless:
        return
    try:
        cv2.namedWindow(YOLO_WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.imshow(YOLO_WINDOW_NAME, ann_bgr)
        cv2.waitKey(1)
    except Exception as exc:
        print(f"[YOLO][WARN] OpenCV window failed: {exc}", flush=True)


def yolo_realtime_gate(world, robot, dof_names, camera, model, fallback_model,
                        frames: int, conf: float, min_hits: int, preview_dir: Path,
                        ros_node=None, image_pub=None, status_pub=None,
                        clock_pub=None):
    """파지 전 실시간 YOLO 게이트. 성공 시 best detection dict, 실패 시 None.
    ros_node 가 있으면 /yolo/annotated · /yolo/status 로 실시간 퍼블리시해
    Nav2 RViz(라이다/맵) 와 시각적으로 구분한다."""
    preview_dir.mkdir(parents=True, exist_ok=True)
    (preview_dir / "raw").mkdir(exist_ok=True)
    (preview_dir / "annotated").mkdir(exist_ok=True)

    def _pub_clock():
        if ros_node is None or clock_pub is None or Clock is None:
            return
        t = float(world.current_time)
        cmsg = Clock()
        cmsg.clock.sec = int(t)
        cmsg.clock.nanosec = int(round((t - int(t)) * 1e9))
        clock_pub.publish(cmsg)

    print(f"[YOLO] survey pose → realtime detect (frames={frames}, conf={conf}, min_hits={min_hits})", flush=True)
    if image_pub is not None:
        print(f"[YOLO] live viz → RViz Image topic '{YOLO_IMAGE_TOPIC}' "
              f"+ status '{YOLO_STATUS_TOPIC}'"
              + ("" if _ARGS.headless else f" + window '{YOLO_WINDOW_NAME}'"),
              flush=True)
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, SURVEY_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(30):
        world.step(render=True)
        _pub_clock()
    # survey 직후 render product 가 한 번 비는 경우 대비
    probe = capture_rgb(camera, world, tries=20)
    if probe is None:
        print("[YOLO][WARN] RGB empty after survey — re-initialize camera", flush=True)
        camera.initialize()
        for _ in range(45):
            world.step(render=True)
            _pub_clock()

    hits = []
    last_ann = None
    for i in range(frames):
        _pub_clock()
        rgb = capture_rgb(camera, world)
        if rgb is None:
            print(f"[YOLO] frame {i}: no RGB", flush=True)
            continue
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        det, ann = detect_trash_can(model, bgr, conf, fallback_model)
        if det is not None:
            hits.append(det)
            print(f"[YOLO] frame {i}: HIT {det['class_name']} conf={det['confidence']:.3f}", flush=True)
            status = f"HIT {det['class_name']} conf={det['confidence']:.3f} ({len(hits)}/{min_hits})"
        else:
            print(f"[YOLO] frame {i}: miss", flush=True)
            status = f"MISS ({len(hits)}/{min_hits})"
        hud = draw_yolo_hud(
            ann, frame_i=i, frames=frames, hits=len(hits), min_hits=min_hits,
            det=det, conf_thresh=conf,
        )
        last_ann = hud
        stem = f"gate_{i:03d}"
        cv2.imwrite(str(preview_dir / "raw" / f"{stem}.jpg"), bgr)
        cv2.imwrite(str(preview_dir / "annotated" / f"{stem}.jpg"), hud)
        publish_yolo_viz(ros_node, image_pub, status_pub, hud, status)
        show_yolo_window(hud)
        if ros_node is not None and rclpy is not None:
            rclpy.spin_once(ros_node, timeout_sec=0.0)

    summary = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "frames": frames,
        "hits": len(hits),
        "min_hits": min_hits,
        "conf": conf,
        "passed": len(hits) >= min_hits,
        "best": max(hits, key=lambda d: d["confidence"]) if hits else None,
    }
    (preview_dir / "yolo_gate_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"[YOLO] gate summary hits={len(hits)}/{frames} passed={summary['passed']} "
          f"preview={preview_dir}", flush=True)

    # 최종 PASS/FAIL 배너를 몇 프레임 더 보내 RViz 에서 결과가 눈에 띄게
    if last_ann is not None:
        banner = last_ann.copy()
        ok = summary["passed"]
        text = "GATE PASS → PICK" if ok else "GATE FAIL"
        color = (0, 200, 0) if ok else (0, 0, 220)
        cv2.rectangle(banner, (0, banner.shape[0] - 60), (banner.shape[1], banner.shape[0]), color, -1)
        cv2.putText(banner, text, (20, banner.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3, cv2.LINE_AA)
        for _ in range(15):
            publish_yolo_viz(ros_node, image_pub, status_pub, banner, text)
            show_yolo_window(banner)
            world.step(render=True)
            _pub_clock()
            if ros_node is not None and rclpy is not None:
                rclpy.spin_once(ros_node, timeout_sec=0.0)

    return summary["best"] if summary["passed"] else None


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


def get_prim_world_bbox_center(prim_path: str) -> np.ndarray:
    """xformOp:translate 는 prim 의 로컬 원점일 뿐 메쉬의 기하학적 중심이 아니다 (실측:
    small_trash_can_body 에셋은 원점이 바닥 모서리 근처, 중심은 world 기준 약
    (+0.15,+0.15,+0.2)m 떨어져 있음). 바운딩박스 중심을 world 좌표로 직접 계산한다."""
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bbox_range = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
    center = (np.array(bbox_range.GetMin()) + np.array(bbox_range.GetMax())) / 2.0
    return center


def get_world_orientation_wxyz(prim_path: str) -> np.ndarray:
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    matrix = omni.usd.get_world_transform_matrix(prim)
    quat = matrix.ExtractRotationQuat()
    imag = quat.GetImaginary()
    return np.array([quat.GetReal(), imag[0], imag[1], imag[2]])


def rotate_vector_by_quat(q_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    w = q_wxyz[0]
    qv = np.asarray(q_wxyz[1:4], dtype=float)
    v = np.asarray(v, dtype=float)
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def create_plug_at_world_pos(trash_can_path: str, world_pos: np.ndarray, plug_name: str = "grip_plug") -> str:
    stage = omni.usd.get_context().get_stage()
    trash_can_prim = stage.GetPrimAtPath(trash_can_path)
    desired_world_pos = Gf.Vec3d(float(world_pos[0]), float(world_pos[1]), float(world_pos[2]))
    trash_can_world_matrix = omni.usd.get_world_transform_matrix(trash_can_prim)
    local_pos = trash_can_world_matrix.GetInverse().Transform(desired_world_pos)

    plug_path = f"{trash_can_path}/{plug_name}"
    if stage.GetPrimAtPath(plug_path).IsValid():
        stage.RemovePrim(plug_path)
    plug_xform = UsdGeom.Xform.Define(stage, plug_path)
    plug_xform.AddTranslateOp().Set(local_pos)
    plug_xform.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return plug_path


def set_drive_gains(stage, root_path: str):
    drive_count = 0
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        for dof_type in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, dof_type)
            if drive:
                drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                drive.GetDampingAttr().Set(DRIVE_DAMPING)
                drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                drive.GetTargetPositionAttr().Set(0.0)
                drive_count += 1
    print(f"[INFO] 관절 드라이브 강성/댐핑/목표위치(0) 설정 완료: {drive_count}개")


def relocate_trash_can(stage, prim_path: str, xy: np.ndarray):
    """스폰 후보 좌표로 쓰레기통의 x,y 만 옮기고 원래 z(낙하 높이)/rotate/scale 은 그대로 둔다."""
    prim = stage.GetPrimAtPath(prim_path)
    translate_attr = prim.GetAttribute("xformOp:translate")
    translate_attr.Set(Gf.Vec3d(float(xy[0]), float(xy[1]), TRASH_SPAWN_Z))


def _smoothstep(alpha: float) -> float:
    return alpha * alpha * (3.0 - 2.0 * alpha)


def _ramp_steps_for_distance(distance: float, max_linear_speed: float) -> int:
    raw_steps = distance / max_linear_speed / PHYSICS_DT
    return int(np.clip(round(raw_steps), MIN_INTERP_STEPS, MAX_INTERP_STEPS))


def ramp_to_joint_positions(world, robot, dof_names, arm_joint_names, target_joints_deg, ramp_steps: int):
    start_positions = robot.get_joint_positions().copy()
    target_positions = start_positions.copy()
    arm_indices = [dof_names.index(name) for name in arm_joint_names if name in dof_names]
    target_rad = np.radians(target_joints_deg)
    for idx, rad in zip(arm_indices, target_rad):
        target_positions[idx] = rad

    for step in range(ramp_steps):
        world.step(render=True)
        alpha = (step + 1) / ramp_steps
        waypoint = start_positions + _smoothstep(alpha) * (target_positions - start_positions)
        robot.apply_action(ArticulationAction(joint_positions=waypoint))
    return target_positions


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


def is_at_pose(current_position: np.ndarray, target_position: np.ndarray, tolerance: float = 0.01) -> bool:
    return bool(np.linalg.norm(np.asarray(current_position) - np.asarray(target_position)) < tolerance)


def move_to_pose(
    world, robot, rmpflow, tool0_path, target_position, target_orientation, label: str,
    max_linear_speed: float = MAX_EE_LINEAR_SPEED,
):
    start_position = get_prim_world_position(tool0_path)
    distance = float(np.linalg.norm(target_position - start_position))
    ramp_steps = _ramp_steps_for_distance(distance, max_linear_speed)
    print(f"[INFO] {label} 목표 {target_position} 로 이동 시작 (dist={distance:.3f}m)...")
    _ramp_ee_target(world, robot, rmpflow, tool0_path, target_position, target_orientation, ramp_steps)

    for step in range(MAX_APPROACH_STEPS):
        world.step(render=True)
        ee_position = get_prim_world_position(tool0_path)
        action = rmpflow.forward(
            target_end_effector_position=target_position,
            target_end_effector_orientation=target_orientation,
        )
        robot.apply_action(action)
        if is_at_pose(ee_position, target_position, POSITION_TOLERANCE):
            print(f"[INFO] {label} 도달 (settle step={step}, ee={ee_position})")
            return True
    final_pos = get_prim_world_position(tool0_path)
    print(f"[WARN] {label} 이동이 {MAX_APPROACH_STEPS} step 내에 수렴하지 못했습니다. final_ee={final_pos}")
    return False


def sync_rmpflow_base_pose(stage, rmpflow):
    """RMPflow 는 world 프레임 타겟을 robot base pose 기준으로 풀기 때문에, Nav2 주행으로
    base_link 가 이동한 뒤에는 반드시 다시 호출해서 최신 base pose 를 알려줘야 한다
    (Isaac Sim RMPflow 문서: targets 는 world 프레임, base pose 는 set_robot_base_pose 로
    등록 — 등록 안 하면 이후 모든 IK/Cartesian 계산이 스테일한 base 기준으로 계산됨)."""
    base_link_prim = stage.GetPrimAtPath(f"{ARM_USD_ROOT}/base_link")
    base_matrix = omni.usd.get_world_transform_matrix(base_link_prim)
    base_translation = base_matrix.ExtractTranslation()
    base_quat = base_matrix.ExtractRotationQuat()
    base_imag = base_quat.GetImaginary()
    base_position = np.array([base_translation[0], base_translation[1], base_translation[2]])
    base_orientation = np.array([base_quat.GetReal(), base_imag[0], base_imag[1], base_imag[2]])
    rmpflow.rmp_flow.set_robot_base_pose(robot_position=base_position, robot_orientation=base_orientation)
    return base_position, base_orientation


def hold_pose(world, robot, rmpflow, target_position, target_orientation, steps: int):
    for _ in range(steps):
        world.step(render=True)
        action = rmpflow.forward(
            target_end_effector_position=target_position,
            target_end_effector_orientation=target_orientation,
        )
        robot.apply_action(action)


def dump_into_big_trash(world, robot, dof_names):
    """big_trash 앞에 도착했다는 전제로, j1 을 절대 DUMP_J1_DEG 로, j6 을 현재값에서
    DUMP_J6_ROTATE_DEG 만큼 상대 회전시켜 쓰레기통을 기울여 비운다. tuck 이후 유지되던
    나머지 관절(2,3,4,5)은 그대로 둔다 — tuck_targets_deg 계산과 동일한 패턴."""
    current_positions_rad = robot.get_joint_positions()
    dump_targets_deg = []
    for name in ARM_JOINT_NAMES:
        idx = dof_names.index(name)
        if name == "joint_1":
            dump_targets_deg.append(DUMP_J1_DEG)
        elif name == "joint_6":
            dump_targets_deg.append(float(np.degrees(current_positions_rad[idx])) + DUMP_J6_ROTATE_DEG)
        else:
            dump_targets_deg.append(float(np.degrees(current_positions_rad[idx])))
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, dump_targets_deg, DUMP_RAMP_STEPS)
    print(f"[INFO] big_trash 덤프 자세 완료 (j1={DUMP_J1_DEG}deg, j6 += {DUMP_J6_ROTATE_DEG}deg).")


def restore_upright_after_dump(world, robot, dof_names):
    """dump_into_big_trash() 에서 j6 을 DUMP_J6_ROTATE_DEG 만큼 돌려 쓰레기통을 기울였던
    걸, 되돌아가기 전에 그만큼 반대로 돌려서 다시 위를 향하게(정상 방향) 되돌린다.
    j1 을 포함한 나머지 관절은 그대로 유지 — dump_targets_deg 계산과 동일한 패턴."""
    current_positions_rad = robot.get_joint_positions()
    restore_targets_deg = []
    for name in ARM_JOINT_NAMES:
        idx = dof_names.index(name)
        if name == "joint_6":
            restore_targets_deg.append(float(np.degrees(current_positions_rad[idx])) - DUMP_J6_ROTATE_DEG)
        else:
            restore_targets_deg.append(float(np.degrees(current_positions_rad[idx])))
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, restore_targets_deg, DUMP_RAMP_STEPS)
    print(f"[INFO] 쓰레기통 다시 위로 복귀 완료 (j6 -= {DUMP_J6_ROTATE_DEG}deg).")


def wrap_pi(angle: float) -> float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def rotate_2d(v: np.ndarray, theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def pick_closest_entry_point(trash_xy: np.ndarray, spawn_xy: np.ndarray):
    """쓰레기통(사각형)-chassis 를 잇는 원래 진입선(OFFSET_TRASH_FROM_CHASSIS)을 90도씩
    돌리면 대칭인 4개의 진입점이 나온다(사각 쓰레기통의 네 변). TARGET_JOINTS_DEG 는
    chassis 의 '몸체 기준' 상대각도라 world yaw 를 어느 쪽으로 돌리든(전체를 강체로
    회전시키는 것과 같으므로) 그대로 유효하다 — 그래서 chassis_goal_yaw 도 회전량만큼
    같이 돌려준다. 로봇 스폰 지점에서 가장 가까운 후보를 선택해 이동 거리를 줄인다."""
    best = None
    for k in range(4):
        theta_k = k * (np.pi / 2.0)
        offset_k = rotate_2d(OFFSET_TRASH_FROM_CHASSIS, theta_k)
        chassis_goal_xy_k = trash_xy - offset_k
        dist = float(np.linalg.norm(chassis_goal_xy_k - spawn_xy))
        if best is None or dist < best[2]:
            best = (chassis_goal_xy_k, theta_k, dist)
    return best[0], best[1]


def get_chassis_yaw(prim_path: str) -> float:
    w, x, y, z = get_world_orientation_wxyz(prim_path)
    return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def drive_straight_open_loop(world, cmd_pub, distance: float, chassis_path: str,
                              speed: float = FINAL_APPROACH_SPEED, ramp_time: float = FINAL_APPROACH_RAMP_TIME,
                              drift_kp: float = FINAL_APPROACH_DRIFT_KP, drift_kd: float = FINAL_APPROACH_DRIFT_KD,
                              drift_w_max: float = FINAL_APPROACH_DRIFT_W_MAX,
                              drift_max_w_step: float = FINAL_APPROACH_DRIFT_MAX_W_STEP,
                              reverse: bool = False):
    """직진(reverse=True 면 같은 heading 을 유지한 채 뒤로 후진). 회전(큰 방향 전환)은 이
    함수 호출 전에 rotate_in_place() 로 이미 끝낸 상태를 전제로 하고, 여기서는 그 heading
    을 유지하며 아주 낮은 게인(drift_kp/kd)으로만 미세 보정한다 — Nova Carter 뒷쪽 캐스터가
    이동거리에 비례해서 서서히 드리프트를 만들기 때문(속도와 무관, ramp 로는 못 없앰).
    오차가 이미 작은 상태에서 낮은 게인만 쓰므로, 코너 진입처럼 오차가 큰 상황에서
    진동났던 예전 heading-hold 와 달리 안전하다. 시작 ramp_time 동안 속도도 0->speed 로
    부드럽게 올린다(시간 기반 ramp). distance 는 항상 양수(이동 거리 크기)로 준다."""
    sign = -1.0 if reverse else 1.0
    start_xy = get_prim_world_position(chassis_path)[:2]
    target_yaw = get_chassis_yaw(chassis_path)  # rotate_in_place 로 이미 맞춘 heading 을 유지
    forward = np.array([np.cos(target_yaw), np.sin(target_yaw)])
    max_steps = int(round(distance / speed / PHYSICS_DT)) + 100
    ramp_steps = max(1, int(round(ramp_time / PHYSICS_DT)))
    prev_yaw = target_yaw
    w_applied = 0.0
    for step in range(max_steps):
        world.step(render=True)
        pos_xy = get_prim_world_position(chassis_path)[:2]
        progress = float(np.dot(pos_xy - start_xy, forward)) * sign
        alpha = min(1.0, (step + 1) / ramp_steps)
        current_speed = sign * speed * _smoothstep(alpha)
        yaw = get_chassis_yaw(chassis_path)
        yaw_err = wrap_pi(yaw - target_yaw)
        yaw_rate = wrap_pi(yaw - prev_yaw) / PHYSICS_DT
        prev_yaw = yaw
        w_target = float(np.clip(-(drift_kp * yaw_err + drift_kd * yaw_rate), -drift_w_max, drift_w_max))
        w_applied += float(np.clip(w_target - w_applied, -drift_max_w_step, drift_max_w_step))
        tw = Twist()
        tw.linear.x = float(current_speed)
        tw.angular.z = w_applied
        cmd_pub.publish(tw)
        if progress >= distance:
            break
    cmd_pub.publish(Twist())
    for _ in range(FINAL_MOVE_SETTLE_STEPS):
        world.step(render=True)


def rotate_in_place(world, cmd_pub, target_yaw: float, kp: float, kd: float, w_max: float,
                     max_w_step: float, tolerance_rad: float, chassis_path: str, max_steps: int = 600):
    """yaw_err 뿐 아니라 w_applied 도 같이 작아야 종료하는 조건은, 잔여 진동/노이즈 때문에
    둘이 동시에 만족되기 어려워서 max_steps(600=10초)를 그대로 다 채우는 경우가 있었다
    (PICK 진입에서 회전이 2번이라 최악의 경우 20초 — 실측으로 확인됨). w_applied 조건
    대신 "yaw_err 가 tolerance 안에 N 스텝 연속 유지"로 디바운스해서 노이즈에 안정적으로
    빨리 종료하도록 고쳤다."""
    prev_yaw = get_chassis_yaw(chassis_path)
    w_applied = 0.0
    settled_steps = 0
    for _ in range(max_steps):
        world.step(render=True)
        yaw = get_chassis_yaw(chassis_path)
        yaw_err = wrap_pi(target_yaw - yaw)
        yaw_rate = wrap_pi(yaw - prev_yaw) / PHYSICS_DT
        prev_yaw = yaw
        if abs(yaw_err) < tolerance_rad:
            settled_steps += 1
            if settled_steps >= 5:
                break
        else:
            settled_steps = 0
        w_target = float(np.clip(kp * yaw_err - kd * yaw_rate, -w_max, w_max))
        w_applied += float(np.clip(w_target - w_applied, -max_w_step, max_w_step))
        tw = Twist()
        tw.angular.z = w_applied
        cmd_pub.publish(tw)
    cmd_pub.publish(Twist())
    for _ in range(FINAL_MOVE_SETTLE_STEPS):
        world.step(render=True)


def run_nav_leg(world, ros_node, clock_pub, goal_pub, cmd_vel_pub, pick_state,
                 standoff_xy, standoff_yaw, chassis_goal_xy, chassis_goal_yaw, label: str) -> bool:
    """Nav2 목표(standoff)를 퍼블리시하며 /start_pick 대기 → 도착 후 실제 위치 기준으로
    회전→직진→회전 해서 chassis_goal_xy/yaw 까지 마저 접근한다. 소형 쓰레기통 파지 구간과
    big_trash 구간(+ 앞으로 추가될 원위치 복귀/도킹 복귀)이 전부 이 함수를 재사용한다.
    simulation_app 이 종료되면 False, 정상 도착하면 True 를 반환한다."""
    # 커맨더가 이전 구간 완료 시 안정성을 위해 /start_pick 을 10번 연속 재발행하는데,
    # 그 사이(파지/들기/tuck 등) 이 스크립트가 rclpy.spin_once 를 전혀 안 불러서 그
    # 메시지들이 큐에 쌓여있다가, 다음 구간이 spin 을 다시 시작하자마자 즉시(도착도
    # 안 했는데) 소비되어 "즉시 도착"으로 오인되는 문제가 있었다(실측으로 확인됨).
    # 새 구간을 시작하기 전에 큐에 남아있는 이전 구간의 메시지를 먼저 비운다.
    for _ in range(30):
        rclpy.spin_once(ros_node, timeout_sec=0.0)
    pick_state["start"] = False

    goal_msg = PoseStamped()
    goal_msg.header.frame_id = "map"
    goal_msg.pose.position.x = float(standoff_xy[0])
    goal_msg.pose.position.y = float(standoff_xy[1])
    goal_msg.pose.orientation.z = float(np.sin(standoff_yaw / 2.0))
    goal_msg.pose.orientation.w = float(np.cos(standoff_yaw / 2.0))
    pick_state["start"] = False

    print(f"[NAV:{label}] Nav2 목표(standoff)={standoff_xy.tolist()}, "
          f"yaw={np.degrees(standoff_yaw):.1f}deg 발행 시작")
    while simulation_app.is_running() and not pick_state["start"]:
        world.step(render=True)
        t = float(world.current_time)
        cmsg = Clock()
        cmsg.clock.sec = int(t)
        cmsg.clock.nanosec = int(round((t - int(t)) * 1e9))
        clock_pub.publish(cmsg)
        goal_msg.header.stamp = cmsg.clock
        goal_pub.publish(goal_msg)
        rclpy.spin_once(ros_node, timeout_sec=0.0)

    if not simulation_app.is_running():
        return False

    print(f"[NAV:{label}] '{START_PICK_TOPIC}'=True 수신 → standoff에서 최종 접근 시작")
    current_chassis_xy = get_prim_world_position(ARTICULATION_ROOT_PATH)[:2]
    to_goal = chassis_goal_xy - current_chassis_xy
    entry_distance = float(np.linalg.norm(to_goal))
    entry_yaw = float(np.arctan2(to_goal[1], to_goal[0]))
    print(f"[APPROACH:{label}] 실제 도착 위치={current_chassis_xy.tolist()} → 목표까지 "
          f"거리={entry_distance:.3f}m, 방향={np.degrees(entry_yaw):.1f}deg 로 진입")
    rotate_in_place(
        world, cmd_vel_pub, entry_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
        FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, ARTICULATION_ROOT_PATH,
    )
    drive_straight_open_loop(world, cmd_vel_pub, entry_distance + PRE_ROTATE_NUDGE_DISTANCE,
                              ARTICULATION_ROOT_PATH, FINAL_APPROACH_SPEED)
    print(f"[APPROACH:{label}] {entry_distance + PRE_ROTATE_NUDGE_DISTANCE:.2f}m 직진 완료 → "
          f"chassis_goal_yaw={np.degrees(chassis_goal_yaw):.0f}deg 으로 제자리 회전")
    rotate_in_place(
        world, cmd_vel_pub, chassis_goal_yaw, FINAL_ROTATE_KP, FINAL_ROTATE_KD, FINAL_ROTATE_W_MAX,
        FINAL_ROTATE_MAX_W_STEP, FINAL_ROTATE_TOLERANCE_RAD, ARTICULATION_ROOT_PATH,
    )
    final_chassis_xy = get_prim_world_position(ARTICULATION_ROOT_PATH)[:2]
    print(f"[APPROACH:{label}] 회전 완료. chassis 실제 위치={final_chassis_xy.tolist()} "
          f"(목표={chassis_goal_xy.tolist()}, 오차={np.linalg.norm(final_chassis_xy-chassis_goal_xy):.3f}m)")

    if FINAL_NUDGE_DISTANCE > 0.0:
        drive_straight_open_loop(world, cmd_vel_pub, FINAL_NUDGE_DISTANCE, ARTICULATION_ROOT_PATH, FINAL_APPROACH_SPEED)
        nudged_chassis_xy = get_prim_world_position(ARTICULATION_ROOT_PATH)[:2]
        print(f"[APPROACH:{label}] {FINAL_NUDGE_DISTANCE:.3f}m 추가 전진 완료. chassis={nudged_chassis_xy.tolist()}")

    return True


def main():
    trash_xy = np.array(TRASH_SPAWN_FIXED)
    spawn_xy = np.array([CARTER_START_POSE["x"], CARTER_START_POSE["y"]])
    # 쓰레기통은 사각형이라, 원래 진입선(OFFSET_TRASH_FROM_CHASSIS)을 90도씩 돌리면
    # 대칭인 진입점 4개(네 변)가 나온다. 그중 로봇 스폰 지점에서 가장 가까운 진입점을
    # 선택해 이동 거리를 줄인다 (TARGET_JOINTS_DEG 는 chassis 몸체 기준 상대각도라
    # world yaw 를 몇 도로 돌리든 그대로 유효 — chassis_goal_yaw 도 같이 회전시켜 둠).
    chassis_goal_xy, chassis_goal_yaw = pick_closest_entry_point(trash_xy, spawn_xy)

    # Nav2 목표는 chassis_goal_xy 가 아니라, 그보다 FINAL_APPROACH_DISTANCE 만큼 뒤로
    # 뺀 standoff 지점(코스트맵 inflation 밖, 안전하게 도달 가능) + 그 접근방향을 보는 yaw.
    # 도착 후 이 스크립트가 (회전 1번 -> 직진 1번으로) chassis_goal_xy 까지 마저 접근하고,
    # 마지막에 chassis_goal_yaw 로 제자리 회전한다 (differential-drive 라 방향 안 틀고
    # 대각선 이동은 불가능하므로).
    approach_dir = rotate_2d(OFFSET_TRASH_FROM_CHASSIS / np.linalg.norm(OFFSET_TRASH_FROM_CHASSIS),
                              chassis_goal_yaw)  # chassis_goal->trash 방향 (선택된 진입점 기준)
    standoff_xy = chassis_goal_xy - approach_dir * FINAL_APPROACH_DISTANCE
    standoff_yaw = float(np.arctan2(approach_dir[1], approach_dir[0]))
    print(f"[SPAWN] 쓰레기통 스폰 좌표 = {trash_xy.tolist()}")
    print(f"[NAV] 선택된 진입점 yaw={np.degrees(chassis_goal_yaw):.0f}deg (스폰 지점 기준 최근접) "
          f"/ chassis_goal(최종)={chassis_goal_xy.tolist()} / "
          f"Nav2 목표(standoff)={standoff_xy.tolist()}, yaw={np.degrees(standoff_yaw):.1f}deg")

    omni.usd.get_context().open_stage(USD_PATH)
    for _ in range(30):
        simulation_app.update()
    stage = omni.usd.get_context().get_stage()

    # 병원 환경 병합 (/Root 의 531개 프롭이 /World 에 그대로 얹힌다. defaultPrim="Root"
    # 는 identity transform 이라 좌표계 보정 불필요 — modified_hospital.usd 자체가
    # move_tash_can.usd 와 동일한 world frame 을 쓴다).
    # --skip-nav 로컬 검증에서는 병원/원격 스폰을 건너뛰어 로딩을 빠르게 한다.
    if not _ARGS.skip_nav:
        world_prim = stage.GetPrimAtPath("/World")
        world_prim.GetReferences().AddReference(HOSPITAL_USD)
        for _ in range(60):
            simulation_app.update()
        print(f"[LOAD] hospital environment={HOSPITAL_USD}")
        relocate_trash_can(stage, TRASH_CAN_PRIM_PATH, trash_xy)
    else:
        print("[LOAD] --skip-nav: hospital merge/trash relocate 생략 (원본 move_tash_can 기하)", flush=True)
        # hospital 조명이 없으므로 관측용 보조광을 미리 둔다 (P8). intensity 는
        # 과노출을 피하도록 낮게 유지.
        ensure_min_scene_light(stage, intensity=400.0)

    # ── YOLO / RealSense: World.reset() 이전에 attach + nested RB strip (P6/P7) ──
    cam_path = ensure_realsense_on_link5(stage)

    world = World(physics_dt=PHYSICS_DT, stage_units_in_meters=1.0)
    set_drive_gains(stage, ARM_USD_ROOT)

    ee_prim_path = find_prim_path_by_name(ARM_USD_ROOT, EE_LINK_NAME)
    if ee_prim_path is None:
        raise RuntimeError(f"'{EE_LINK_NAME}' 링크를 {ARM_USD_ROOT} 하위에서 찾지 못했습니다.")
    tool0_path = find_prim_path_by_name(ARM_USD_ROOT, "tool0")
    if tool0_path is None:
        raise RuntimeError(f"'tool0' 프레임을 {ARM_USD_ROOT} 하위에서 찾지 못했습니다.")

    gripper_prim = stage.GetPrimAtPath(SURFACE_GRIPPER_PATH)
    if not gripper_prim.IsValid():
        raise RuntimeError(f"Surface Gripper 프림이 없습니다: {SURFACE_GRIPPER_PATH}")

    robot = world.scene.add(
        SingleManipulator(
            prim_path=ARTICULATION_ROOT_PATH,
            name="m0609_mobile_robot",
            end_effector_prim_path=ee_prim_path,
            gripper=None,
        )
    )
    camera = make_realsense_camera(cam_path)

    world.reset()
    # reset 후 PhysX 가 reference RB 를 다시 살리는 경우가 있어 한 번 더 strip (P7)
    n_strip2 = _strip_nested_rigid_bodies(stage, REALSENSE_PRIM_PATH)
    if n_strip2:
        print(f"[YOLO] post-reset nested RB/collision re-strip ops={n_strip2}", flush=True)
        for _ in range(10):
            world.step(render=True)
    robot.initialize()
    warm_realsense_camera(camera, world, stage=stage)

    dof_names = list(robot.dof_names)
    default_positions = robot.get_joint_positions()
    for name in ARM_JOINT_NAMES:
        if name in dof_names:
            default_positions[dof_names.index(name)] = 0.0
    robot.set_joint_positions(default_positions)
    for _ in range(10):
        world.step(render=True)

    rmpflow = RMPFlowController(
        name="trash_can_nav_pick_cspace_controller",
        robot_articulation=robot,
        urdf_path=NO_GRIPPER_URDF_PATH,
    )

    sync_rmpflow_base_pose(stage, rmpflow)
    # move_tash_can.usd 에 이미 있는 /World/GroundPlane(z=0)을 RMPflow 장애물로 등록 —
    # 이거 없으면 RMPflow IK 가 바닥을 전혀 모르는 채로 목표만 쫓아가서 팔이 바닥을
    # 뚫고 지나갈 수 있다. static 장애물이라 Nav2 이동 후 sync_rmpflow_base_pose() 를
    # 다시 부를 때마다 자동으로 robot-base 기준 상대위치가 갱신된다.
    rmpflow.add_ground_plane(prim_path="/World/GroundPlane", z_position=0.0)

    surface_gripper = SurfaceGripper(
        end_effector_prim_path=ee_prim_path,
        surface_gripper_path=SURFACE_GRIPPER_PATH,
    )
    surface_gripper.initialize()
    print(f"[INFO] Surface Gripper 초기화 완료: {SURFACE_GRIPPER_PATH}")

    # 쓰레기통이 새 스폰 좌표에서 중력으로 바닥에 안착할 시간을 준다.
    for _ in range(SETTLE_STEPS):
        world.step(render=True)

    weights = resolve_yolo_weights(_ARGS.weights)
    yolo_model = YOLO(weights)
    yolo_fallback = None
    if Path(weights).resolve() != Path(_FALLBACK_WEIGHTS).resolve() and Path(_FALLBACK_WEIGHTS).is_file():
        yolo_fallback = YOLO(_FALLBACK_WEIGHTS)
    print(f"[YOLO] weights={weights} fallback={'yes' if yolo_fallback else 'no'} "
          f"conf={_ARGS.yolo_conf}", flush=True)

    # ── ROS2 : /clock 발행 + Nav2 목표 퍼블리시 + /start_pick 대기 ──
    # --skip-nav 이면 Nav2 handoff 없이 바로 YOLO 게이트+파지로 간다.
    global rclpy, Bool, PoseStamped, Twist, Clock
    if not _ARGS.skip_nav:
        # 터미널에서 ros_set(/opt/ros/humble) 을 source 한 상태여도 Isaac py3.11
        # 번들 rclpy 를 쓰도록 경로를 정리한다 (P5).
        prepare_isaac_ros_python_env()
        import rclpy as _rclpy
        from std_msgs.msg import Bool as _Bool
        from geometry_msgs.msg import PoseStamped as _PoseStamped, Twist as _Twist
        from rosgraph_msgs.msg import Clock as _Clock
        rclpy, Bool, PoseStamped, Twist, Clock = _rclpy, _Bool, _PoseStamped, _Twist, _Clock
        print(f"[ROS] rclpy loaded from {_rclpy.__file__}", flush=True)
        rclpy.init()
    ros_node = None
    clock_pub = goal_pub = cmd_vel_pub = None
    yolo_image_pub = yolo_status_pub = None
    pick_state = {"start": False}

    if not _ARGS.skip_nav:
        ros_node = rclpy.create_node("trash_can_nav_pick_controller_v2")
        clock_pub = ros_node.create_publisher(Clock, "/clock", 10)
        goal_pub = ros_node.create_publisher(PoseStamped, NAV_GOAL_TOPIC, 10)
        cmd_vel_pub = ros_node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        from sensor_msgs.msg import Image as RosImage
        from std_msgs.msg import String as RosString
        yolo_image_pub = ros_node.create_publisher(RosImage, YOLO_IMAGE_TOPIC, 10)
        yolo_status_pub = ros_node.create_publisher(RosString, YOLO_STATUS_TOPIC, 10)
        ros_node.create_subscription(
            Bool, START_PICK_TOPIC, lambda m: pick_state.__setitem__("start", bool(m.data)), 10)

        print(f"[ROS] '{NAV_GOAL_TOPIC}' 발행 + '/clock' 발행 + '{START_PICK_TOPIC}' 구독 시작")
        print(f"[ROS] YOLO viz: Image '{YOLO_IMAGE_TOPIC}' + String '{YOLO_STATUS_TOPIC}' "
              f"(RViz Add → By topic → Image)", flush=True)
        print("[RUN] 다른 터미널에서 Nav2 bringup 후 'ros2 run commander trash_can_nav_pick_mission' 실행하세요.")

        if not run_nav_leg(world, ros_node, clock_pub, goal_pub, cmd_vel_pub, pick_state,
                           standoff_xy, standoff_yaw, chassis_goal_xy, chassis_goal_yaw, "PICK"):
            ros_node.destroy_node()
            rclpy.shutdown()
            return

        # Nav2 주행으로 base_link 가 이동했으니, RMPflow 에 최신 base pose 를 다시 알려준다
        # (이거 없이는 이후 move_to_pose/보정 계산이 전부 스폰 시점의 스테일한 base 기준으로 풀림).
        sync_rmpflow_base_pose(stage, rmpflow)
    else:
        print("[YOLO] --skip-nav: Nav2 생략, 현재 기하에서 YOLO 게이트+파지 검증", flush=True)
        # skip-nav 에서는 병원 스폰 상대기하가 깨질 수 있어, 쓰레기통을 chassis 기준
        # OFFSET_TRASH_FROM_CHASSIS 위치로 재배치해 TARGET_JOINTS_DEG 가 유효하게 만든다.
        chassis_xy = get_prim_world_position(ARTICULATION_ROOT_PATH)[:2]
        local_trash_xy = chassis_xy + OFFSET_TRASH_FROM_CHASSIS
        relocate_trash_can(stage, TRASH_CAN_PRIM_PATH, local_trash_xy)
        for _ in range(SETTLE_STEPS):
            world.step(render=True)
        print(f"[YOLO] skip-nav trash relocated to {local_trash_xy.tolist()} (validated offset)", flush=True)
        sync_rmpflow_base_pose(stage, rmpflow)
        if not _ARGS.headless:
            print(f"[YOLO] OpenCV window '{YOLO_WINDOW_NAME}' will show detection boxes", flush=True)

    # ── v2: 파지 전 YOLO 실시간 인식 게이트 ──
    det = yolo_realtime_gate(
        world, robot, dof_names, camera, yolo_model, yolo_fallback,
        frames=_ARGS.yolo_frames, conf=_ARGS.yolo_conf, min_hits=_ARGS.yolo_min_hits,
        preview_dir=_YOLO_PREVIEW_DIR,
        ros_node=ros_node, image_pub=yolo_image_pub, status_pub=yolo_status_pub,
        clock_pub=clock_pub,
    )
    if det is None and not _ARGS.force_pick:
        print("[YOLO][ERROR] 쓰레기통 미검출 — 파지 중단. "
              "--force-pick 또는 --yolo-conf 조정 / preview 폴더 확인.", flush=True)
        if ros_node is not None:
            ros_node.destroy_node()
            rclpy.shutdown()
        return
    if det is None and _ARGS.force_pick:
        print("[YOLO][WARN] --force-pick: 미검출이지만 기존 4_ 파지 시퀀스 강제 진행", flush=True)
    else:
        print(f"[YOLO] gate PASS → 기존 4_ 파지 시퀀스 시작 "
              f"({det['class_name']} conf={det['confidence']:.3f})", flush=True)

    # ── 이하 3_mobile_manipulator_trash_can_pick_test.py 와 동일한 파지 시퀀스 ──
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, TARGET_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        world.step(render=True)
    grasp_position = get_prim_world_position(tool0_path)
    grasp_orientation = get_world_orientation_wxyz(tool0_path)
    print(f"[INFO] 목표 관절 도달. tool0 world pos={grasp_position}")

    move_direction_world = rotate_vector_by_quat(grasp_orientation, np.array([0.0, 0.0, 1.0]))
    move_direction_world /= np.linalg.norm(move_direction_world)

    # ── Nav2 도착 오차 보정: TARGET_JOINTS_DEG 는 "검증된 상대기하"를 가정한 고정 자세라,
    # Nav2 가 그 기하에서 살짝 벗어난 곳에 서면 creep 전진(깊이 방향만 흡수)만으론 못 잡는다.
    # 실측 쓰레기통 위치 기준으로 전진축과 수직인 성분(좌우/높이 오차)만 RMPflow IK 로
    # 한 번 보정하고, 깊이 방향은 기존 creep 로직에 맡긴다. ──
    trash_can_position_now = get_prim_world_bbox_center(TRASH_CAN_PRIM_PATH)
    to_trash = trash_can_position_now - grasp_position
    depth_component = float(np.dot(to_trash, move_direction_world))
    lateral_vec = to_trash - depth_component * move_direction_world
    lateral_error = float(np.linalg.norm(lateral_vec))
    print(f"[INFO] 실측 쓰레기통 위치={trash_can_position_now}, 전진축 수직 오차={lateral_error:.3f}m")

    if LATERAL_CORRECTION_MIN < lateral_error <= LATERAL_CORRECTION_MAX:
        corrected_target = grasp_position + lateral_vec
        move_to_pose(world, robot, rmpflow, tool0_path, corrected_target, grasp_orientation,
                     "실측 기반 좌우/높이 보정")
        grasp_position = get_prim_world_position(tool0_path)
    elif lateral_error > LATERAL_CORRECTION_MAX:
        print(f"[WARN] 전진축 수직 오차({lateral_error:.3f}m)가 너무 커서 보정 생략 "
              f"(Nav2 도착 위치 자체가 크게 벗어났을 가능성 — 그대로 creep 시도).")

    current_target = grasp_position.copy()
    gripped_ok = False
    for creep_step in range(CREEP_MAX_STEPS):
        current_target = current_target + move_direction_world * CREEP_STEP_SIZE
        for _ in range(CREEP_SETTLE_STEPS):
            world.step(render=True)
            action = rmpflow.forward(
                target_end_effector_position=current_target,
                target_end_effector_orientation=grasp_orientation,
            )
            robot.apply_action(action)
        surface_gripper.close()
        if surface_gripper.is_closed():
            gripped_ok = True
            traveled = CREEP_STEP_SIZE * (creep_step + 1)
            print(f"[INFO] 전진 {creep_step + 1}스텝(누적 {traveled:.3f}m)에서 파지 성공.")
            break

    grasp_position = current_target
    plug_path = create_plug_at_world_pos(TRASH_CAN_PRIM_PATH, get_prim_world_position(tool0_path))

    if gripped_ok:
        print("[CHECKPOINT] Surface Gripper 상태: CLOSED.")
    else:
        max_travel = CREEP_STEP_SIZE * CREEP_MAX_STEPS
        print(f"[CHECKPOINT] 최대 {max_travel:.3f}m 전진해도 파지 실패 "
              f"(Nav2 도착 위치가 검증된 상대기하에서 너무 벗어났을 가능성 — 상단 docstring 참고).")

    lift_target = grasp_position + LIFT_OFFSET
    move_to_pose(world, robot, rmpflow, tool0_path, lift_target, grasp_orientation, "쓰레기통 들어올리기")
    hold_pose(world, robot, rmpflow, lift_target, grasp_orientation, GRASP_HOLD_STEPS)

    ee_pos_after_lift = get_prim_world_position(tool0_path)
    plug_pos_after_lift = get_prim_world_position(plug_path)
    plug_gripper_gap = float(np.linalg.norm(plug_pos_after_lift - ee_pos_after_lift))
    print(f"[RESULT] 들어올리기 후 그리퍼-plug 간격={plug_gripper_gap:.4f}m")
    if plug_gripper_gap < 0.03:
        print("[RESULT] 파지 성공: 쓰레기통이 그리퍼에 단단히 붙어 들어올려졌습니다.")
    else:
        print("[RESULT] 파지 실패로 보입니다: 그리퍼-plug 간격이 큽니다.")

    # 주행 전, j1만 추가로 회전시켜 쓰레기통을 몸 쪽으로 당겨 접는다.
    current_positions_rad = robot.get_joint_positions()
    tuck_targets_deg = []
    for name in ARM_JOINT_NAMES:
        idx = dof_names.index(name)
        if name == "joint_1":
            tuck_targets_deg.append(TUCK_J1_DEG)
        else:
            tuck_targets_deg.append(float(np.degrees(current_positions_rad[idx])))
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, tuck_targets_deg, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        world.step(render=True)
    print(f"[INFO] j1 tuck(-> {TUCK_J1_DEG}deg) 완료.")

    if _ARGS.pick_only or _ARGS.skip_nav:
        print("[YOLO] --pick-only/--skip-nav: DUMP/RETURN/DOCK 생략하고 종료", flush=True)
        if ros_node is not None:
            while simulation_app.is_running() and not _ARGS.headless:
                world.step(render=True)
                rclpy.spin_once(ros_node, timeout_sec=0.0)
            ros_node.destroy_node()
            rclpy.shutdown()
        elif not _ARGS.headless:
            while simulation_app.is_running():
                world.step(render=True)
        return

    # ── 두 번째 구간: big_trash 로 이동 → 덤프. 4-진입점 계산 없이 항상 +Y 쪽에서
    # -Y(big_trash 방향)를 보고 진입 (씬 상 거의 직선으로 갈 수 있고, +Y 쪽이 벽에서
    # 1.55m 로 제일 트여있음 — -Y 쪽은 0m=완전히 막힘, 맵 자동 스캔으로 확인). ──
    big_trash_face_dir = -BIG_TRASH_APPROACH_DIR  # standoff/goal 공통: big_trash 를 바라보는 방향
    big_trash_yaw = float(np.arctan2(big_trash_face_dir[1], big_trash_face_dir[0]))
    big_trash_standoff_xy = BIG_TRASH_POSITION_XY + BIG_TRASH_APPROACH_DIR * BIG_TRASH_STANDOFF_DISTANCE
    big_trash_goal_xy = BIG_TRASH_POSITION_XY + BIG_TRASH_APPROACH_DIR * BIG_TRASH_FINAL_DISTANCE

    if not run_nav_leg(world, ros_node, clock_pub, goal_pub, cmd_vel_pub, pick_state,
                       big_trash_standoff_xy, big_trash_yaw, big_trash_goal_xy, big_trash_yaw, "DUMP"):
        ros_node.destroy_node()
        rclpy.shutdown()
        return

    # dump_into_big_trash() 는 순수 조인트 공간 이동(TARGET_JOINTS_DEG/tuck 과 동일 방식)
    # 이라 RMPflow 를 안 거치므로 sync_rmpflow_base_pose() 재호출이 필요 없다.
    dump_into_big_trash(world, robot, dof_names)

    # 비운 뒤 기울어져 있던 쓰레기통을 다시 위를 향하게 되돌린다 (뒤로 빠지거나 주행하는
    # 동안 기울어진 채로 있지 않도록, 후진/RETURN 전에 먼저 처리).
    restore_upright_after_dump(world, robot, dof_names)

    # big_trash 바로 앞에서 곧바로 Nav2(RETURN)를 켜면 경로계획/코스트맵이 애매해질 수
    # 있으니, 먼저 한 발 물러난 뒤에 다음 구간을 시작한다 (heading 유지, 회전 없음).
    drive_straight_open_loop(world, cmd_vel_pub, POST_DUMP_BACKUP_DISTANCE, ARTICULATION_ROOT_PATH,
                              FINAL_APPROACH_SPEED, reverse=True)
    print(f"[APPROACH:DUMP] {POST_DUMP_BACKUP_DISTANCE:.2f}m 후진 완료 → RETURN 구간 시작")

    # ── 세 번째 구간: 원위치로 복귀해서 쓰레기통을 내려놓는다. 파지 때 계산해둔
    # chassis_goal_xy/chassis_goal_yaw/standoff_xy/standoff_yaw 를 그대로 재사용한다
    # (같은 자리로 돌아가는 것이므로 새로 계산할 필요 없음). ──
    if not run_nav_leg(world, ros_node, clock_pub, goal_pub, cmd_vel_pub, pick_state,
                       standoff_xy, standoff_yaw, chassis_goal_xy, chassis_goal_yaw, "RETURN"):
        ros_node.destroy_node()
        rclpy.shutdown()
        return

    # 복귀 이동으로 base 가 다시 움직였으니 RMPflow 에 최신 base pose 를 알려준다.
    sync_rmpflow_base_pose(stage, rmpflow)

    # dump 로 뒤틀어뒀던 손목(j6 180도 회전 상태)에서 곧바로 RMPflow Cartesian IK 로
    # grasp_position/orientation 을 향해 큰 방향전환을 시키면, 팔이 예측 못한 경로로
    # 돌면서 carter 차체와 부딪힐 위험이 있다(차체는 RMPflow 장애물로 등록 안 돼 있음).
    # 그래서 먼저 검증된 TARGET_JOINTS_DEG(파지 때와 동일한 조인트 공간 자세)로 안전하게
    # 복귀한 뒤, 그 다음에만 Cartesian 이동으로 넘어간다.
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, TARGET_JOINTS_DEG, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        world.step(render=True)

    # 파지 성공 시점의 tool0 world pose(grasp_position/orientation)로 다시 내려가 그
    # 자리에 내려놓는다.
    move_to_pose(world, robot, rmpflow, tool0_path, grasp_position, grasp_orientation, "쓰레기통 원위치 내려놓기")
    hold_pose(world, robot, rmpflow, grasp_position, grasp_orientation, GRASP_HOLD_STEPS)

    surface_gripper.open()
    for _ in range(GRASP_HOLD_STEPS):
        world.step(render=True)
    print(f"[INFO] Surface Gripper 개방 완료 (is_closed={surface_gripper.is_closed()}).")

    # 내려놓은 뒤 팔을 위로 빼서 쓰레기통과의 충돌을 피한다.
    retract_target = grasp_position + LIFT_OFFSET
    move_to_pose(world, robot, rmpflow, tool0_path, retract_target, grasp_orientation, "내려놓은 후 팔 후퇴")
    hold_pose(world, robot, rmpflow, retract_target, grasp_orientation, GRASP_HOLD_STEPS)

    trash_can_final_xy = get_prim_world_position(TRASH_CAN_PRIM_PATH)
    print(f"[RESULT] 쓰레기통 원위치 복귀 완료. 최종 위치={trash_can_final_xy.tolist()} "
          f"(원래 스폰={trash_xy.tolist()})")

    # 쓰레기통을 내려놨으니 팔을 홈 자세(전부 0도)로 되돌려 안전하게 주행할 수 있게 한다.
    ramp_to_joint_positions(world, robot, dof_names, ARM_JOINT_NAMES, [0.0] * 6, JOINT_RAMP_STEPS)
    for _ in range(GRASP_HOLD_STEPS):
        world.step(render=True)
    print("[INFO] 팔 홈 자세(0deg) 복귀 완료.")

    # ── 네 번째(마지막) 구간: docking_station_02(CARTER_START_POSE)로 복귀. 물체를
    # 다루는 게 아니라 그냥 원래 스폰 지점으로 주차하는 거라 장애물/코스트맵 여유를
    # 따로 둘 필요가 없다 — standoff 를 목표 지점과 동일하게 둬서 Nav2 가 바로 그
    # 지점을 향하게 하고, 도착 후 미세 오차만 run_nav_leg 의 회전→직진→회전으로 보정한다.
    dock_goal_xy = np.array([CARTER_START_POSE["x"], CARTER_START_POSE["y"]])
    dock_yaw = float(np.radians(CARTER_START_POSE["yaw_deg"]))
    if not run_nav_leg(world, ros_node, clock_pub, goal_pub, cmd_vel_pub, pick_state,
                       dock_goal_xy, dock_yaw, dock_goal_xy, dock_yaw, "DOCK"):
        ros_node.destroy_node()
        rclpy.shutdown()
        return

    print("[INFO] 쓰레기통 Nav2 파지 + big_trash 덤프 + 원위치 복귀 + docking_station 복귀 "
          "테스트 완료. 창을 닫으면 종료됩니다.")
    while simulation_app.is_running():
        world.step(render=True)
        rclpy.spin_once(ros_node, timeout_sec=0.0)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    simulation_app.close()
