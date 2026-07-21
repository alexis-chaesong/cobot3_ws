"""Isaac Sim 5.1 + Doosan M0609 + OnRobot RG2 — Surface Gripper 기반 걸레 툴 체인저.

Part 1: 거치대(tool stand) <-> RG2 fingertip 간 걸레(Mop/Cloth) 탈부착.

설계 요점
--------
- 물리적 파지/고정은 Isaac Sim 5.1의 Surface Gripper
  (``isaacsim.robot.surface_gripper``, ``isaacsim.robot.manipulators.grippers
  .surface_gripper.SurfaceGripper``) 가 전담한다. Surface Gripper는 새로운 prim이
  아니라, 기존 RG2 fingertip rigid body prim에 부착되는 D6 조인트 + 스키마 프림으로
  authoring 된다 (:mod:`surface_gripper_utils` 참고).
- RG2 손가락의 실제 open/close 모션(시뮬레이션 관절 각도 및 pymodbus를 통한 실물 제어)은
  화면상 "RG2가 걸레를 잡은 것처럼" 보이기 위한 **시각적 동기화** 용도로만 별도 호출된다.
  Surface Gripper의 close()/open() 타이밍과 맞출 뿐, 서로 물리적으로 의존하지 않는다
  (RG2 모션이 실패해도 Surface Gripper의 고정 여부에는 영향이 없다).

Isaac Sim 5.1 기준 SurfaceGripper 실제 시그니처 (로컬 설치본
``isaacsim.robot.manipulators.grippers.surface_gripper.SurfaceGripper`` 소스로 확인):

    SurfaceGripper(end_effector_prim_path: str, surface_gripper_path: str) -> None

    close() / open() / is_closed() / is_open() / initialize(physics_sim_view=None)

과거(≤4.x, ``omni.isaac.manipulators``) API에 있던 translate/direction/force_limit/
torque_limit 같은 생성자 인자는 5.1에는 없다 — 해당 값들은 Surface Gripper "프림의
USD 속성"(maxGripDistance, coaxialForceLimit, shearForceLimit 등)으로 옮겨졌다.
"""
from __future__ import annotations

import logging
import sys
from pathlib import Path
from typing import Optional

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

import surface_gripper_utils  # noqa: E402  (Isaac Sim 미기동 환경에서도 import 자체는 가능)

try:
    import omni.usd
    from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper
    from isaacsim.robot.surface_gripper._surface_gripper import acquire_surface_gripper_interface

    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    print("Warning: Isaac Sim modules not found. Run this script within Isaac Sim (python.sh).")

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


class ToolChangerController:
    """RG2 fingertip에 부착된 Surface Gripper로 걸레(Mop)를 탈부착하는 컨트롤러."""

    def __init__(
        self,
        rg2_fingertip_prim_path: str,
        mop_handle_prim_path: str,
        stand_position: np.ndarray,
        stand_orientation: np.ndarray,
        approach_orientation: Optional[np.ndarray] = None,
        fingertip_offset_from_ik_frame: Optional[np.ndarray] = None,
        surface_gripper_prim_path: Optional[str] = None,
        rg2_gripper=None,
        rg2_hardware_client=None,
        auto_create_surface_gripper: bool = True,
        max_grip_distance: float = 0.02,
    ) -> None:
        """
        Args:
            rg2_fingertip_prim_path: Surface Gripper가 부착될 RG2 rigid body prim 경로.
                **주의**: RG2는 평행 2지 그리퍼라 물리적으로 단일한 "fingertip" 링크가
                없다. left_inner_finger/right_inner_finger 처럼 관절로 움직이는 링크에
                붙이면, 손가락이 열리고 닫힐 때마다 파지 조인트의 고정 위치도 함께
                움직여 버린다. 손가락 모션과 완전히 분리된 고정을 원한다면(요구사항:
                "서로 물리적으로 의존하지 않음") tool0에 고정된 비가동 링크
                (예: gripper_body, quick_changer)를 지정하는 것을 권장한다.
            mop_handle_prim_path: 거치대에 놓인 걸레 USD의 접합부(handle) prim 경로.
                이 prim의 world pose를 접근 목표 좌표로 사용한다.
            stand_position / stand_orientation: 거치대(tool stand) 자체의 pose.
                걸레 반납 후 팔이 복귀할 목표 좌표.
            approach_orientation: 걸레 접합부에 접근할 때 사용할 엔드이펙터
                orientation. None이면 stand_orientation을 그대로 사용한다.
                **주의**: 걸레 handle prim 자신의 world orientation을 그대로 IK
                목표로 쓰지 않는다 — M0609에서 identity(1,0,0,0) orientation은
                손목 특이점 근처라서 RMPflow가 팔을 베이스 근처로 접어버리는
                현상을 headless 테스트로 직접 확인했다. (0,1,0,0)처럼 검증된
                orientation을 stand_orientation/approach_orientation으로
                지정해야 한다.
            fingertip_offset_from_ik_frame: rg2_fingertip_prim_path(예: gripper_body)가
                RMPflow의 IK 목표 프레임(tool0) 기준으로 실제로 어디에 있는지의
                world-frame 오차 벡터 (fingertip_world_pos - ik_frame_world_pos).
                gripper_body는 tool0에 고정 조인트로만 연결돼 있어 이 오프셋은
                로봇 자세와 무관하게 일정하다. None(기본값)이면 0으로 취급해
                tool0을 handle 좌표에 그대로 겹치게 하는데, 실제로는 fingertip이
                거기서 수 cm 벗어난 곳에 위치하게 되어 Surface Gripper가 grip
                시도 시 그 오차를 강하게 보정하려다 물체를 튕겨내는 문제를 겪었다
                (headless 실측으로 확인: 이 자산에서 gripper_body는 tool0보다
                약 (-0.027, -0.009, -0.046)m 어긋나 있었다). 이 값을 넘기면
                approach_tool_stand()/stand_return_target()이 목표에서 이만큼
                빼서 IK에 넘기므로, fingertip이 실제 목표 좌표에 정확히 오게 된다.
            surface_gripper_prim_path: 이미 GUI 등에서 저작해 둔 Surface Gripper
                프림이 있다면 그 경로를 지정한다. None이면
                ``<fingertip>/mop_surface_gripper`` 경로에 자동 생성한다.
            rg2_gripper: (선택) 시각 동기화용 시뮬레이션 그리퍼 객체
                (예: ``isaacsim.robot.manipulators.grippers.ParallelGripper``).
                ``.close()`` / ``.open()`` 을 그대로 호출한다.
            rg2_hardware_client: (선택) 실물 OnRobot RG2를 pymodbus 등으로 제어하는
                클라이언트. ``.close_gripper()`` / ``.open_gripper()`` 를 호출한다.
                TODO: 실제 레지스터 주소/슬레이브 ID는 장비 매뉴얼에 맞춰 구현 필요.
            auto_create_surface_gripper: True면 생성 시점에 스테이지에 Surface
                Gripper 조인트+프림이 없을 경우 자동으로 authoring한다.
            max_grip_distance: Surface Gripper가 물체를 붙잡을 수 있는 최대 거리(m).
        """
        self.rg2_fingertip_path = rg2_fingertip_prim_path
        self.mop_handle_path = mop_handle_prim_path
        self.stand_position = np.asarray(stand_position, dtype=float)
        self.stand_orientation = np.asarray(stand_orientation, dtype=float)
        self.approach_orientation = (
            np.asarray(approach_orientation, dtype=float)
            if approach_orientation is not None
            else self.stand_orientation
        )
        self.fingertip_offset_from_ik_frame = (
            np.asarray(fingertip_offset_from_ik_frame, dtype=float)
            if fingertip_offset_from_ik_frame is not None
            else np.zeros(3)
        )
        self.rg2_gripper = rg2_gripper
        self.rg2_hardware_client = rg2_hardware_client

        self.surface_gripper_prim_path = surface_gripper_prim_path
        self.surface_gripper: Optional["SurfaceGripper"] = None
        self._gripper_interface = None

        if not ISAAC_SIM_AVAILABLE:
            logger.warning("Isaac Sim이 아닌 환경입니다. Surface Gripper는 Mock 모드로 동작합니다.")
            return

        if auto_create_surface_gripper:
            self._create_surface_gripper_prim(max_grip_distance=max_grip_distance)

        if self.surface_gripper_prim_path is None:
            raise ValueError(
                "surface_gripper_prim_path가 없습니다. auto_create_surface_gripper=True로 "
                "두거나, 이미 존재하는 Surface Gripper 프림 경로를 직접 지정하세요."
            )

        self._gripper_interface = acquire_surface_gripper_interface()
        try:
            self.surface_gripper = SurfaceGripper(
                end_effector_prim_path=self.rg2_fingertip_path,
                surface_gripper_path=self.surface_gripper_prim_path,
            )
        except Exception as e:
            logger.error(f"Surface Gripper 래퍼 초기화 실패: {e}")
            self.surface_gripper = None

    def _create_surface_gripper_prim(self, max_grip_distance: float) -> None:
        stage = omni.usd.get_context().get_stage()
        self.surface_gripper_prim_path = surface_gripper_utils.setup_mop_surface_gripper(
            stage,
            fingertip_prim_path=self.rg2_fingertip_path,
            gripper_prim_path=self.surface_gripper_prim_path,
            max_grip_distance=max_grip_distance,
        )

    def initialize(self) -> None:
        """world.reset()/Play 이후, 하드 리셋마다 반드시 호출.

        Surface Gripper 래퍼는 물리 시뮬레이션 뷰가 준비된 뒤에만 initialize할 수
        있으므로, 반드시 ``world.reset()`` (또는 timeline play) 다음에 호출한다.
        """
        if self.surface_gripper:
            self.surface_gripper.initialize()
            logger.info(f"[CHECKPOINT] Surface Gripper 초기화 완료: {self.surface_gripper_prim_path}")

    # ------------------------------------------------------------------
    # 목표 좌표 조회 (실제 이동은 호출부의 RMPflow 루프가 담당 — Part 3에서 궤적 인터페이스로 확장)
    # ------------------------------------------------------------------
    def get_mop_handle_world_pose(self):
        """거치대에 놓인 걸레 USD의 접합부(handle) world pose를 조회한다.

        Returns:
            (position: np.ndarray[3], orientation: np.ndarray[4] (w,x,y,z))
        """
        stage = omni.usd.get_context().get_stage()
        handle_prim = stage.GetPrimAtPath(self.mop_handle_path)
        if not handle_prim.IsValid():
            raise ValueError(f"mop handle prim이 유효하지 않습니다: {self.mop_handle_path}")

        matrix = omni.usd.get_world_transform_matrix(handle_prim)
        translation = matrix.ExtractTranslation()
        rotation = matrix.ExtractRotationQuat()
        position = np.array([translation[0], translation[1], translation[2]])
        imaginary = rotation.GetImaginary()
        orientation = np.array([rotation.GetReal(), imaginary[0], imaginary[1], imaginary[2]])
        return position, orientation

    def approach_tool_stand(self):
        """거치대(걸레 접합부) 접근을 위한 IK 목표 좌표를 반환한다.

        목표 orientation은 handle prim 자신의 orientation이 아니라
        ``self.approach_orientation`` (검증된 값)을 사용한다. handle prim의
        orientation은 USD 모델링 편의상 임의로 authoring된 값일 뿐이라 IK 목표로
        신뢰할 수 없다 (identity orientation이 M0609 손목 특이점 근처라 팔이
        접혀버리는 문제를 실제로 겪었다).

        목표 위치(position)는 handle prim의 실제 world position에서
        ``fingertip_offset_from_ik_frame``을 뺀 값이다 — RMPflow는 tool0(IK 프레임)을
        목표로 이동시키지만, 실제로 걸레를 붙잡는 건 그로부터 어긋난 위치에 있는
        rg2_fingertip_prim_path(gripper_body)이기 때문에, IK 목표를 그만큼
        미리 보정해서 fingertip이 handle 좌표에 정확히 오도록 한다.

        실제 관절 이동은 호출부에서 매 시뮬레이션 tick마다
        ``rmpflow_controller.forward(target_position, target_orientation)`` 을 호출해
        수행해야 한다 (Isaac Sim의 MotionPolicyController.forward()는 목표를 한 번만
        설정하는 API가 아니라, 매 프레임 재호출이 필요한 stateless 스텝 함수이다).
        """
        handle_position, _handle_orientation = self.get_mop_handle_world_pose()
        target_position = handle_position - self.fingertip_offset_from_ik_frame
        target_orientation = self.approach_orientation
        logger.info(
            f"로봇 팔이 걸레 접합부 {handle_position} 위치로 접근 중입니다... "
            f"(IK 목표={target_position}, fingertip 오프셋 보정={self.fingertip_offset_from_ik_frame})"
        )
        return target_position, target_orientation

    def stand_return_target(self):
        """작업 종료 후 거치대 원위치 복귀를 위한 IK 목표 좌표.

        approach_tool_stand()와 동일한 이유로 fingertip_offset_from_ik_frame을 뺀다.
        """
        target_position = self.stand_position - self.fingertip_offset_from_ik_frame
        logger.info(f"거치대 원위치 {self.stand_position} 로 복귀 목표를 설정합니다 (IK 목표={target_position}).")
        return target_position, self.stand_orientation

    @staticmethod
    def is_at_pose(current_position: np.ndarray, target_position: np.ndarray, tolerance: float = 0.01) -> bool:
        """호출부 이동 루프에서 목표 도달 여부를 판단하기 위한 헬퍼."""
        return bool(np.linalg.norm(np.asarray(current_position) - np.asarray(target_position)) < tolerance)

    # ------------------------------------------------------------------
    # 파지 / 반납
    # ------------------------------------------------------------------
    def grasp_mop(self) -> None:
        """걸레 파지: RG2 시각 동기화(닫힘) + Surface Gripper 물리적 고정."""
        logger.info("걸레 파지(Grasp) 시퀀스를 시작합니다.")

        # 1. RG2 시각적 모션 동기화 (Surface Gripper와 물리적으로 독립적, 타이밍만 일치)
        self._sync_rg2_visual_close()

        # 2. Surface Gripper를 이용한 물리적 고정
        if self.surface_gripper:
            self.surface_gripper.close()

            # [검증용 로그/체크포인트] Gripper 상태 확인
            if self.surface_gripper.is_closed():
                logger.info(
                    f"[CHECKPOINT] Surface Gripper 상태: CLOSED ({self.surface_gripper_prim_path}). "
                    "걸레가 물리적으로 고정되었습니다."
                )
                self._log_gripped_objects()
            else:
                logger.warning("[CHECKPOINT] Surface Gripper 닫힘 실패 (그립 범위 내 물체 없음/거리 초과).")
        else:
            logger.info("[CHECKPOINT] Surface Gripper 상태: CLOSED. (Mock)")

    def release_mop_to_stand(self) -> None:
        """작업 종료 후 거치대에 걸레 반납: RG2 시각 동기화(열림) + Surface Gripper 해제."""
        logger.info("거치대로 원위치하여 걸레를 반납합니다.")

        # 1. RG2 시각적 열림 동기화
        self._sync_rg2_visual_open()

        # 2. Surface Gripper를 이용한 물리적 결합 해제
        if self.surface_gripper:
            self.surface_gripper.open()

            # [검증용 로그/체크포인트] Gripper 상태 확인
            if not self.surface_gripper.is_closed():
                logger.info(
                    f"[CHECKPOINT] Surface Gripper 상태: OPEN ({self.surface_gripper_prim_path}). "
                    "걸레가 물리적으로 해제되었습니다."
                )
            else:
                logger.warning("[CHECKPOINT] Surface Gripper 열림 실패.")
        else:
            logger.info("[CHECKPOINT] Surface Gripper 상태: OPEN. (Mock)")

    def _log_gripped_objects(self) -> None:
        if self._gripper_interface and self.surface_gripper_prim_path:
            gripped = self._gripper_interface.get_gripped_objects(self.surface_gripper_prim_path)
            logger.info(f"[CHECKPOINT] 파지 중인 오브젝트: {list(gripped)}")

    # ------------------------------------------------------------------
    # RG2 시각적 동기화 (Surface Gripper의 물리적 고정과 독립적, 타이밍만 일치시킴)
    # ------------------------------------------------------------------
    def _sync_rg2_visual_close(self) -> None:
        logger.info("[동기화] RG2 손가락 닫힘 명령 (시뮬레이션 관절 / pymodbus)")
        if self.rg2_gripper is not None:
            self.rg2_gripper.close()
        if self.rg2_hardware_client is not None:
            # TODO: 실물 OnRobot RG2 pymodbus 제어. 레지스터 주소/슬레이브 ID는
            # OnRobot RG2 Modbus TCP 매뉴얼 기준으로 채워야 한다.
            self.rg2_hardware_client.close_gripper()

    def _sync_rg2_visual_open(self) -> None:
        logger.info("[동기화] RG2 손가락 열림 명령 (시뮬레이션 관절 / pymodbus)")
        if self.rg2_gripper is not None:
            self.rg2_gripper.open()
        if self.rg2_hardware_client is not None:
            # TODO: 실물 OnRobot RG2 pymodbus 제어.
            self.rg2_hardware_client.open_gripper()
