from pathlib import Path

import numpy as np
import omni.usd
import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.prims import SingleArticulation


def _read_base_link_world_pose(robot_articulation: SingleArticulation):
    """Nova Carter 등에 장착된 팔은 articulation root pose가 (0,0,0)으로 나올 수 있어
    URDF base_link의 월드 pose를 RMPFlow 기준으로 사용한다."""
    prim_path = robot_articulation.prim_path
    base_path = f"{prim_path}/base_link"
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(base_path)
    if not prim.IsValid():
        return robot_articulation.get_world_pose()

    matrix = omni.usd.get_world_transform_matrix(prim)
    translation = matrix.ExtractTranslation()
    quat = matrix.ExtractRotationQuat()
    imag = quat.GetImaginary()
    position = np.array([translation[0], translation[1], translation[2]], dtype=float)
    orientation = np.array([quat.GetReal(), imag[0], imag[1], imag[2]], dtype=float)
    return position, orientation


class RMPFlowController(mg.MotionPolicyController):
    """M0609용 RMPFlow controller."""

    def __init__(
        self,
        name: str,
        robot_articulation: SingleArticulation,
        physics_dt: float = 1.0 / 60.0,
        urdf_path: str | None = None,
        robot_description_path: str | None = None,
        rmpflow_config_path: str | None = None,
        end_effector_frame_name: str = "tool0",
        maximum_substep_size: float = 0.00334,
    ) -> None:
        base_dir = Path(__file__).resolve().parent
        # base_dir = src/integration/integration/rmpflow -> parents[2] = src
        assets_dir = base_dir.parents[2] / "assets"
        urdf_path = str(Path(urdf_path) if urdf_path else assets_dir / "robots" / "m0609_with_gripper.urdf")
        robot_description_path = str(
            Path(robot_description_path) if robot_description_path else base_dir / "m0609_description.yaml"
        )
        rmpflow_config_path = str(
            Path(rmpflow_config_path) if rmpflow_config_path else base_dir / "m0609_rmpflow_common.yaml"
        )

        self.rmp_flow = mg.lula.motion_policies.RmpFlow(
            robot_description_path=robot_description_path,
            rmpflow_config_path=rmpflow_config_path,
            urdf_path=urdf_path,
            end_effector_frame_name=end_effector_frame_name,
            maximum_substep_size=maximum_substep_size,
        )

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)
        super().__init__(name=name, articulation_motion_policy=self.articulation_rmp)

        self._default_position, self._default_orientation = _read_base_link_world_pose(
            self._articulation_motion_policy._robot_articulation
        )
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )
        
        # [Fix] 명시적으로 조인트 인덱스 매핑 캐싱
        self._arm_dof_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self._arm_dof_indices = [
            self._articulation_motion_policy._robot_articulation.get_dof_index(name)
            for name in self._arm_dof_names
        ]

    def add_ground_plane(self, prim_path: str = "/World/GroundPlane", z_position: float = 0.0) -> bool:
        """바닥을 RMPflow 장애물로 등록한다. 이 컨트롤러는 target-tracking IK 만 하고
        world 장애물은 전혀 등록하지 않았어서, IK 가 바닥 아래로 팔을 뚫고 지나가도
        모르는 상태였다 (add_obstacle/add_ground_plane 을 한 번도 호출한 적이 없었음).

        prim_path 가 이미 존재하면(예: move_tash_can.usd 에 이미 있는 /World/GroundPlane)
        그 prim 을 그대로 wrap 하고, 없으면 z_position 높이에 새로 만든다. Lula 는 ground
        plane 을 직접 지원하지 않아 내부적으로 보이지 않는 큰 cuboid(기본 50x50m)를 그
        높이에 맞춰 만들어 static 장애물로 등록한다 — static 장애물은 set_robot_base_pose()
        가 호출될 때마다(우리 쪽 sync_rmpflow_base_pose) 자동으로 robot-base 기준 상대
        위치가 재계산되므로, Nav2 로 로봇이 이동해도 별도 갱신 없이 계속 올바르게 유지된다.
        """
        ground_plane = GroundPlane(prim_path=prim_path, z_position=z_position)
        return self.rmp_flow.add_ground_plane(ground_plane)

    def forward(self, *args, **kwargs):
        action = super().forward(*args, **kwargs)
        # 생성된 액션이 전체 관절에 잘못 적용되지 않도록 인덱스 강제 지정
        if action.joint_indices is None or len(action.joint_indices) == 6:
            action.joint_indices = self._arm_dof_indices
        return action

    def reset(self):
        super().reset()
        self._default_position, self._default_orientation = _read_base_link_world_pose(
            self._articulation_motion_policy._robot_articulation
        )
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position,
            robot_orientation=self._default_orientation,
        )
