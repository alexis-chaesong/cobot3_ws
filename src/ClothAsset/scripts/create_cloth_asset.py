import omni.usd
import omni.kit.commands
from pxr import Usd, UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema
from omni.physx.scripts import physicsUtils

# ========================= 설정값 =========================
ASSET_ROOT = "/World/ClothAsset"      # <- /World 밑으로 통일
MESH_PATH = f"{ASSET_ROOT}/mesh"
PS_PATH = f"{ASSET_ROOT}/particleSystem"

WIDTH = 0.40
HEIGHT = 0.40
U_PATCHES = 80
V_PATCHES = 80
CONTACT_OFFSET = 0.003
REST_OFFSET = 0.002
PARTICLE_CONTACT_OFFSET = 0.003
SOLID_REST_OFFSET = 0.002
SOLVER_POSITION_ITERATION_COUNT = 20

EXPORT_PATH = "/home/rokey/cobot3_ws/src/asset/scenes/cloth_pad.usd"        # 파일 명을 바꿔서 다시 저장할거면 cloth_pad를 원하는 이름으로 바꾸면됨
# ============================================================


def create_cloth_asset_base():
    stage = omni.usd.get_context().get_stage()

    if stage.GetPrimAtPath(ASSET_ROOT).IsValid():
        stage.RemovePrim(ASSET_ROOT)

    # 루트 Xform (pxr API - 이건 항상 지정 경로 그대로 생성됨)
    root_prim = UsdGeom.Xform.Define(stage, ASSET_ROOT)

    # Particle System (pxr API)
    ps = PhysxSchema.PhysxParticleSystem.Define(stage, PS_PATH)
    ps.CreateContactOffsetAttr().Set(CONTACT_OFFSET)
    ps.CreateRestOffsetAttr().Set(REST_OFFSET)
    ps.CreateParticleContactOffsetAttr().Set(PARTICLE_CONTACT_OFFSET)
    ps.CreateSolidRestOffsetAttr().Set(SOLID_REST_OFFSET)
    ps.CreateSolverPositionIterationCountAttr().Set(SOLVER_POSITION_ITERATION_COUNT)

    # 메시 (kit command - 실제 생성 경로를 반드시 반환값으로 확인)
    result, actual_mesh_path = omni.kit.commands.execute(
        "CreateMeshPrimWithDefaultXform",
        prim_type="Plane",
        prim_path=MESH_PATH,
        u_patches=U_PATCHES,
        v_patches=V_PATCHES,
    )

    # 실제 생성 경로가 의도한 경로와 다르면 강제로 옮김
    if actual_mesh_path != MESH_PATH:
        print(f"[경고] 메시가 의도한 경로가 아닌 {actual_mesh_path} 에 생성됨 -> {MESH_PATH} 로 이동")
        omni.kit.commands.execute(
            "MovePrim",
            path_from=actual_mesh_path,
            path_to=MESH_PATH,
        )

    mesh_prim = stage.GetPrimAtPath(MESH_PATH)
    physicsUtils.set_or_add_scale_op(mesh_prim, scale=Gf.Vec3f(WIDTH, HEIGHT, 1.0))

    print(f"[완료] {ASSET_ROOT} 생성됨 (mesh: {MESH_PATH}, particleSystem: {PS_PATH})")
    print("[다음 단계] Stage 트리에서 mesh 선택 -> +Add > Physics > Particle Cloth 로 붙여주세요.")
    print(f"[확인] Particle Cloth의 Particle System 필드가 '{PS_PATH}' 를 가리키는지 반드시 확인하세요.")


def export_cloth_asset():
    stage = omni.usd.get_context().get_stage()

    root_prim = stage.GetPrimAtPath(ASSET_ROOT)
    if not root_prim.IsValid():
        print(f"[오류] {ASSET_ROOT} 프림이 없습니다.")
        return

    mesh_prim = stage.GetPrimAtPath(MESH_PATH)
    if not mesh_prim.HasAPI(PhysxSchema.PhysxParticleClothAPI):
        print("[경고] mesh에 Particle Cloth API가 아직 안 붙어있습니다.")
        return

    # relationship이 실제로 올바른 particleSystem을 가리키는지 최종 확인
    particle_api = PhysxSchema.PhysxParticleAPI(mesh_prim)
    targets = particle_api.GetParticleSystemRel().GetTargets()
    if not targets or str(targets[0]) != PS_PATH:
        print(f"[경고] Particle System 연결이 잘못됨: {targets} (기대값: {PS_PATH})")
        print("Export를 중단합니다. Property 탭에서 Particle System 필드를 다시 확인해주세요.")
        return

    export_layer = Sdf.Layer.CreateNew(EXPORT_PATH)
    Sdf.CopySpec(
        stage.GetRootLayer(), Sdf.Path(ASSET_ROOT),
        export_layer, Sdf.Path(ASSET_ROOT),
    )
    export_layer.defaultPrim = ASSET_ROOT.split("/")[-1]
    export_layer.Save()

    print(f"[완료] 에셋 저장됨: {EXPORT_PATH}")


# ========================= 실행부 =========================
create_cloth_asset_base()
# export_cloth_asset()
