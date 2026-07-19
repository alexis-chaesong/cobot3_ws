#demo
import omni.usd
import omni.kit.commands
from pxr import UsdGeom, Gf, Sdf, UsdPhysics
from omni.physx.scripts import physicsUtils

stage = omni.usd.get_context().get_stage()

# 기존 테스트 프림 정리
for p in ["/World/cloth", "/World/particleSystem", "/World/physicsScene",
          "/World/GroundPlane", "/World/Plane_01"]:
    prim = stage.GetPrimAtPath(p)
    if prim.IsValid():
        stage.RemovePrim(p)

# Physics Scene
UsdPhysics.Scene.Define(stage, "/World/physicsScene")		#물리 시뮬레이터 관장

# 바닥
result, ground_path = omni.kit.commands.execute("CreateMeshPrimWithDefaultXform", prim_type="Plane")	#바닥 역할 맡을 표면 생성
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(ground_path))			#Collision API 붙이기

# 세분화된 천 메시 (0.04 x 0.04m, 20x20 격자 밀도 선정) 
result, cloth_path = omni.kit.commands.execute(
    "CreateMeshPrimWithDefaultXform",
    prim_type="Plane",
    u_patches=20,
    v_patches=20,
)
cloth_prim = stage.GetPrimAtPath(cloth_path)
physicsUtils.set_or_add_translate_op(cloth_prim, translate=Gf.Vec3f(0.0, 0.0, 0.05))
physicsUtils.set_or_add_scale_op(cloth_prim, scale=Gf.Vec3f(0.04, 0.04, 1.0))

print("메시 생성 완료:", cloth_path)
