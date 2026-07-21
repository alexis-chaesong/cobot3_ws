"""
stage_inspection.py
--------------------
Read-only diagnostic script for the currently open stage (mobile_manipulator_v2.usd).
Run this in Isaac Sim Script Editor BEFORE using mop_grasp_control.py / floor_wipe_motion.py,
any time the scene structure changes (new asset imported, prim renamed, etc.).

It only reads the stage, it does not modify anything.

Usage (Script Editor):
  1. Open mobile_manipulator_v2.usd
  2. Paste this whole file and Run
  3. Compare printed paths against the constants at the top of mop_grasp_control.py
"""

import omni.usd
from pxr import Usd, UsdPhysics

MOP_NAME_HINTS = ("mop", "mopset")


def find_articulation_roots(stage):
    print("--- ArticulationRootAPI prims ---")
    found = []
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            print(f"  {prim.GetPath()}")
            found.append(str(prim.GetPath()))
    if not found:
        print("  (none found)")
    return found


def find_joints_under(stage, root_path: str):
    print(f"--- Joints under {root_path} ---")
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        print(f"  [WARNING] invalid prim: {root_path}")
        return []
    joints = []
    for prim in Usd.PrimRange(root_prim):
        type_name = prim.GetTypeName()
        if "Joint" in type_name:
            joints.append((str(prim.GetPath()), type_name))
            print(f"  {prim.GetPath()}  ({type_name})")
    return joints


def find_finger_like_links(stage, root_path: str):
    print(f"--- Finger/knuckle link prims under {root_path} ---")
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        print(f"  [WARNING] invalid prim: {root_path}")
        return []
    results = []
    for prim in Usd.PrimRange(root_prim):
        path_str = str(prim.GetPath())
        if "/joints/" in path_str:
            continue
        name = prim.GetName().lower()
        if any(key in name for key in ("finger", "knuckle", "pad")):
            has_rb = prim.HasAPI(UsdPhysics.RigidBodyAPI)
            print(f"  {path_str}  RigidBody={has_rb}")
            results.append(path_str)
    return results


def find_mop_prims(stage):
    print("--- Prims whose name looks like a mop ---")
    found = []
    for prim in stage.Traverse():
        name = prim.GetName().lower()
        if any(hint in name for hint in MOP_NAME_HINTS):
            has_rb = prim.HasAPI(UsdPhysics.RigidBodyAPI)
            has_col = prim.HasAPI(UsdPhysics.CollisionAPI)
            print(f"  {prim.GetPath()}  ({prim.GetTypeName()})  RigidBody={has_rb} Collision={has_col}")
            found.append(str(prim.GetPath()))
    if not found:
        print("  (none found - mop not yet added to this scene)")
    return found


def run_stage_inspection():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        print("[ERROR] No stage is open.")
        return

    print("=== STAGE INSPECTION START ===")
    art_roots = find_articulation_roots(stage)

    for root in art_roots:
        find_joints_under(stage, root)

    find_finger_like_links(stage, "/World/m0609_with_gripper")
    find_mop_prims(stage)

    print("=== STAGE INSPECTION DONE ===")


if __name__ == "__main__":
    run_stage_inspection()
