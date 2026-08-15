"""Import PM V2 robot into Blender from local URDF + STL meshes.

Run inside Blender (Scripting tab) or headless:

    blender --background --python setup_blender_pm.py

Optional:

    blender --background --python setup_blender_pm.py -- --save blend/pm_v2_template.blend
"""

from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import bpy
from mathutils import Euler, Matrix, Vector

from pm_links import LINK_MAPPER

URDF_PATH = os.path.join(
    SCRIPT_DIR, "model/pm_v2/robot/pm_v2/urdf/serial_pm_v2.urdf"
)
MESH_DIR = os.path.join(SCRIPT_DIR, "model/pm_v2/robot/pm_v2/meshes")
DEFAULT_BLEND_PATH = os.path.join(SCRIPT_DIR, "blend/pm_v2_template.blend")

# Links without STL in URDF; use primitive placeholders at the joint frame.
SPHERE_LINKS = {
    "LINK_FOOT_L": 0.02,
    "LINK_FOOT_R": 0.02,
}


def _parse_vec(text: str, size: int) -> list[float]:
    values = [float(v) for v in text.split()]
    if len(values) != size:
        raise ValueError(f"Expected {size} values, got {text!r}")
    return values


def _origin_matrix(origin: ET.Element | None) -> Matrix:
    if origin is None:
        return Matrix.Identity(4)
    xyz = _parse_vec(origin.get("xyz", "0 0 0"), 3)
    rpy = _parse_vec(origin.get("rpy", "0 0 0"), 3)
    rot = Euler(rpy, "XYZ").to_matrix().to_4x4()
    return Matrix.Translation(xyz) @ rot


def _mesh_filename(visual: ET.Element) -> str | None:
    geometry = visual.find("geometry")
    if geometry is None:
        return None
    mesh = geometry.find("mesh")
    if mesh is None:
        return None
    filename = mesh.get("filename", "")
    return os.path.basename(filename)


def _sphere_radius(visual: ET.Element) -> float | None:
    geometry = visual.find("geometry")
    if geometry is None:
        return None
    sphere = geometry.find("sphere")
    if sphere is None:
        return None
    return float(sphere.get("radius", "0.02"))


def _parse_urdf(path: str):
    root = ET.parse(path).getroot()
    links: dict[str, dict] = {}
    joints: dict[str, dict] = {}
    root_link = None

    for link in root.findall("link"):
        name = link.get("name")
        visual = link.find("visual")
        mesh_name = _mesh_filename(visual) if visual is not None else None
        sphere_radius = _sphere_radius(visual) if visual is not None else None
        links[name] = {
            "mesh_name": mesh_name,
            "sphere_radius": sphere_radius,
        }

    child_links = set()
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        child_links.add(child)
        joints[child] = {
            "parent": parent,
            "origin": _origin_matrix(joint.find("origin")),
        }

    for link_name in links:
        if link_name not in child_links:
            root_link = link_name
            break

    if root_link is None:
        raise RuntimeError("Could not determine URDF root link")

    return links, joints, root_link


def _compute_world_transforms(joints: dict[str, dict], root_link: str) -> dict[str, Matrix]:
    world = {root_link: Matrix.Identity(4)}
    pending = list(joints.items())
    progress = True
    while pending and progress:
        progress = False
        next_pending = []
        for child, info in pending:
            parent = info["parent"]
            if parent not in world:
                next_pending.append((child, info))
                continue
            world[child] = world[parent] @ info["origin"]
            progress = True
        pending = next_pending

    if pending:
        unresolved = [child for child, _ in pending]
        raise RuntimeError(f"Could not resolve URDF transforms for: {unresolved}")
    return world


def _clear_scene() -> None:
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for block in (
        bpy.data.meshes,
        bpy.data.materials,
        bpy.data.images,
        bpy.data.cameras,
        bpy.data.lights,
    ):
        for item in list(block):
            if item.users == 0:
                block.remove(item)


def _import_stl(filepath: str, link_name: str, world_matrix: Matrix) -> bpy.types.Object:
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"Missing mesh file: {filepath}")
    bpy.ops.wm.stl_import(filepath=filepath)
    obj = bpy.context.selected_objects[0]
    obj.name = link_name
    obj.matrix_world = world_matrix
    return obj


def _create_sphere(link_name: str, radius: float, world_matrix: Matrix) -> bpy.types.Object:
    bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, segments=16, ring_count=8)
    obj = bpy.context.active_object
    obj.name = link_name
    obj.matrix_world = world_matrix
    return obj


def setup_robot(clear: bool = True) -> list[str]:
    if clear:
        _clear_scene()

    links, joints, root_link = _parse_urdf(URDF_PATH)
    world_transforms = _compute_world_transforms(joints, root_link)
    imported: list[str] = []

    for link_name in LINK_MAPPER:
        if link_name not in world_transforms:
            print(f"[skip] {link_name}: not found in URDF transform tree")
            continue

        world_matrix = world_transforms[link_name]
        link_info = links.get(link_name, {})
        mesh_name = link_info.get("mesh_name")
        sphere_radius = SPHERE_LINKS.get(link_name, link_info.get("sphere_radius"))

        if mesh_name:
            mesh_path = os.path.join(MESH_DIR, mesh_name)
            _import_stl(mesh_path, link_name, world_matrix)
            imported.append(link_name)
            print(f"[mesh] {link_name} <- {mesh_name}")
        elif sphere_radius is not None:
            _create_sphere(link_name, sphere_radius, world_matrix)
            imported.append(link_name)
            print(f"[sphere] {link_name} radius={sphere_radius}")
        else:
            empty = bpy.data.objects.new(link_name, None)
            bpy.context.collection.objects.link(empty)
            empty.matrix_world = world_matrix
            imported.append(link_name)
            print(f"[empty] {link_name}")

    scene = bpy.context.scene
    scene.frame_start = 0
    scene.frame_end = 250
    return imported


def _parse_cli_save_path(argv: list[str]) -> str | None:
    if "--save" in argv:
        idx = argv.index("--save")
        if idx + 1 >= len(argv):
            raise ValueError("Missing path after --save")
        return os.path.abspath(argv[idx + 1])
    return None


def main() -> None:
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1 :]
    else:
        argv = []

    save_path = _parse_cli_save_path(argv) or DEFAULT_BLEND_PATH
    imported = setup_robot(clear=True)

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=save_path)
    print(f"Imported {len(imported)}/{len(LINK_MAPPER)} links")
    print(f"Saved Blender file: {save_path}")


if __name__ == "__main__":
    main()
