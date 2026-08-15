import os
import sys
import time

# check if the system is windows, if so, add the path of blender
if os.name == "nt":
    packages_path = (
        r"C:\Users\JC-Ba\AppData\Roaming\Python\Python311\Scripts"
        + r"\\..\\site-packages"
    )
    sys.path.insert(0, packages_path)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else ""

import bpy
import numpy as np


def _project_root() -> str:
    extra_starts = [SCRIPT_DIR] if SCRIPT_DIR else []
    if bpy.data.filepath:
        extra_starts.append(os.path.dirname(os.path.abspath(bpy.data.filepath)))

    for start in extra_starts:
        cur = os.path.abspath(start)
        for _ in range(8):
            marker = os.path.join(cur, "pm_links.py")
            if os.path.isfile(marker):
                return cur
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent

    raise FileNotFoundError(
        "Cannot find dial-mpc project root (pm_links.py). "
        "Open and run /home/ubuntu/dial-mpc/import_blender_pm.py instead of a copy "
        "embedded in the .blend file."
    )


PROJECT_ROOT = _project_root()
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from pm_links import BODY_INDEX, LINK_MAPPER as link_mapper


# --- performance knobs ---
IMPORT_TRAJECTORIES = False  # DIAL-MPC 轨迹曲线，极慢，默认关闭
FRAME_STEP = 1  # 每隔 N 帧导入一次，预览时可设为 2/5/10
OUTPUT_BLEND = os.path.join(PROJECT_ROOT, "blend/pm_v2_animated.blend")


def _ensure_action(obj: bpy.types.Object) -> bpy.types.Action:
    obj.animation_data_create()
    old_action = obj.animation_data.action
    if old_action is not None:
        bpy.data.actions.remove(old_action)
    action = bpy.data.actions.new(f"{obj.name}_anim")
    obj.animation_data.action = action
    return action


def _write_fcurve(
    action: bpy.types.Action,
    obj: bpy.types.Object,
    data_path: str,
    index: int,
    frames: np.ndarray,
    values: np.ndarray,
) -> None:
    curve = action.fcurve_ensure_for_datablock(
        datablock=obj,
        data_path=data_path,
        index=index,
    )
    keyframes = curve.keyframe_points
    keyframes.add(len(frames))
    frame_values = zip(frames.tolist(), values.tolist())
    for i, (frame, value) in enumerate(frame_values):
        keyframes[i].co = (frame, value)
        keyframes[i].interpolation = "LINEAR"
    curve.update()


def _apply_link_animation_fast(
    obj: bpy.types.Object,
    frames: np.ndarray,
    locations: np.ndarray,
    quats_xyzw: np.ndarray,
) -> None:
    obj.rotation_mode = "QUATERNION"
    action = _ensure_action(obj)
    quats_wxyz = quats_xyzw[:, [3, 0, 1, 2]]

    for axis in range(3):
        _write_fcurve(action, obj, "location", axis, frames, locations[:, axis])
    for axis in range(4):
        _write_fcurve(
            action, obj, "rotation_quaternion", axis, frames, quats_wxyz[:, axis]
        )

    obj.location = locations[0]
    obj.rotation_quaternion = quats_wxyz[0].tolist()


def _import_robot_animation(
    link_pos: np.ndarray,
    link_quat_xyzw: np.ndarray,
    hrender_start: int,
    hrender_end: int,
    frame_step: int,
) -> int:
    slice_pos = link_pos[:, hrender_start:hrender_end:frame_step]
    slice_quat = link_quat_xyzw[:, hrender_start:hrender_end:frame_step]
    frames = np.arange(0, slice_pos.shape[1], dtype=np.float64)
    imported = 0

    t0 = time.time()
    for link_idx, link_name in enumerate(link_mapper):
        if link_name not in bpy.data.objects:
            continue
        body_idx = BODY_INDEX[link_name]
        _apply_link_animation_fast(
            bpy.data.objects[link_name],
            frames,
            slice_pos[body_idx],
            slice_quat[body_idx],
        )
        imported += 1
        if link_idx % 5 == 0 or link_idx == len(link_mapper) - 1:
            elapsed = time.time() - t0
            print(
                f"[robot] {link_idx + 1}/{len(link_mapper)} links, "
                f"{len(frames)} frames/link, {elapsed:.1f}s"
            )

    return len(frames)


def _import_trajectories(
    link_pos_original: np.ndarray,
    xsite_feet_original: np.ndarray,
    hrender_start: int,
    hrender: int,
) -> None:
    Hsample = 40
    Nsample = 8
    Ndiffuse = 4
    Ndownsample = 10
    Hdownsample = 3

    xssss_torso = np.zeros((hrender, Ndiffuse, Nsample, Hsample, 3))
    xssss_feet = np.zeros((hrender, Ndiffuse, Nsample, Hsample, 2, 3))
    for i in range(hrender):
        xs_torso_ref = (
            link_pos_original[i + hrender_start : i + hrender_start + Hsample, 0]
            + np.array([0.1, 0.0, 0.0])
        )
        xs_feet_ref = xsite_feet_original[
            i + hrender_start : i + hrender_start + Hsample
        ]
        for j in range(Ndiffuse):
            sigma = 0.1 * (0.5**j)
            xssss_torso[i, j] = (
                xs_torso_ref + np.random.randn(Nsample, Hsample, 3) * sigma
            )
            xssss_feet[i, j] = (
                xs_feet_ref + np.random.randn(Nsample, Hsample, 2, 3) * sigma
            )
            alpha = 0.2
            for k in range(1, Hsample):
                xssss_torso[i, j, :, k] = (
                    alpha * xssss_torso[i, j, :, k]
                    + (1 - alpha) * xssss_torso[i, j, :, k - 1]
                )
                xssss_feet[i, j, :, k] = (
                    alpha * xssss_feet[i, j, :, k]
                    + (1 - alpha) * xssss_feet[i, j, :, k - 1]
                )

    n_points = Hsample // Ndownsample
    traj_names = [
        "LINK_BASE",
        "force_sensor_left_foot",
        "force_sensor_right_foot",
    ]
    render_frames = np.arange(0, hrender, Hdownsample, dtype=np.float64)
    point_indices = np.arange(0, Hsample, Ndownsample)

    t0 = time.time()
    for i in range(Ndiffuse):
        k = i / (Ndiffuse - 1)
        color = (1 - k) * np.array([1, 1, 1, 0.1]) + k * np.array([1, 0, 0, 1.0])
        material = bpy.data.materials.new(name=f"diffuse_material_{i}")
        material.diffuse_color = color
        material.use_nodes = True
        principled_bsdf = material.node_tree.nodes.get("Principled BSDF")
        if principled_bsdf:
            principled_bsdf.inputs["Base Color"].default_value = color

        for j in range(Nsample):
            for traj_name in traj_names:
                if traj_name == "LINK_BASE":
                    traj = xssss_torso[:, i, j]
                elif traj_name == "force_sensor_left_foot":
                    traj = xssss_feet[:, i, j, :, 0]
                else:
                    traj = xssss_feet[:, i, j, :, 1]

                curve_data = bpy.data.curves.new(
                    name=f"{traj_name}_diffuse{i}_sample{j}", type="CURVE"
                )
                curve_data.dimensions = "3D"
                curve_data.bevel_depth = 0.002
                curve_data.fill_mode = "FULL"
                spline = curve_data.splines.new(type="NURBS")
                spline.points.add(n_points)
                spline.use_cyclic_u = False
                spline.order_u = 4
                spline.resolution_u = 12

                curve_object = bpy.data.objects.new(
                    f"{traj_name}_diffuse{i}_sample{j}", curve_data
                )
                bpy.context.collection.objects.link(curve_object)
                curve_object.data.materials.append(material)

                action = _ensure_action(curve_object)
                for point_idx, src_idx in enumerate(point_indices):
                    spline.points[point_idx].co = (
                        float(traj[0, src_idx, 0]),
                        float(traj[0, src_idx, 1]),
                        float(traj[0, src_idx, 2]),
                        1.0,
                    )
                    for axis in range(3):
                        values = traj[render_frames, src_idx, axis]
                        _write_fcurve(
                            action,
                            curve_object,
                            f"points[{point_idx}].co",
                            axis,
                            render_frames,
                            values,
                        )

    print(f"[trajectories] done in {time.time() - t0:.1f}s")


def import_animation():
    exp_idx = 0
    exp_name = [
        "",
    ][exp_idx]
    file_name = f"pm_data_{exp_name}"

    hrender_start, hrender_end = [
        [0, None],
    ][exp_idx]

    file_prefix = os.path.join(PROJECT_ROOT, "data")
    link_pos_original = np.load(os.path.join(file_prefix, f"{file_name}_xpos.npy"))
    link_quat_wxyz_original = np.load(os.path.join(file_prefix, f"{file_name}_xquat.npy"))

    if hrender_end is None:
        hrender_end = link_pos_original.shape[0]
    hrender = hrender_end - hrender_start

    link_quat_xyzw_original = link_quat_wxyz_original[:, :, [1, 2, 3, 0]]
    link_pos = np.transpose(link_pos_original, (1, 0, 2))
    link_quat_xyzw = np.transpose(link_quat_xyzw_original, (1, 0, 2))

    t0 = time.time()
    n_frames = _import_robot_animation(
        link_pos,
        link_quat_xyzw,
        hrender_start,
        hrender_end,
        FRAME_STEP,
    )

    scene = bpy.context.scene
    scene.frame_start = 0
    scene.frame_end = max(0, n_frames - 1)
    print(
        f"[done] robot animation: {n_frames} frames, "
        f"{len(link_mapper)} links, {time.time() - t0:.1f}s total"
    )

    os.makedirs(os.path.dirname(OUTPUT_BLEND), exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=OUTPUT_BLEND)
    print(f"[saved] {OUTPUT_BLEND}")

    if IMPORT_TRAJECTORIES:
        xsite_feet_original = np.load(
            os.path.join(file_prefix, f"{file_name}_xsite_feet.npy")
        )
        _import_trajectories(
            link_pos_original, xsite_feet_original, hrender_start, hrender
        )


if __name__ == "__main__":
    import_animation()
