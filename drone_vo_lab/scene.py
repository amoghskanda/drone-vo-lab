"""Original procedural scenes and prescribed camera motion, without external assets.

This is a sensor benchmark. Motion is scripted, not a flight dynamics controller.
Ground-truth camera poses leave this module through the dataset for evaluation only.
"""

from dataclasses import dataclass

import mujoco
import numpy as np

from .geometry import CV_TO_GL, intrinsics, look_at


@dataclass
class Sequence:
    rgb: np.ndarray
    depth: np.ndarray
    timestamps: np.ndarray
    poses: np.ndarray
    calibration: np.ndarray
    scene: str
    seed: int


def scene_xml(seed: int, scene: str = "warehouse") -> str:
    if scene not in {"warehouse", "low_texture"}:
        raise ValueError(f"Unknown scene: {scene}")
    rng = np.random.default_rng(seed)
    pieces = []

    def box(position, size, color):
        p = " ".join(f"{v:.4f}" for v in position)
        s = " ".join(f"{v:.4f}" for v in size)
        rgba = " ".join(f"{v:.3f}" for v in color) + " 1"
        pieces.append(f'<geom type="box" pos="{p}" size="{s}" rgba="{rgba}"/>')

    palette = [[0.20, 0.47, 0.54], [0.70, 0.39, 0.22], [0.68, 0.61, 0.36], [0.25, 0.32, 0.39]]
    for side in [-1, 1]:
        for y in np.arange(-3, 5, 2.0):
            for level in [0.45, 1.3, 2.15]:
                box([side * 3.2, y, level - 0.35], [0.8, 0.88, 0.045], [0.18, 0.23, 0.27])
                for slot in [-0.45, 0.45]:
                    height = rng.uniform(0.18, 0.32)
                    color = palette[int(rng.integers(len(palette)))]
                    if scene == "low_texture":
                        color = [0.48, 0.49, 0.50]
                    box([side * 3.2, y + slot, level], [0.35, 0.27, height], color)
                    if scene == "warehouse":
                        box(
                            [side * 2.84, y + slot, level], [0.012, 0.12, 0.075], [0.91, 0.89, 0.79]
                        )
            for dy in [-0.9, 0.9]:
                box([side * 3.2, y + dy, 1.5], [0.83, 0.035, 1.5], [0.16, 0.22, 0.26])
    if scene == "warehouse":
        for x in np.linspace(-4.5, 4.5, 15):
            for z in np.linspace(0.4, 3.2, 8):
                if rng.random() > 0.35:
                    box([x, 6.88, z], [0.11, 0.012, 0.10], palette[int(rng.integers(4))])
    floor_material = 'material="floor_mat"' if scene == "warehouse" else 'rgba="0.42 0.44 0.45 1"'
    return f"""<mujoco model="Drone VO procedural aisle">
      <option gravity="0 0 -9.81"/>
      <visual><global offwidth="1280" offheight="720"/><headlight ambient="0.55 0.55 0.55" diffuse="0.7 0.7 0.7"/></visual>
      <asset>
        <texture name="floor_tex" type="2d" builtin="checker" rgb1="0.29 0.33 0.35" rgb2="0.45 0.48 0.47" width="512" height="512"/>
        <material name="floor_mat" texture="floor_tex" texrepeat="8 12" texuniform="false" reflectance="0"/>
      </asset>
      <worldbody>
        <light pos="0 0 6" directional="true" dir="0.2 0.2 -1"/>
        <geom type="box" pos="0 0 -0.08" size="5 7 0.08" {floor_material}/>
        <geom type="box" pos="0 7 2" size="5 0.1 2" rgba="0.75 0.76 0.74 1"/>
        <geom type="box" pos="-5 0 2" size="0.1 7 2" rgba="0.63 0.67 0.69 1"/>
        <geom type="box" pos="5 0 2" size="0.1 7 2" rgba="0.63 0.67 0.69 1"/>
        {"".join(pieces)}
        <body name="sensor_rig" mocap="true"><camera name="rgbd" fovy="65"/></body>
      </worldbody>
    </mujoco>"""


def camera_path(t: float, duration: float, seed: int = 0) -> np.ndarray:
    phase = t / duration
    offset = seed * 0.23
    position = np.array(
        [
            0.85 * np.sin(2 * np.pi * phase + offset),
            -4.8 + 7.0 * np.sin(np.pi * phase) ** 2,
            1.45 + 0.22 * np.sin(4 * np.pi * phase),
        ]
    )
    target = position + [0.55 * np.sin(3 * np.pi * phase), 4.0, -0.25]
    return look_at(position, target, roll=0.055 * np.sin(4 * np.pi * phase))


def capture(scene: str, seed: int, frames: int, fps: float, width: int, height: int) -> Sequence:
    model = mujoco.MjModel.from_xml_string(scene_xml(seed, scene))
    data = mujoco.MjData(model)
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "rgbd")
    timestamps = np.arange(frames) / fps
    rgb, depth, poses = [], [], []
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        for t in timestamps:
            pose = camera_path(float(t), max((frames - 1) / fps, 1 / fps), seed)
            gl_rotation = pose[:3, :3] @ CV_TO_GL
            data.mocap_pos[0] = pose[:3, 3]
            mujoco.mju_mat2Quat(data.mocap_quat[0], gl_rotation.ravel())
            data.time = t
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera="rgbd")
            renderer.disable_depth_rendering()
            rgb.append(renderer.render().copy())
            renderer.enable_depth_rendering()
            depth.append(renderer.render().copy())
            actual = np.eye(4)
            actual[:3, :3] = data.cam_xmat[camera_id].reshape(3, 3) @ CV_TO_GL
            actual[:3, 3] = data.cam_xpos[camera_id]
            poses.append(actual)
    return Sequence(
        np.array(rgb),
        np.array(depth),
        timestamps,
        np.array(poses),
        intrinsics(width, height, 65),
        scene,
        seed,
    )
