import os

import mujoco
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PM_MODEL_PATH = os.environ.get(
    "PM_MUJOCO_MODEL",
    os.path.join(SCRIPT_DIR, "model/pm_v2/pm_v2_mesh.xml"),
)
NQ = 31
LEFT_FOOT_SITE = "force_sensor_left_foot"
RIGHT_FOOT_SITE = "force_sensor_right_foot"


def convert_state():
    exp_idx = 0
    task = [
        "",
    ][exp_idx]
    input_path = f"data/pm_data{task}.npy"

    raw = np.load(input_path)
    print(f"data shape: {raw.shape}")
    timestep = raw[:, 0] - raw[0, 0]
    qpos = raw[:, 1 : 1 + NQ]
    print(f"frame number: {len(timestep)}")

    model = mujoco.MjModel.from_xml_path(PM_MODEL_PATH)
    n_body = model.nbody - 1
    mj_data = mujoco.MjData(model)
    left_foot_id = model.site(LEFT_FOOT_SITE).id
    right_foot_id = model.site(RIGHT_FOOT_SITE).id

    xpos = np.zeros((len(timestep), n_body, 3))
    xquat = np.zeros((len(timestep), n_body, 4))
    xsite_feet = np.zeros((len(timestep), 2, 3))

    for i in range(len(timestep)):
        mj_data.qpos[:] = qpos[i]
        mujoco.mj_forward(model, mj_data)
        xpos[i] = mj_data.xpos[1:]
        xquat[i] = mj_data.xquat[1:]
        xsite_feet[i, 0] = mj_data.site_xpos[left_foot_id]
        xsite_feet[i, 1] = mj_data.site_xpos[right_foot_id]

    np.save(f"data/pm_data_{task}_xpos.npy", xpos)
    np.save(f"data/pm_data_{task}_xquat.npy", xquat)
    np.save(f"data/pm_data_{task}_xsite_feet.npy", xsite_feet)
    print(f"saved {n_body} links to data/pm_data_{task}_*.npy")


if __name__ == "__main__":
    convert_state()
