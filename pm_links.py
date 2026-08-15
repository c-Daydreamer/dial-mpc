# MuJoCo xpos/xquat 的 body 顺序（与 convert_state_pm.py 输出一致，共 29 个）
MUJOCO_BODY_NAMES = [
    "LINK_BASE",
    "LINK_HIP_PITCH_L",
    "LINK_HIP_ROLL_L",
    "LINK_HIP_YAW_L",
    "LINK_KNEE_PITCH_L",
    "LINK_ANKLE_PITCH_L",
    "LINK_ANKLE_ROLL_L",
    "LINK_FOOT_L",
    "LINK_HIP_PITCH_R",
    "LINK_HIP_ROLL_R",
    "LINK_HIP_YAW_R",
    "LINK_KNEE_PITCH_R",
    "LINK_ANKLE_PITCH_R",
    "LINK_ANKLE_ROLL_R",
    "LINK_FOOT_R",
    "LINK_TORSO_YAW",
    "LINK_SHOULDER_PITCH_L",
    "LINK_SHOULDER_ROLL_L",
    "LINK_SHOULDER_YAW_L",
    "LINK_ELBOW_PITCH_L",
    "LINK_ELBOW_YAW_L",
    "LINK_ELBOW_END_L",
    "LINK_SHOULDER_PITCH_R",
    "LINK_SHOULDER_ROLL_R",
    "LINK_SHOULDER_YAW_R",
    "LINK_ELBOW_PITCH_R",
    "LINK_ELBOW_YAW_R",
    "LINK_ELBOW_END_R",
    "LINK_HEAD_YAW",
]

SKIP_LINKS = {
    "LINK_ELBOW_END_L",
    "LINK_ELBOW_END_R",
}

# Blender 导入/显示的 link（不含 SKIP_LINKS）
LINK_MAPPER = [name for name in MUJOCO_BODY_NAMES if name not in SKIP_LINKS]

BODY_INDEX = {name: idx for idx, name in enumerate(MUJOCO_BODY_NAMES)}
