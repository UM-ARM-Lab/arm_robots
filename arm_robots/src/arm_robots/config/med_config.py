import numpy as np

# Joint Names

ARM_JOINT_NAMES = (
    "med_kuka_joint_1",
    "med_kuka_joint_2",
    "med_kuka_joint_3",
    "med_kuka_joint_4",
    "med_kuka_joint_5",
    "med_kuka_joint_6",
    "med_kuka_joint_7",
)

# From Med manual 4.3.2 and copied to kuka_iiwa_interface joint_limits.yaml
KUKA_MED_MAX_JOINT_VELOCITIES_DEG_S = (85.0, 85.0, 100.0, 75.0, 130.0, 135.0, 135.0)
KUKA_MED_MAX_JOINT_VELOCITIES = list(s * np.pi / 180 for s in KUKA_MED_MAX_JOINT_VELOCITIES_DEG_S)
KUKA_MED_MAX_JOINT_ACCEL_DEG_S2 = (50.0, 50.0, 100.0, 100.0, 1000.0, 650.0, 1500.0)
KUKA_MED_MAX_JOINT_ACCEL = list(s * np.pi / 180 for s in KUKA_MED_MAX_JOINT_ACCEL_DEG_S2)
