from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [3.13]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0]


def joint_names() -> List[str]:
    return [
        "shoulder_joint",
        "upperarm_joint",
        "forearm_joint",
        "wrist_joint",
        "frame_joint",
    ]


def base_link_name() -> str:
    return "base_link"


def end_effector_name() -> str:
    return "worm_link"


def gripper_joint_names() -> List[str]:
    return [
        "worm_joint",
    ]
