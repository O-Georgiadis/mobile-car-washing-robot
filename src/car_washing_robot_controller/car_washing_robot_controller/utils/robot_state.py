from enum import StrEnum


class RobotState(StrEnum):
    APPROACH = "APPROACH"
    FOLLOW = "FOLLOW"
    TURNING = "TURNING"
    COMPLETE = "COMPLETE"

