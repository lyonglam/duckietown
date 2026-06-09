from enum import Enum


class DrivingState(Enum):
    NORMAL_LANE_FOLLOWING = "NORMAL_LANE_FOLLOWING"
    AVOID_LEFT_DUCKIE = "AVOID_LEFT_DUCKIE"
    AVOID_RIGHT_DUCKIE = "AVOID_RIGHT_DUCKIE"
    AVOID_BOTH_DUCKIES = "AVOID_BOTH_DUCKIES"
    SAFETY_STOP = "SAFETY_STOP"


def behavior_decision(
    lane_error: float,
    lane_confidence: float,
    duckie_detected: bool,
    duckie_side: str,
    duckie_confidence: float,
    duckie_size: float,
):
    """
    Decides what the Duckiebot should do based on lane and duckie information.

    Inputs:
    - lane_error: how far the robot is from lane center
    - lane_confidence: how sure we are that the lane is detected
    - duckie_detected: True if duckie is visible
    - duckie_side: "none", "left", "right", "both", or "center"
    - duckie_confidence: how sure we are that a duckie was detected
    - duckie_size: approximate size of duckie in image; bigger means closer

    Outputs:
    - state: current driving behavior
    - speed: target speed
    - target_offset: where to aim inside the lane
    - reason: explanation for debugging
    """

    NORMAL_SPEED = 0.30
    SLOW_SPEED = 0.18
    VERY_SLOW_SPEED = 0.12

    CENTER_OFFSET = 0.0
    LEFT_OFFSET = -0.15
    RIGHT_OFFSET = 0.15

    MIN_LANE_CONFIDENCE = 0.50
    MIN_DUCKIE_CONFIDENCE = 0.60
    UNSAFE_DUCKIE_SIZE = 0.35

    if lane_confidence < MIN_LANE_CONFIDENCE:
        return {
            "state": DrivingState.SAFETY_STOP.value,
            "speed": 0.0,
            "target_offset": CENTER_OFFSET,
            "reason": "lane confidence too low",
        }

    if duckie_detected and duckie_confidence >= MIN_DUCKIE_CONFIDENCE:
        if duckie_size >= UNSAFE_DUCKIE_SIZE:
            return {
                "state": DrivingState.SAFETY_STOP.value,
                "speed": 0.0,
                "target_offset": CENTER_OFFSET,
                "reason": "duckie too close or unsafe",
            }

        if duckie_side == "left":
            return {
                "state": DrivingState.AVOID_LEFT_DUCKIE.value,
                "speed": SLOW_SPEED,
                "target_offset": RIGHT_OFFSET,
                "reason": "duckie on left, shifting right",
            }

        if duckie_side == "right":
            return {
                "state": DrivingState.AVOID_RIGHT_DUCKIE.value,
                "speed": SLOW_SPEED,
                "target_offset": LEFT_OFFSET,
                "reason": "duckie on right, shifting left",
            }

        if duckie_side == "both":
            return {
                "state": DrivingState.AVOID_BOTH_DUCKIES.value,
                "speed": VERY_SLOW_SPEED,
                "target_offset": CENTER_OFFSET,
                "reason": "duckies on both sides, slowing down",
            }

        if duckie_side == "center":
            return {
                "state": DrivingState.SAFETY_STOP.value,
                "speed": 0.0,
                "target_offset": CENTER_OFFSET,
                "reason": "duckie in center/unsafe path",
            }

    return {
        "state": DrivingState.NORMAL_LANE_FOLLOWING.value,
        "speed": NORMAL_SPEED,
        "target_offset": CENTER_OFFSET,
        "reason": "normal lane following",
    }


if __name__ == "__main__":
    result = behavior_decision(
        lane_error=0.0,
        lane_confidence=0.9,
        duckie_detected=True,
        duckie_side="left",
        duckie_confidence=0.8,
        duckie_size=0.2,
    )

    print(result)