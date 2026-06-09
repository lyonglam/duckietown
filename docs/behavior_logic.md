# Behavior Logic

This module decides how the Duckiebot should behave based on lane detection and duckie detection.

## Inputs

- lane_error
- lane_confidence
- duckie_detected
- duckie_side
- duckie_confidence
- duckie_size

## States

| Situation               | State                 | Speed     | Target offset |
|-------------------------|-----------------------|----------:|--------------:|
| No duckie               | NORMAL_LANE_FOLLOWING | normal    | center        |
| Duckie on left          | AVOID_LEFT_DUCKIE     | slow      | right         |
| Duckie on right         | AVOID_RIGHT_DUCKIE    | slow      | left          |
| Duckies on both sides   | AVOID_BOTH_DUCKIES    | very slow | center        |
| Lane lost               | SAFETY_STOP           | 0         | center        |
| Duckie too close        | SAFETY_STOP           | 0         | center        |

## Notes

The behavior logic does not directly control the motors yet.  
It outputs a desired speed and target offset.  
Later, the ROS behavior node will connect this logic to the controller and wheel commands.