from behavior_logic import behavior_decision


def run_tests():
    tests = [
        {
            "name": "normal driving",
            "input": {
                "lane_error": 0.0,
                "lane_confidence": 0.9,
                "duckie_detected": False,
                "duckie_side": "none",
                "duckie_confidence": 0.0,
                "duckie_size": 0.0,
            },
            "expected_state": "NORMAL_LANE_FOLLOWING",
        },
        {
            "name": "duckie on left",
            "input": {
                "lane_error": 0.0,
                "lane_confidence": 0.9,
                "duckie_detected": True,
                "duckie_side": "left",
                "duckie_confidence": 0.8,
                "duckie_size": 0.2,
            },
            "expected_state": "AVOID_LEFT_DUCKIE",
        },
        {
            "name": "duckie on right",
            "input": {
                "lane_error": 0.0,
                "lane_confidence": 0.9,
                "duckie_detected": True,
                "duckie_side": "right",
                "duckie_confidence": 0.8,
                "duckie_size": 0.2,
            },
            "expected_state": "AVOID_RIGHT_DUCKIE",
        },
        {
            "name": "lane lost",
            "input": {
                "lane_error": 0.0,
                "lane_confidence": 0.2,
                "duckie_detected": False,
                "duckie_side": "none",
                "duckie_confidence": 0.0,
                "duckie_size": 0.0,
            },
            "expected_state": "SAFETY_STOP",
        },
        {
            "name": "duckie too close",
            "input": {
                "lane_error": 0.0,
                "lane_confidence": 0.9,
                "duckie_detected": True,
                "duckie_side": "left",
                "duckie_confidence": 0.9,
                "duckie_size": 0.5,
            },
            "expected_state": "SAFETY_STOP",
        },
    ]

    for test in tests:
        result = behavior_decision(**test["input"])
        actual_state = result["state"]

        if actual_state == test["expected_state"]:
            print(f"PASS: {test['name']}")
        else:
            print(f"FAIL: {test['name']}")
            print(f"Expected: {test['expected_state']}")
            print(f"Got: {actual_state}")


if __name__ == "__main__":
    run_tests()