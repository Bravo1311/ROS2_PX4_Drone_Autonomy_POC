#!/usr/bin/env python3
import json
from stage1_target_selection import query_ollama

IMAGE = "/home/bravo1311/px4_ros2_ws/src/ROS2-PX4_Drone_Teleoperation_Using_Joystick/px4_drone_intelligence/maps/map.jpg"

TESTS = [
    {
        "instruction": "Go to marker M1",
        "expected": "M1",
    },
    {
        "instruction": "Go to the marker between the walls",
        "expected": "M3",
    },
    {
        "instruction": "Go to the marker on the right",
        "expected": "M1",
    },
    {
        "instruction": "Go to the marker in the left open area",
        "expected": "M0",
    },
    {
        "instruction": "Go to the upper-left marker",
        "expected": "M2",
    },
]


def main():
    correct = 0

    for i, test in enumerate(TESTS, start=1):
        result = query_ollama(IMAGE, test["instruction"])
        pred = result["target_marker"]
        ok = pred == test["expected"]
        correct += int(ok)

        print(f"\nTest {i}")
        print(f"Instruction: {test['instruction']}")
        print(f"Expected:    {test['expected']}")
        print(f"Predicted:   {pred}")
        print(f"Reason:      {result.get('reason_short', '')}")
        print(f"Result:      {'PASS' if ok else 'FAIL'}")

    print(f"\nScore: {correct}/{len(TESTS)}")


if __name__ == "__main__":
    main()