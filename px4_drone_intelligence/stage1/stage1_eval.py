#!/usr/bin/env python3

# Copyright 2026 Kartik Agrawal
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


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