from drone_agent_core.robot import Detection, FakeRobot, Telemetry


def test_fake_robot_returns_scripted_telemetry():
    robot = FakeRobot(telemetry=Telemetry(1.0, 2.0, 5.0, 40.0, True, "OFFBOARD"))
    assert robot.get_telemetry().battery_pct == 40.0


def test_fake_robot_defaults_are_a_landed_disarmed_drone():
    t = FakeRobot().get_telemetry()
    assert t.armed is False and t.altitude_m == 0.0


def test_detections_and_graph_come_back_as_copies():
    robot = FakeRobot(detections=[Detection("person", 0.9)], graph={"topics": ["/a"]})
    robot.get_detections().clear()
    robot.list_ros_graph("topics").clear()
    assert len(robot.get_detections()) == 1
    assert robot.list_ros_graph("topics") == ["/a"]


def test_unknown_graph_kind_is_empty_and_calls_are_recorded():
    robot = FakeRobot()
    assert robot.list_ros_graph("services") == []
    assert robot.calls == ["list_ros_graph"]