from drone_agent_core.ratelimit import RateLimiter

class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def make(intervals=None):
    clock = FakeClock()
    return RateLimiter(intervals or {"get_telemetry": 2.0}, clock), clock


def test_first_call_goes_ahead():
    limiter, _ = make()
    assert limiter.check("get_telemetry") == 0.0


def test_second_call_too_soon_reports_wait():
    limiter, clock = make()
    limiter.check("get_telemetry")
    clock.now += 0.5
    assert limiter.check("get_telemetry") == 1.5


def test_call_after_interval_goes_ahead():
    limiter, clock = make()
    limiter.check("get_telemetry")
    clock.now += 2.0
    assert limiter.check("get_telemetry") == 0.0


def test_refused_call_does_not_restart_the_interval():
    limiter, clock = make()
    limiter.check("get_telemetry")
    clock.now += 1.0
    limiter.check("get_telemetry")      # refused
    clock.now += 1.0                    # 2.0 after the first accepted call
    assert limiter.check("get_telemetry") == 0.0


def test_tools_are_limited_independently_and_unknown_tools_are_free():
    limiter, _ = make()
    limiter.check("get_telemetry")
    assert limiter.check("get_detections") == 0.0
    assert limiter.check("get_detections") == 0.0