import time

class RateLimiter:
    """Minimum interval between calls, per tool name"""
    def __init__(self, intervals: dict[str, float], clock=time.monotonic):
        self._intervals = dict(intervals)
        self._clock = clock
        self._last: dict[str, float] = {}

    def check(self, tool_name: str) -> float:
        """Return 0.0 if the call may go ahead (and record it), otherwise the remaining seconds"""
        interval = self._intervals.get(tool_name, 0.0)
        now = self._clock()
        last = self._last.get(tool_name)
        if last is not None and now - last < interval:
            return interval - (now - last)
        self._last[tool_name] = now
        return 0.0  