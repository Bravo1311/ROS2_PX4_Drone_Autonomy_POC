class LastKnown:
    """Remembers the latest value and hands it out while it is recent enough"""
    
    def __init__(self, tolerance_s: float):
        self._tolerance_s = tolerance_s
        self._value = None
        self._at: float | None = None
        
    def update(self, value, now: float) -> None:
        self._value = value
        self._at = now
        
    def get(self, now: float):
        """The value, or None if there never was one or it is older than the tolerance"""
        if self._at is None or now - self._at > self._tolerance_s:
            return None
        return self._value