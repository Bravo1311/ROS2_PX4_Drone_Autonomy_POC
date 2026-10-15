from dataclasses import dataclass, fields
import yaml

class ProfileError(Exception):
    """Raised when a profile in invalid"""

AUTONOMY_MODES = ("observe", "supervised")

@dataclass(frozen=True)
class Policy:
    start_armed: bool = False
    autonomy: str = "supervised"
    approval_ttl_s: float = 60

    def __post_init__(self):
        if not isinstance(self.start_armed, bool):
            raise ProfileError(f"start_armed must be a boolean, got {self.start_armed!r}")
        if not isinstance(self.autonomy, str):
            raise ProfileError(f"autonomy must be a string and one of {AUTONOMY_MODES}, got {self.autonomy!r}")
        if self.autonomy not in AUTONOMY_MODES:
            raise ProfileError(f"autonomy must be one of {AUTONOMY_MODES}, got {self.autonomy!r}")
        ttl = self.approval_ttl_s
        if isinstance(ttl, bool) or not isinstance(ttl, (int, float)) or ttl <= 0:
            raise ProfileError(
                f"approval_ttl_s must be a number above 0, got {ttl!r}"
            )

def load_policy(data: dict) -> Policy:
    allowed = {f.name for f in fields(Policy)}
    unknown = set(data) - allowed
    if unknown:
        raise ProfileError(
            f"Unknown policy keys(s): {sorted(unknown)}; allowed: {sorted(allowed)}"
        )
    return Policy(**data)

@dataclass(frozen=True)
class Profile:
    policy: Policy

def load_profile(path) -> Profile:
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        raise ProfileError("profile must be a mapping at the top level")
    unknown = set(raw) - {"policy"}
    if unknown:
        raise ProfileError(f"unknown top-level section(s): {sorted(unknown)}")
    section = raw.get("policy") or {}
    if not isinstance(section, dict):
        raise ProfileError("'policy' must be a mapping")
    return Profile(policy=load_policy(section))