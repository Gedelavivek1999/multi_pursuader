import time
from enum import Enum
import numpy as np


class WatchdogState(Enum):
    NOMINAL = 0
    STAGE1_HOLD = 1      # Stale odometry/target data: hold last safe position
    STAGE2_FAILSAFE = 2  # Extended dropout: descend gently to safe altitude


class TwoStageWatchdog:
    """Monitors odometry and target freshness to enforce two-stage failsafe setpoints."""

    def __init__(
        self,
        stage1_timeout_s: float = 0.25,
        stage2_timeout_s: float = 0.80,
        safe_alt_ned: float = -2.0,
        descent_step_m: float = 0.02,
    ):
        self.stage1_timeout_s = stage1_timeout_s
        self.stage2_timeout_s = stage2_timeout_s
        self.safe_alt_ned = safe_alt_ned
        self.descent_step_m = descent_step_m

        now = time.monotonic()
        self.last_self_update = now
        self.last_target_update = now
        self.state = WatchdogState.NOMINAL
        self.hold_pos = None

    def kick_self(self):
        """Call inside your own drone's local position callback."""
        self.last_self_update = time.monotonic()

    def kick_target(self):
        """Call inside the target/evader/pursuer position callback."""
        self.last_target_update = time.monotonic()

    def step(self, current_pos_ned: np.ndarray, nominal_setpoint_ned: np.ndarray):
        """
        Evaluate watchdog state and return (active_setpoint_ned, state, transitioned).
        """
        now = time.monotonic()
        stale_age = max(now - self.last_self_update, now - self.last_target_update)

        if stale_age > self.stage2_timeout_s:
            next_state = WatchdogState.STAGE2_FAILSAFE
        elif stale_age > self.stage1_timeout_s:
            next_state = WatchdogState.STAGE1_HOLD
        else:
            next_state = WatchdogState.NOMINAL

        transitioned = next_state != self.state
        if transitioned:
            if next_state in (WatchdogState.STAGE1_HOLD, WatchdogState.STAGE2_FAILSAFE):
                self.hold_pos = np.array(current_pos_ned, dtype=float).copy()
            elif next_state == WatchdogState.NOMINAL:
                self.hold_pos = None
            self.state = next_state

        if self.state == WatchdogState.NOMINAL:
            return np.array(nominal_setpoint_ned, dtype=float), self.state, transitioned

        base = self.hold_pos if self.hold_pos is not None else np.array(current_pos_ned, dtype=float)
        if self.state == WatchdogState.STAGE1_HOLD:
            return base.copy(), self.state, transitioned

        # STAGE2_FAILSAFE: hold horizontal XY and step gently toward safe_alt_ned (in NED, -2.0m is 2m up)
        safe_sp = base.copy()
        safe_sp[2] = min(float(safe_sp[2]) + self.descent_step_m, self.safe_alt_ned)
        return safe_sp, self.state, transitioned