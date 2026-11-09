"""SAFE mode.

Purpose:  Fallback for low power and subsystem recovery. Runs the bare minimum
          (health checks, beacon, persistence) and slowly seeks the sun to
          recharge. It is not an attitude mode; sun-seeking is the single
          active-control exception, because it fixes the usual cause of SAFE.
Inputs:   ``StateEstimate``, ``FdirFlags``, ``ExecutiveParams``.
Outputs:  ``ModeCommand`` (target is the slew-limited sun target, or None when
          passive); next mode DETUMBLE once every reason has cleared (OQ-7).
Status:   [NEW] [BLOCKED: OQ-7]
Port from: none

Entry: FDIR forces it from any mode (``FdirFlags.safe_reasons``).

Behavior is picked each tick from what is still usable, because SAFE is often
entered *because* the estimator or a wheel failed:

    SUN_SEEK  attitude + rates valid, wheels healthy, power above the floor
              -> slow wheel PD toward ``control.sun_target``, passed through
                 ``control.slew_limiter`` (``safe_slew_rate``) and run at
                 ``safe_gain_scale`` gain by the controller.
    PASSIVE   attitude invalid, low power, wheels disabled, or wheel momentum
              above ``safe_wheel_momentum_limit`` (SAFE cannot dump it)
              -> no target; actuators stay safed.
    (In eclipse, SUN_SEEK holds the last limited target.)

Exit: every reason cleared for ``safe_clear_hold_s`` and at least
``safe_min_dwell_s`` spent in SAFE, then DETUMBLE. DETUMBLE passes straight
through when rates are already low, so SAFE needs no attitude recovery of its
own. A BOOT_LOOP reason needs its own clearing rule (stable uptime), or SAFE
would exit and re-enter forever.

Open decision: PASSIVE could add a magnetometer-only B-dot when rates are high
and attitude is invalid. Not planned yet.

Related open questions: OQ-7 (SAFE exit criteria and comms), OQ-8, HW (sun sensor).
"""
from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING, Optional

from cloversat.datatypes.command_types import Mode, ModeCommand
from cloversat.executive.state_manager import ModeHandler

if TYPE_CHECKING:
    from cloversat.config.executive_params import ExecutiveParams
    from cloversat.datatypes.state_types import StateEstimate
    from cloversat.executive.fdir_checks import FdirFlags


class SafeBehavior(Enum):
    """What SAFE does with the actuators this tick."""

    SUN_SEEK = "sun_seek"
    PASSIVE = "passive"


def choose_behavior(estimate: StateEstimate, flags: FdirFlags, params: ExecutiveParams) -> SafeBehavior:
    """Pick SUN_SEEK only if attitude, wheels and power all allow it."""
    raise NotImplementedError


def reasons_cleared(flags: FdirFlags) -> bool:
    """True when no SAFE reason is active (before the hold-time check)."""
    raise NotImplementedError


class SafeMode(ModeHandler):
    """SAFE: minimal loads and health checks, slow sun-seek when it can."""

    mode = Mode.SAFE

    def __init__(self, params: ExecutiveParams) -> None:
        raise NotImplementedError

    def enter(self, t: float) -> None:
        # Must seed the limited target from the *current* attitude, otherwise
        # the first tick commands a step toward the sun.
        raise NotImplementedError

    def step(self, estimate: StateEstimate, flags: FdirFlags, t: float) -> ModeCommand:
        raise NotImplementedError

    def next_mode(self, estimate: StateEstimate, flags: FdirFlags, t: float) -> Optional[Mode]:
        raise NotImplementedError

    def exit(self, t: float) -> None:
        raise NotImplementedError
