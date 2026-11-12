"""SAFE mode.

Purpose:  Fallback for low power and subsystem recovery. Runs the bare minimum
          (health checks, beacon, persistence) and slowly seeks the sun to
          recharge. It is not an attitude mode; sun-seeking is the single
          active-control exception, because it fixes the usual cause of SAFE.
Inputs:   ``StateEstimate``, ``FdirFlags``, ``ExecutiveParams``.
Outputs:  ``ModeCommand`` (target is the slew-limited sun target, or None when
          passive); next mode DETUMBLE once every reason has cleared.
Status:   [NEW]  Phases and exit are done; sun-seek is not (see below).
Port from: none

Entry: FDIR forces it from any mode (``FdirFlags.safe_reasons``), or BOOT sends
it there on a boot loop or bad params.

Two explicit pieces of state, both drawn in ``docs/state_flow.md``:

``SafePhase`` (a real state machine; the legal moves are
``SAFE_PHASE_TRANSITIONS``):
    ACTIVE    at least one reason is present, or FDIR is not valid yet.
    CLEARING  every reason is gone; the hold timer is running. If a reason comes
              back, go to ACTIVE and the timer is discarded.
Exit to DETUMBLE needs CLEARING for ``safe_clear_hold_s`` and at least
``safe_min_dwell_s`` in SAFE overall, so one clean tick can't bounce us out.
DETUMBLE passes straight through when rates are already low, so SAFE needs no
attitude recovery of its own.

``SafeBehavior`` is not state. It is recomputed from scratch every tick by
``choose_behavior`` from what is usable right now, because SAFE is often entered
*because* the estimator or a wheel failed:
    SUN_SEEK  attitude + rates + wheels valid, wheels not disabled, power above
              the floor, wheel momentum under ``safe_wheel_momentum_limit``
              -> slow wheel PD toward ``control.sun_target`` through
                 ``control.slew_limiter``, at ``safe_gain_scale`` gain.
    PASSIVE   anything else -> no target; actuators stay safed.

TODO(sun-seek): the SUN_SEEK command needs ``limit_slew`` and the controller's
    low-gain path. Until then ``step`` raises if it would be chosen, and
    ``safe_wheel_momentum_limit`` defaults to None, which forces PASSIVE.

Assumes FDIR names the wheel subsystem ``"wheels"`` in ``disabled_subsystems``.

Open decision: PASSIVE could add a magnetometer-only B-dot when rates are high
and attitude is invalid. Not planned yet.

Related open questions: OQ-7 (comms, passive vs B-dot), HW (sun sensor).
"""
from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING, Optional

import numpy as np

from cloversat.datatypes.command_types import Mode, ModeCommand
from cloversat.executive.state_manager import ModeHandler

if TYPE_CHECKING:
    from cloversat.config.executive_params import ExecutiveParams
    from cloversat.datatypes.state_types import StateEstimate
    from cloversat.executive.fdir_checks import FdirFlags


class SafePhase(Enum):
    """Whether SAFE is holding for a reason or counting down to exit."""

    ACTIVE = "active"
    CLEARING = "clearing"


SAFE_PHASE_TRANSITIONS: dict[SafePhase, frozenset[SafePhase]] = {
    SafePhase.ACTIVE: frozenset({SafePhase.CLEARING}),
    SafePhase.CLEARING: frozenset({SafePhase.ACTIVE}),
}


class SafeBehavior(Enum):
    """What SAFE does with the actuators this tick. Derived, never stored."""

    SUN_SEEK = "sun_seek"
    PASSIVE = "passive"


def choose_behavior(estimate: StateEstimate, flags: FdirFlags, params: ExecutiveParams) -> SafeBehavior:
    """SUN_SEEK only if attitude, wheels and power all allow it."""
    limit = params.safe_wheel_momentum_limit
    if limit is None:
        return SafeBehavior.PASSIVE  # can't judge saturation, so don't spin up
    usable = (
        estimate.attitude_valid
        and estimate.rates_valid
        and estimate.wheels_valid
        and "wheels" not in flags.disabled_subsystems
        and not flags.low_power
        and float(np.linalg.norm(estimate.h_wheels_body)) < limit
    )
    return SafeBehavior.SUN_SEEK if usable else SafeBehavior.PASSIVE


def reasons_cleared(flags: FdirFlags) -> bool:
    """True when FDIR has evaluated and no SAFE reason is active.

    ``flags.valid`` matters: before FDIR's first evaluation an empty reason set
    means "not checked yet", not "all clear".
    """
    return flags.valid and not flags.force_safe and not flags.safe_reasons


class SafeMode(ModeHandler):
    """SAFE: minimal loads and health checks, slow sun-seek when it can."""

    mode = Mode.SAFE

    def __init__(self, params: ExecutiveParams) -> None:
        self._params = params
        self._phase = SafePhase.ACTIVE
        self._entered_at = 0.0
        self._clear_since: Optional[float] = None  # set only while CLEARING

    @property
    def phase(self) -> SafePhase:
        return self._phase

    def enter(self, t: float) -> None:
        self._phase = SafePhase.ACTIVE
        self._entered_at = t
        self._clear_since = None
        # TODO(sun-seek): seed the slew limiter from the *current* attitude,
        # otherwise the first tick commands a step toward the sun.

    def step(self, estimate: StateEstimate, flags: FdirFlags, t: float) -> ModeCommand:
        self._update_phase(flags, t)
        if choose_behavior(estimate, flags, self._params) is SafeBehavior.SUN_SEEK:
            raise NotImplementedError("SAFE sun-seek: needs limit_slew and the controller's low-gain path")
        return ModeCommand(mode=Mode.SAFE)

    def next_mode(self, estimate: StateEstimate, flags: FdirFlags, t: float) -> Optional[Mode]:
        # Phase is last tick's, so also require the reasons to be clear *now*:
        # a reason that just came back must stop an exit that was about to fire.
        if self._phase is not SafePhase.CLEARING or not reasons_cleared(flags):
            return None
        hold, dwell = self._params.safe_clear_hold_s, self._params.safe_min_dwell_s
        if hold is None or dwell is None:
            # SAFE is the fallback for faults, so it must not raise on a bad
            # param. With no usable timer it simply stays.
            return None
        held = t - self._clear_since >= hold
        dwelled = t - self._entered_at >= dwell
        return Mode.DETUMBLE if (held and dwelled) else None

    def exit(self, t: float) -> None:
        pass

    def _update_phase(self, flags: FdirFlags, t: float) -> None:
        cleared = reasons_cleared(flags)
        if self._phase is SafePhase.ACTIVE and cleared:
            self._goto(SafePhase.CLEARING)
            self._clear_since = t
        elif self._phase is SafePhase.CLEARING and not cleared:
            self._goto(SafePhase.ACTIVE)
            self._clear_since = None

    def _goto(self, nxt: SafePhase) -> None:
        if nxt not in SAFE_PHASE_TRANSITIONS[self._phase]:
            raise RuntimeError(f"SAFE phase {self._phase.name} -> {nxt.name} is not allowed")
        self._phase = nxt
