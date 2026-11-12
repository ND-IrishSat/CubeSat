"""BOOT mode.

Purpose:  Load ``PersistedState``, restore time, record this reset, verify
          params, and wait for FDIR's first valid flags. Then leave for
          DETUMBLE, or SAFE on a boot loop or bad params.
Inputs:   ``PersistedState`` (via persistence), a ``restore_time`` callable,
          ``FdirFlags``, ``ExecutiveParams``.
Outputs:  ``ModeCommand`` (no pointing); next mode.
Status:   [NEW]
Port from: none

Always re-enters through DETUMBLE, never "where we left off". After a reset the
rates are unknown (we may have tumbled while the computer was down, or the reset
may itself be from a collision), and DETUMBLE passes straight through when
rates are already low. Low power needs no SAFE-first step: FDIR can force SAFE
from DETUMBLE, which is why BOOT waits for FDIR's first valid flags.

BOOT is an explicit sequence of ``BootStep``s with the legal order in
``BOOT_STEP_TRANSITIONS`` (drawn in ``docs/state_flow.md``). Each step is one
method that returns the step to go to; returning its own step means "keep
waiting". Steps that don't wait run back to back inside one ``step()`` call.

Not handled yet:
    - Which subsystem to disable on a boot loop (nothing identifies it yet).
    - A clock that can't be restored: the reset is not recorded, so a boot loop
      with a dead RTC goes undetected.
    - The ``deployed`` flag when ``state_trusted`` is False has to be re-read
      from the sensor before any burn; that check belongs to deployment.

Later:
    - Scheduled reboots.
"""
from __future__ import annotations

from dataclasses import replace
from enum import Enum
from typing import TYPE_CHECKING, Callable, Optional

from cloversat.datatypes.command_types import Mode, ModeCommand
from cloversat.datatypes.state_types import PersistedState
from cloversat.executive.param_checks import params_checksum, validate_params
from cloversat.executive.state_manager import ModeHandler

if TYPE_CHECKING:
    from cloversat.config.executive_params import ExecutiveParams
    from cloversat.datatypes.state_types import StateEstimate
    from cloversat.executive.fdir_checks import FdirFlags
    from cloversat.executive.persistence import Persistence


class BootStep(Enum):
    """Where BOOT is in its start-up sequence."""

    LOAD_STATE = "load_state"
    RESTORE_TIME = "restore_time"
    RECORD_RESET = "record_reset"
    VERIFY_PARAMS = "verify_params"
    AWAIT_HEALTH = "await_health"
    DONE = "done"


BOOT_STEP_TRANSITIONS: dict[BootStep, frozenset[BootStep]] = {
    BootStep.LOAD_STATE: frozenset({BootStep.RESTORE_TIME}),
    BootStep.RESTORE_TIME: frozenset({BootStep.RECORD_RESET}),
    BootStep.RECORD_RESET: frozenset({BootStep.VERIFY_PARAMS}),
    BootStep.VERIFY_PARAMS: frozenset({BootStep.AWAIT_HEALTH}),
    BootStep.AWAIT_HEALTH: frozenset({BootStep.DONE}),
    BootStep.DONE: frozenset(),
}


def is_boot_loop(state: PersistedState, t: float, params: ExecutiveParams) -> bool:
    """True if ``boot_loop_resets`` or more boots fall within the window ending at ``t``.

    ``t`` is RTC time, the same clock as ``state.recent_reset_times``. The
    result depends only on the arguments, so FDIR can re-evaluate it every
    tick: as time passes without a reset the window slides past the old boots
    and the loop clears by itself, with no latch to release.
    """
    window = params.boot_loop_window_s
    recent = [x for x in state.recent_reset_times if 0 <= t - x <= window]
    return len(recent) >= params.boot_loop_resets


def choose_reentry_mode(boot_loop: bool, params_ok: bool) -> Mode:
    """DETUMBLE normally; SAFE if the boot is looping or the params are unusable."""
    return Mode.SAFE if (boot_loop or not params_ok) else Mode.DETUMBLE


class BootMode(ModeHandler):
    """BOOT: run the start-up sequence, then pick DETUMBLE or SAFE.

    Results are left as attributes for ``main.py`` and FDIR to read:
    ``state`` (what was loaded), ``state_trusted`` (False if no valid copy
    existed), ``boot_loop``, ``params_ok``, ``params_checksum`` (FDIR's
    baseline for spotting later changes) and ``copies_saved``.
    """

    mode = Mode.BOOT

    def __init__(
        self,
        params: ExecutiveParams,
        persistence: Persistence,
        restore_time: Callable[[], Optional[float]],
    ) -> None:
        """``restore_time`` sets the system clock from the RTC and returns that
        time [s], or None if there is no trustworthy source."""
        self._params = params
        self._persistence = persistence
        self._restore_time = restore_time
        self._actions = {
            BootStep.LOAD_STATE: self._load_state,
            BootStep.RESTORE_TIME: self._restore_clock,
            BootStep.RECORD_RESET: self._record_reset,
            BootStep.VERIFY_PARAMS: self._verify_params,
            BootStep.AWAIT_HEALTH: self._await_health,
        }
        self._reset_run()

    def _reset_run(self) -> None:
        self._boot_step = BootStep.LOAD_STATE
        self._wall_time: Optional[float] = None
        self._await_started = 0.0
        self.state = PersistedState()
        self.state_trusted = False
        self.boot_loop = False
        self.params_ok = False
        self.params_checksum = 0
        self.copies_saved = 0

    @property
    def boot_step(self) -> BootStep:
        return self._boot_step

    def enter(self, t: float) -> None:
        self._reset_run()

    def step(self, estimate: StateEstimate, flags: FdirFlags, t: float) -> ModeCommand:
        # Run every step that doesn't need to wait. A step that returns its own
        # value is waiting (only AWAIT_HEALTH does), so stop until next tick.
        while self._boot_step is not BootStep.DONE:
            nxt = self._actions[self._boot_step](flags, t)
            if nxt is self._boot_step:
                break
            self._goto(nxt)
        return ModeCommand(mode=Mode.BOOT)

    def next_mode(self, estimate: StateEstimate, flags: FdirFlags, t: float) -> Optional[Mode]:
        if self._boot_step is not BootStep.DONE:
            return None
        return choose_reentry_mode(self.boot_loop, self.params_ok)

    def exit(self, t: float) -> None:
        pass

    def _goto(self, nxt: BootStep) -> None:
        if nxt not in BOOT_STEP_TRANSITIONS[self._boot_step]:
            raise RuntimeError(f"BOOT step {self._boot_step.name} -> {nxt.name} is not allowed")
        self._boot_step = nxt

    # --- one method per step; each returns the step to go to next ---

    def _load_state(self, flags: FdirFlags, t: float) -> BootStep:
        loaded = self._persistence.load()
        self.state_trusted = loaded is not None
        # With no valid copy we start from blank state. `state_trusted` tells
        # the rest of the system not to believe its `deployed` flag.
        self.state = loaded if loaded is not None else PersistedState()
        return BootStep.RESTORE_TIME

    def _restore_clock(self, flags: FdirFlags, t: float) -> BootStep:
        self._wall_time = self._restore_time()
        return BootStep.RECORD_RESET

    def _record_reset(self, flags: FdirFlags, t: float) -> BootStep:
        now = self._wall_time
        if now is None:
            return BootStep.VERIFY_PARAMS  # can't timestamp this boot; see docstring
        window = self._params.boot_loop_window_s
        # Boots stamped in the future mean the clock jumped; they can't be
        # trusted, so they are dropped along with the ones outside the window.
        kept = [x for x in self.state.recent_reset_times if 0 <= now - x <= window]
        # Keep only as many as detection can use, so the list stays small.
        history = (kept + [now])[-self._params.boot_loop_resets:]
        self.state = replace(self.state, recent_reset_times=history, t_saved=now)
        self.boot_loop = is_boot_loop(self.state, now, self._params)
        # Saved right away: if the next thing that happens is another reset,
        # this one must still be counted.
        self.copies_saved = self._persistence.save(self.state)
        return BootStep.VERIFY_PARAMS

    def _verify_params(self, flags: FdirFlags, t: float) -> BootStep:
        self.params_ok = not validate_params(self._params)
        self.params_checksum = params_checksum(self._params)
        self._await_started = t
        return BootStep.AWAIT_HEALTH

    def _await_health(self, flags: FdirFlags, t: float) -> BootStep:
        timed_out = t - self._await_started >= self._params.boot_health_wait_s
        return BootStep.DONE if (flags.valid or timed_out) else BootStep.AWAIT_HEALTH
