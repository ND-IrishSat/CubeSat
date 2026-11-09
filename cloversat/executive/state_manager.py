"""Operating-mode state machine.

Purpose:  Pick the active mode each tick. Checks the active mode's exit
          criteria against ``StateEstimate`` and FDIR flags only, and outputs
          a ``ModeCommand``.
Inputs:   ``StateEstimate``, ``FdirFlags``, time.
Outputs:  ``ModeCommand``.
Status:   [NEW]
Port from: legacy/Simulator/simulator.py ``check_state`` (reference only),
           legacy/params.py ``PROTOCOL_MAP`` (old mode list)

Modes (``datatypes.command_types.Mode``): BOOT, DETUMBLE, ORIENT_TRANSITION,
OPERATIONAL, SAFE. Each is implemented by one ``*_mode.py`` as a
``ModeHandler``. FDIR can force SAFE from any mode.

# TODO(bug): legacy check_state shares one if/elif chain between the torquer
#   timer and the state checks, so with ACCURATE_MAG_READINGS on the state
#   checks never run. Keep timing (control/actuation_scheduler) separate.
# TODO(bug): legacy detumble exit uses the *true* state. Exit criteria here
#   must only read StateEstimate.

Related open questions: OQ-4, OQ-5, OQ-6, OQ-7.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, Optional

from cloversat.datatypes.command_types import Mode, ModeCommand

if TYPE_CHECKING:
    from cloversat.config.executive_params import ExecutiveParams
    from cloversat.datatypes.state_types import StateEstimate
    from cloversat.executive.fdir_checks import FdirFlags


@dataclass(frozen=True)
class Transition:
    """One completed mode change, for telemetry and persistence.

    Attributes:
        t: Time of the change [s].
        from_mode: Mode that was active.
        to_mode: Mode that became active.
        reason: Why: ``"exit criteria"``, or the FDIR reasons, or the caller's
            text for ``force_mode``.
    """

    t: float
    from_mode: Mode
    to_mode: Mode
    reason: str


class ModeHandler(ABC):
    """One operating mode. Implemented by each ``*_mode.py``."""

    mode: Mode

    @abstractmethod
    def enter(self, t: float) -> None:
        """Called once when the mode becomes active."""
        ...

    @abstractmethod
    def step(self, estimate: StateEstimate, flags: FdirFlags, t: float) -> ModeCommand:
        """Return this tick's ``ModeCommand``."""
        ...

    @abstractmethod
    def next_mode(self, estimate: StateEstimate, flags: FdirFlags, t: float) -> Optional[Mode]:
        """Return the mode to transition to, or None to stay."""
        ...

    @abstractmethod
    def exit(self, t: float) -> None:
        """Called once when the mode stops being active."""
        ...


class StateManager:
    """Runs the mode state machine over a set of ``ModeHandler``s.

    Contains no mode logic of its own; each handler decides its exit criteria.
    It does no I/O either: the caller persists ``mode`` (or reacts to
    ``on_transition``) so this class stays testable with stub handlers.

    At most one transition happens per ``step``. A chain of modes whose exit
    criteria are already met therefore takes one tick per hop, which keeps each
    mode's ``enter`` visible to the controller for at least one tick.
    """

    def __init__(
        self,
        params: ExecutiveParams,
        handlers: dict[Mode, ModeHandler],
        initial: Mode,
        on_transition: Optional[Callable[[Transition], None]] = None,
    ) -> None:
        missing = [m.name for m in Mode if m not in handlers]
        if missing:
            raise ValueError(f"no handler for mode(s): {', '.join(missing)}")
        if initial not in handlers:
            raise ValueError(f"initial mode {initial} has no handler")
        self._params = params
        self._handlers = handlers
        self._mode = initial
        self._on_transition = on_transition
        # The initial handler is entered on the first step(), because enter()
        # needs a time and the constructor doesn't have one.
        self._started = False
        self._last_transition: Optional[Transition] = None

    @property
    def mode(self) -> Mode:
        """Currently active mode."""
        return self._mode

    @property
    def last_transition(self) -> Optional[Transition]:
        """Most recent mode change, or None if there hasn't been one."""
        return self._last_transition

    def step(self, estimate: StateEstimate, flags: FdirFlags, t: float) -> ModeCommand:
        """Check transitions (FDIR SAFE first), then step the active mode."""
        if not self._started:
            self._started = True
            self._handlers[self._mode].enter(t)

        if flags.force_safe and self._mode is not Mode.SAFE:
            # FDIR overrides whatever the active mode wanted; its own
            # next_mode is not consulted.
            self._transition(Mode.SAFE, t, self._fdir_reason(flags))
        else:
            # Includes SAFE itself: SAFE sees the flags and decides when its
            # reasons have cleared.
            nxt = self._handlers[self._mode].next_mode(estimate, flags, t)
            if nxt is not None and nxt is not self._mode:
                self._transition(nxt, t, "exit criteria")

        command = self._handlers[self._mode].step(estimate, flags, t)
        if command.mode is not self._mode:
            raise RuntimeError(f"{self._mode} handler returned a {command.mode} command")
        return command

    def force_mode(self, mode: Mode, t: float, reason: str) -> None:
        """Transition immediately (used by FDIR and boot re-entry).

        Forcing the mode that is already active does nothing, so it does not
        re-run ``enter`` or reset that mode's timers.
        """
        if mode is self._mode:
            return
        if mode not in self._handlers:
            raise ValueError(f"mode {mode} has no handler")
        self._transition(mode, t, reason)

    def _transition(self, to_mode: Mode, t: float, reason: str) -> None:
        from_mode = self._mode
        if self._started:
            self._handlers[from_mode].exit(t)
        else:
            # Forced before the first step: the initial handler was never
            # entered, so there is nothing to exit.
            self._started = True
        self._mode = to_mode
        self._handlers[to_mode].enter(t)
        self._last_transition = Transition(t, from_mode, to_mode, reason)
        if self._on_transition is not None:
            self._on_transition(self._last_transition)

    @staticmethod
    def _fdir_reason(flags: FdirFlags) -> str:
        reasons = sorted(r.value for r in flags.safe_reasons)
        return "fdir: " + (", ".join(reasons) if reasons else "force_safe")
