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

The legal changes are the ``MODE_TRANSITIONS`` table below, and every change
is checked against it. ``docs/state_flow.md`` draws the same table; a test
fails if the two differ, so edit both together.

A handler that raises is quarantined, not allowed to take the control loop
down: see ``StateManager`` for the policy.

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

from cloversat.datatypes.command_types import Mode, ModeCommand, SubMode

if TYPE_CHECKING:
    from cloversat.config.executive_params import ExecutiveParams
    from cloversat.datatypes.state_types import StateEstimate
    from cloversat.executive.fdir_checks import FdirFlags


# Which mode may follow which. This is the single source of truth for the mode
# graph; nothing else decides what is legal. SAFE is reachable from every other
# mode because FDIR can force it at any time. SAFE only ever leaves through
# DETUMBLE, which passes straight through when rates are already low.
MODE_TRANSITIONS: dict[Mode, frozenset[Mode]] = {
    Mode.BOOT: frozenset({Mode.DETUMBLE, Mode.SAFE}),
    Mode.DETUMBLE: frozenset({Mode.ORIENT_TRANSITION, Mode.SAFE}),
    Mode.ORIENT_TRANSITION: frozenset({Mode.OPERATIONAL, Mode.DETUMBLE, Mode.SAFE}),
    Mode.OPERATIONAL: frozenset({Mode.SAFE}),
    Mode.SAFE: frozenset({Mode.DETUMBLE}),
}

# OPERATIONAL sub-modes: SUN_POINT is home, and the others preempt it and
# always return to it. They never switch directly to each other. Enforced by
# ``OperationalMode`` once OQ-6 (how it chooses) is settled.
SUBMODE_TRANSITIONS: dict[SubMode, frozenset[SubMode]] = {
    SubMode.SUN_POINT: frozenset({SubMode.GPS_POINT, SubMode.GROUND_POINT, SubMode.MOMENTUM_DUMP}),
    SubMode.GPS_POINT: frozenset({SubMode.SUN_POINT}),
    SubMode.GROUND_POINT: frozenset({SubMode.SUN_POINT}),
    SubMode.MOMENTUM_DUMP: frozenset({SubMode.SUN_POINT}),
}


class IllegalTransition(ValueError):
    """A mode change that ``MODE_TRANSITIONS`` does not allow."""


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


@dataclass(frozen=True)
class HandlerFault:
    """A handler call that raised. Kept as text because nothing keeps tracebacks
    after launch; this and the transition reason are what telemetry can carry.

    Attributes:
        t: Time of the fault [s].
        mode: Mode whose handler raised.
        where: ``"enter"``, ``"step"``, ``"next_mode"``, ``"exit"`` or
            ``"on_transition"``.
        error: ``repr`` of the exception.
    """

    t: float
    mode: Mode
    where: str
    error: str


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

    Handler faults: any ``Exception`` from a handler (a missing param, a bug, a
    command for the wrong mode, an illegal ``next_mode`` answer) is recorded in
    ``last_fault`` and quarantines that mode by moving to SAFE. If SAFE itself
    raises there is nowhere further to fall back, so the tick returns a bare
    SAFE command with no target and the fault is recorded. A SAFE that keeps
    failing shows up as ``consecutive_fault_ticks`` growing; the caller
    (``main.py``) decides when to stop trusting us and let the watchdog reset.
    Only ``Exception`` is caught, never ``KeyboardInterrupt`` or ``SystemExit``.
    A bad ``force_mode`` request is the caller's bug and still raises.
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
        self._last_fault: Optional[HandlerFault] = None
        self._fault_count = 0
        self._consecutive_fault_ticks = 0
        self._faulted_this_tick = False

    @property
    def mode(self) -> Mode:
        """Currently active mode."""
        return self._mode

    @property
    def last_transition(self) -> Optional[Transition]:
        """Most recent mode change, or None if there hasn't been one."""
        return self._last_transition

    @property
    def last_fault(self) -> Optional[HandlerFault]:
        """Most recent handler fault, or None if there hasn't been one."""
        return self._last_fault

    @property
    def fault_count(self) -> int:
        """Handler faults since construction."""
        return self._fault_count

    @property
    def consecutive_fault_ticks(self) -> int:
        """Ticks in a row, up to now, in which at least one handler faulted."""
        return self._consecutive_fault_ticks

    def step(self, estimate: StateEstimate, flags: FdirFlags, t: float) -> ModeCommand:
        """Check transitions (FDIR SAFE first), then step the active mode."""
        self._faulted_this_tick = False
        command = self._run_tick(estimate, flags, t)
        self._consecutive_fault_ticks = self._consecutive_fault_ticks + 1 if self._faulted_this_tick else 0
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

    def _run_tick(self, estimate: StateEstimate, flags: FdirFlags, t: float) -> ModeCommand:
        if not self._started:
            self._started = True
            try:
                self._handlers[self._mode].enter(t)
            except Exception as exc:
                self._quarantine("enter", exc, t)

        if flags.force_safe and self._mode is not Mode.SAFE:
            # FDIR overrides whatever the active mode wanted; its own
            # next_mode is not consulted.
            self._transition(Mode.SAFE, t, self._fdir_reason(flags))
        else:
            # Includes SAFE itself: SAFE sees the flags and decides when its
            # reasons have cleared.
            nxt = self._ask_next_mode(estimate, flags, t)
            if nxt is not None:
                self._transition(nxt, t, "exit criteria")

        try:
            command = self._handlers[self._mode].step(estimate, flags, t)
            if command.mode is not self._mode:
                raise RuntimeError(f"{self._mode.name} handler returned a {command.mode} command")
        except Exception as exc:
            self._quarantine("step", exc, t)
            # No target and no submode: the actuators stay safed this tick.
            return ModeCommand(mode=Mode.SAFE)
        return command

    def _ask_next_mode(self, estimate: StateEstimate, flags: FdirFlags, t: float) -> Optional[Mode]:
        """The active handler's requested next mode, or None to stay.

        Also rejects an answer the table doesn't allow, so a buggy handler is
        quarantined here instead of raising out of ``_transition``.
        """
        try:
            nxt = self._handlers[self._mode].next_mode(estimate, flags, t)
            if nxt is None or nxt is self._mode:
                return None
            self._check_legal(nxt, "exit criteria")
            return nxt
        except Exception as exc:
            self._quarantine("next_mode", exc, t)
            return None

    def _check_legal(self, to_mode: Mode, reason: str) -> None:
        if to_mode not in MODE_TRANSITIONS[self._mode]:
            raise IllegalTransition(f"{self._mode.name} -> {to_mode} is not allowed ({reason})")

    def _record_fault(self, where: str, exc: Exception, t: float) -> None:
        self._faulted_this_tick = True
        self._fault_count += 1
        self._last_fault = HandlerFault(t, self._mode, where, repr(exc))

    def _quarantine(self, where: str, exc: Exception, t: float) -> None:
        """Record the fault, and leave the faulting mode for SAFE.

        In SAFE there is nowhere to go, so it is only recorded.
        """
        self._record_fault(where, exc, t)
        if self._mode is not Mode.SAFE:
            self._transition(Mode.SAFE, t, f"handler fault: {where}")

    def _transition(self, to_mode: Mode, t: float, reason: str) -> None:
        # Checked before anything is exited, so a bad request leaves the
        # machine exactly as it was.
        self._check_legal(to_mode, reason)
        from_mode = self._mode
        if self._started:
            try:
                self._handlers[from_mode].exit(t)
            except Exception as exc:
                # We are leaving anyway; the fault is recorded, not acted on.
                self._record_fault("exit", exc, t)
        else:
            # Forced before the first step: the initial handler was never
            # entered, so there is nothing to exit.
            self._started = True
        self._mode = to_mode
        enter_failed = False
        try:
            self._handlers[to_mode].enter(t)
        except Exception as exc:
            self._record_fault("enter", exc, t)
            enter_failed = True
        self._last_transition = Transition(t, from_mode, to_mode, reason)
        if self._on_transition is not None:
            try:
                self._on_transition(self._last_transition)
            except Exception as exc:
                # A failed save or log must not take the state machine down.
                self._record_fault("on_transition", exc, t)
        if enter_failed and to_mode is not Mode.SAFE:
            self._transition(Mode.SAFE, t, "handler fault: enter")

    @staticmethod
    def _fdir_reason(flags: FdirFlags) -> str:
        reasons = sorted(r.value for r in flags.safe_reasons)
        return "fdir: " + (", ".join(reasons) if reasons else "force_safe")
