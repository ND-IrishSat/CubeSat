import pytest

from cloversat.config.executive_params import ExecutiveParams
from cloversat.datatypes.command_types import Mode, ModeCommand
from cloversat.executive.fdir_checks import FdirFlags, SafeReason
from cloversat.executive.state_manager import (
    MODE_TRANSITIONS,
    SUBMODE_TRANSITIONS,
    IllegalTransition,
    ModeHandler,
    StateManager,
)
from cloversat.datatypes.command_types import SubMode


class StubHandler(ModeHandler):
    """Records calls into a shared log; `goto` is what next_mode returns."""

    def __init__(self, mode, log, goto=None):
        self.mode = mode
        self.log = log
        self.goto = goto
        self.raises = set()  # method names that should raise

    def _maybe_raise(self, name):
        if name in self.raises:
            raise RuntimeError(f"{self.mode.name}.{name} boom")

    def enter(self, t):
        self.log.append(("enter", self.mode))
        self._maybe_raise("enter")

    def step(self, estimate, flags, t):
        self.log.append(("step", self.mode))
        self._maybe_raise("step")
        return ModeCommand(mode=self.mode)

    def next_mode(self, estimate, flags, t):
        self.log.append(("next_mode", self.mode))
        self._maybe_raise("next_mode")
        return self.goto

    def exit(self, t):
        self.log.append(("exit", self.mode))
        self._maybe_raise("exit")


def build(initial=Mode.BOOT, gotos=None, **kwargs):
    log = []
    gotos = gotos or {}
    handlers = {m: StubHandler(m, log, gotos.get(m)) for m in Mode}
    sm = StateManager(ExecutiveParams(), handlers, initial, **kwargs)
    return sm, handlers, log


def step(sm, flags=None, t=0.0):
    return sm.step(None, flags or FdirFlags(), t)


def test_missing_handler_rejected():
    handlers = {m: StubHandler(m, []) for m in Mode if m is not Mode.SAFE}
    with pytest.raises(ValueError, match="SAFE"):
        StateManager(ExecutiveParams(), handlers, Mode.BOOT)


def test_initial_mode_entered_on_first_step():
    sm, _, log = build()
    assert sm.mode is Mode.BOOT and log == []
    step(sm)
    assert log[0] == ("enter", Mode.BOOT)


def test_stays_put_when_next_mode_is_none():
    sm, _, log = build(initial=Mode.DETUMBLE)
    cmd = step(sm)
    assert sm.mode is Mode.DETUMBLE and cmd.mode is Mode.DETUMBLE
    assert sm.last_transition is None
    assert ("exit", Mode.DETUMBLE) not in log


def test_transition_runs_exit_then_enter_then_step_new_mode():
    sm, _, log = build(initial=Mode.BOOT, gotos={Mode.BOOT: Mode.DETUMBLE})
    cmd = step(sm, t=5.0)
    assert log == [
        ("enter", Mode.BOOT),
        ("next_mode", Mode.BOOT),
        ("exit", Mode.BOOT),
        ("enter", Mode.DETUMBLE),
        ("step", Mode.DETUMBLE),
    ]
    assert cmd.mode is Mode.DETUMBLE
    tr = sm.last_transition
    assert (tr.t, tr.from_mode, tr.to_mode, tr.reason) == (5.0, Mode.BOOT, Mode.DETUMBLE, "exit criteria")


def test_one_transition_per_tick():
    sm, _, _ = build(
        initial=Mode.BOOT,
        gotos={Mode.BOOT: Mode.DETUMBLE, Mode.DETUMBLE: Mode.ORIENT_TRANSITION},
    )
    step(sm)
    assert sm.mode is Mode.DETUMBLE
    step(sm)
    assert sm.mode is Mode.ORIENT_TRANSITION


@pytest.mark.parametrize("start", [Mode.BOOT, Mode.DETUMBLE, Mode.ORIENT_TRANSITION, Mode.OPERATIONAL])
def test_fdir_forces_safe_from_any_mode(start):
    sm, handlers, log = build(initial=start)
    step(sm)  # enter initial
    handlers[start].goto = Mode.OPERATIONAL  # would transition if FDIR didn't override
    log.clear()
    flags = FdirFlags(force_safe=True, safe_reasons={SafeReason.LOW_POWER})
    cmd = step(sm, flags)
    assert sm.mode is Mode.SAFE and cmd.mode is Mode.SAFE
    # The active mode's own exit criteria are skipped when FDIR overrides.
    assert ("next_mode", start) not in log
    assert log[:2] == [("exit", start), ("enter", Mode.SAFE)]
    assert sm.last_transition.reason == "fdir: low_power"


def test_safe_not_reentered_while_flags_stay_set():
    sm, _, log = build(initial=Mode.SAFE)
    flags = FdirFlags(force_safe=True)
    step(sm, flags)
    step(sm, flags)
    assert log.count(("enter", Mode.SAFE)) == 1
    assert ("next_mode", Mode.SAFE) in log  # SAFE decides its own exit


def test_safe_exits_via_its_own_next_mode():
    sm, _, _ = build(initial=Mode.SAFE, gotos={Mode.SAFE: Mode.DETUMBLE})
    step(sm, FdirFlags(force_safe=True))
    assert sm.mode is Mode.DETUMBLE


def test_force_mode_current_mode_is_noop():
    sm, _, log = build(initial=Mode.DETUMBLE)
    step(sm)
    log.clear()
    sm.force_mode(Mode.DETUMBLE, 1.0, "test")
    assert log == [] and sm.last_transition is None


def test_force_mode_transitions_and_records_reason():
    sm, _, log = build(initial=Mode.ORIENT_TRANSITION)
    step(sm)
    log.clear()
    sm.force_mode(Mode.OPERATIONAL, 2.0, "test")
    assert log == [("exit", Mode.ORIENT_TRANSITION), ("enter", Mode.OPERATIONAL)]
    assert sm.mode is Mode.OPERATIONAL
    assert sm.last_transition.reason == "test"


def test_force_mode_before_first_step_skips_exit_of_unentered_initial():
    sm, _, log = build(initial=Mode.BOOT)
    sm.force_mode(Mode.SAFE, 0.0, "boot loop")
    assert log == [("enter", Mode.SAFE)]
    step(sm)
    assert log.count(("enter", Mode.SAFE)) == 1 and ("enter", Mode.BOOT) not in log


def test_on_transition_callback_receives_each_change():
    seen = []
    sm, _, _ = build(initial=Mode.BOOT, gotos={Mode.BOOT: Mode.DETUMBLE}, on_transition=seen.append)
    step(sm)
    assert [(x.from_mode, x.to_mode) for x in seen] == [(Mode.BOOT, Mode.DETUMBLE)]




# --- the transition table is the source of truth for what is legal ---


def test_table_has_every_mode_and_no_self_loops():
    assert set(MODE_TRANSITIONS) == set(Mode)
    assert all(m not in nxt for m, nxt in MODE_TRANSITIONS.items())


def test_safe_reachable_from_every_other_mode_and_leaves_only_via_detumble():
    assert all(Mode.SAFE in nxt for m, nxt in MODE_TRANSITIONS.items() if m is not Mode.SAFE)
    assert MODE_TRANSITIONS[Mode.SAFE] == {Mode.DETUMBLE}


def test_submodes_return_home_and_never_switch_directly():
    assert set(SUBMODE_TRANSITIONS) == set(SubMode)
    for sub, nxt in SUBMODE_TRANSITIONS.items():
        if sub is not SubMode.SUN_POINT:
            assert nxt == {SubMode.SUN_POINT}


def test_illegal_force_mode_raises():
    sm, _, _ = build(initial=Mode.DETUMBLE)
    step(sm)
    with pytest.raises(IllegalTransition):
        sm.force_mode(Mode.OPERATIONAL, 1.0, "test")
    assert sm.mode is Mode.DETUMBLE


# --- handler faults are quarantined, not propagated ---

BARE_SAFE = ModeCommand(mode=Mode.SAFE)


def test_step_fault_quarantines_to_safe_with_a_bare_command():
    sm, handlers, _ = build(initial=Mode.DETUMBLE)
    step(sm)
    handlers[Mode.DETUMBLE].raises = {"step"}
    cmd = step(sm, t=3.0)
    assert sm.mode is Mode.SAFE
    assert cmd == BARE_SAFE and cmd.target is None
    assert sm.last_fault.where == "step" and sm.last_fault.mode is Mode.DETUMBLE
    assert "boom" in sm.last_fault.error
    assert sm.last_transition.reason == "handler fault: step"


def test_next_mode_fault_quarantines_and_safe_still_steps_that_tick():
    sm, handlers, log = build(initial=Mode.DETUMBLE)
    step(sm)
    handlers[Mode.DETUMBLE].raises = {"next_mode"}
    log.clear()
    cmd = step(sm)
    assert sm.mode is Mode.SAFE and cmd.mode is Mode.SAFE
    assert ("step", Mode.SAFE) in log
    assert sm.last_fault.where == "next_mode"


def test_illegal_next_mode_answer_is_a_handler_fault_not_a_crash():
    sm, _, log = build(initial=Mode.BOOT, gotos={Mode.BOOT: Mode.OPERATIONAL})
    cmd = step(sm)
    assert sm.mode is Mode.SAFE and cmd.mode is Mode.SAFE
    assert "IllegalTransition" in sm.last_fault.error
    assert ("exit", Mode.BOOT) in log


def test_handler_returning_a_command_for_the_wrong_mode_is_quarantined():
    sm, handlers, _ = build(initial=Mode.DETUMBLE)
    step(sm)
    handlers[Mode.DETUMBLE].mode = Mode.SAFE  # makes its step() emit a SAFE command
    assert step(sm) == BARE_SAFE
    assert sm.mode is Mode.SAFE and sm.last_fault.where == "step"


def test_safe_failing_has_nowhere_to_go_so_it_returns_a_bare_command():
    sm, handlers, log = build(initial=Mode.SAFE)
    step(sm)
    handlers[Mode.SAFE].raises = {"step"}
    log.clear()
    assert step(sm) == BARE_SAFE
    assert sm.mode is Mode.SAFE and sm.last_transition is None
    assert sm.last_fault.where == "step"
    assert ("enter", Mode.SAFE) not in log


def test_exit_fault_is_recorded_but_the_transition_completes():
    sm, handlers, _ = build(initial=Mode.BOOT, gotos={Mode.BOOT: Mode.DETUMBLE})
    handlers[Mode.BOOT].raises = {"exit"}
    cmd = step(sm)
    assert sm.mode is Mode.DETUMBLE and cmd.mode is Mode.DETUMBLE
    assert sm.last_fault.where == "exit" and sm.fault_count == 1


def test_enter_fault_on_a_normal_transition_ends_in_safe():
    seen = []
    sm, handlers, _ = build(initial=Mode.BOOT, gotos={Mode.BOOT: Mode.DETUMBLE}, on_transition=seen.append)
    handlers[Mode.DETUMBLE].raises = {"enter"}
    step(sm)
    assert sm.mode is Mode.SAFE
    assert [(x.from_mode, x.to_mode) for x in seen] == [(Mode.BOOT, Mode.DETUMBLE), (Mode.DETUMBLE, Mode.SAFE)]


def test_initial_enter_fault_goes_to_safe():
    sm, handlers, _ = build(initial=Mode.BOOT)
    handlers[Mode.BOOT].raises = {"enter"}
    cmd = step(sm)
    assert sm.mode is Mode.SAFE and cmd.mode is Mode.SAFE
    assert sm.last_fault.where == "enter"


def test_on_transition_callback_failure_does_not_stop_the_machine():
    def boom(_):
        raise OSError("disk full")

    sm, _, _ = build(initial=Mode.BOOT, gotos={Mode.BOOT: Mode.DETUMBLE}, on_transition=boom)
    cmd = step(sm)
    assert sm.mode is Mode.DETUMBLE and cmd.mode is Mode.DETUMBLE
    assert sm.last_fault.where == "on_transition"


def test_consecutive_fault_ticks_count_and_reset():
    sm, handlers, _ = build(initial=Mode.SAFE)
    step(sm)
    assert sm.consecutive_fault_ticks == 0
    handlers[Mode.SAFE].raises = {"step"}
    step(sm)
    step(sm)
    assert sm.consecutive_fault_ticks == 2
    handlers[Mode.SAFE].raises = set()
    step(sm)
    assert sm.consecutive_fault_ticks == 0 and sm.fault_count == 2


def test_non_exception_baseexceptions_are_not_swallowed():
    sm, handlers, _ = build(initial=Mode.DETUMBLE)
    step(sm)

    def interrupt(*a):
        raise KeyboardInterrupt

    handlers[Mode.DETUMBLE].step = interrupt
    with pytest.raises(KeyboardInterrupt):
        step(sm)
