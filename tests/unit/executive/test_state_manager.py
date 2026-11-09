import pytest

from cloversat.config.executive_params import ExecutiveParams
from cloversat.datatypes.command_types import Mode, ModeCommand
from cloversat.executive.fdir_checks import FdirFlags, SafeReason
from cloversat.executive.state_manager import ModeHandler, StateManager


class StubHandler(ModeHandler):
    """Records calls into a shared log; `goto` is what next_mode returns."""

    def __init__(self, mode, log, goto=None):
        self.mode = mode
        self.log = log
        self.goto = goto

    def enter(self, t):
        self.log.append(("enter", self.mode))

    def step(self, estimate, flags, t):
        self.log.append(("step", self.mode))
        return ModeCommand(mode=self.mode)

    def next_mode(self, estimate, flags, t):
        self.log.append(("next_mode", self.mode))
        return self.goto

    def exit(self, t):
        self.log.append(("exit", self.mode))


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
    sm, _, log = build(initial=Mode.DETUMBLE)
    step(sm)
    log.clear()
    sm.force_mode(Mode.OPERATIONAL, 2.0, "boot re-entry")
    assert log == [("exit", Mode.DETUMBLE), ("enter", Mode.OPERATIONAL)]
    assert sm.mode is Mode.OPERATIONAL
    assert sm.last_transition.reason == "boot re-entry"


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


def test_handler_returning_wrong_mode_is_an_error():
    sm, handlers, _ = build(initial=Mode.BOOT)
    handlers[Mode.BOOT].mode = Mode.SAFE  # makes step() emit a SAFE command
    with pytest.raises(RuntimeError):
        step(sm)
