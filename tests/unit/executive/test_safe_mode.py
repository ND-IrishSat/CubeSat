from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from cloversat.config.executive_params import ExecutiveParams
from cloversat.datatypes.command_types import Mode
from cloversat.executive.fdir_checks import FdirFlags, SafeReason
from cloversat.executive.safe_mode import (
    SAFE_PHASE_TRANSITIONS,
    SafeBehavior,
    SafeMode,
    SafePhase,
    choose_behavior,
    reasons_cleared,
)

P = ExecutiveParams()  # hold 60 s, dwell 120 s (placeholders)
CLEAR = FdirFlags(valid=True)
LOW = FdirFlags(valid=True, force_safe=True, safe_reasons={SafeReason.LOW_POWER}, low_power=True)


def est(**kw):
    base = dict(attitude_valid=True, rates_valid=True, wheels_valid=True, h_wheels_body=np.zeros(3))
    return SimpleNamespace(**{**base, **kw})


def entered(t=0.0):
    safe = SafeMode(P)
    safe.enter(t)
    return safe


def run(safe, flags, t):
    """One state-manager tick: exit check first, then step."""
    nxt = safe.next_mode(None, flags, t)
    safe.step(est(attitude_valid=False), flags, t)  # passive: sun-seek isn't built
    return nxt


# --- phases ---


def test_phase_table_is_active_clearing_both_ways():
    assert SAFE_PHASE_TRANSITIONS == {
        SafePhase.ACTIVE: {SafePhase.CLEARING},
        SafePhase.CLEARING: {SafePhase.ACTIVE},
    }


def test_starts_active_and_stays_while_a_reason_is_present():
    safe = entered()
    run(safe, LOW, 10.0)
    assert safe.phase is SafePhase.ACTIVE


def test_clear_flags_start_clearing():
    safe = entered()
    run(safe, CLEAR, 10.0)
    assert safe.phase is SafePhase.CLEARING


def test_reason_returning_discards_the_hold_timer():
    safe = SafeMode(replace(P, safe_min_dwell_s=1.0))   # take dwell out of the picture
    safe.enter(0.0)
    run(safe, CLEAR, 10.0)       # CLEARING from t=10
    run(safe, LOW, 40.0)         # reason returns: ACTIVE, timer discarded
    assert safe.phase is SafePhase.ACTIVE
    run(safe, CLEAR, 50.0)       # CLEARING again, timer restarts at t=50
    assert run(safe, CLEAR, 100.0) is None            # 50 s held; a kept timer would say 90
    assert run(safe, CLEAR, 111.0) is Mode.DETUMBLE   # 61 s since the restart


def test_fdir_not_yet_valid_is_not_all_clear():
    safe = entered()
    run(safe, FdirFlags(valid=False), 10.0)
    assert safe.phase is SafePhase.ACTIVE
    assert reasons_cleared(FdirFlags(valid=False)) is False


# --- exit ---


def test_exit_needs_hold_and_dwell_then_goes_to_detumble():
    safe = entered(0.0)
    run(safe, CLEAR, 10.0)                       # CLEARING from t=10
    assert run(safe, CLEAR, 60.0) is None        # held 50 s < 60, dwell 60 < 120
    assert run(safe, CLEAR, 100.0) is None       # held 90 s, dwell 100 < 120
    assert run(safe, CLEAR, 130.0) is Mode.DETUMBLE


def test_dwell_alone_is_not_enough():
    safe = entered(0.0)
    assert run(safe, CLEAR, 500.0) is None       # first clear tick: hold not started
    assert run(safe, CLEAR, 530.0) is None       # 30 s < 60 s hold
    assert run(safe, CLEAR, 561.0) is Mode.DETUMBLE


def test_reason_appearing_blocks_an_exit_that_was_about_to_fire():
    safe = entered(0.0)
    run(safe, CLEAR, 10.0)
    assert safe.next_mode(None, LOW, 300.0) is None   # phase says CLEARING, flags say no


def test_enter_resets_timers():
    safe = entered(0.0)
    run(safe, CLEAR, 10.0)
    safe.enter(1000.0)
    assert safe.phase is SafePhase.ACTIVE
    assert safe.next_mode(None, CLEAR, 1001.0) is None


# --- command and behavior ---


def test_passive_command_has_no_target():
    safe = entered()
    cmd = safe.step(est(attitude_valid=False), LOW, 1.0)
    assert cmd.mode is Mode.SAFE and cmd.submode is None and cmd.target is None


def test_default_params_force_passive_even_if_everything_is_healthy():
    # safe_wheel_momentum_limit is None until sun-seek is built.
    assert choose_behavior(est(), CLEAR, P) is SafeBehavior.PASSIVE


@pytest.mark.parametrize("bad", [
    dict(attitude_valid=False),
    dict(rates_valid=False),
    dict(wheels_valid=False),
    dict(h_wheels_body=np.array([1.0, 0.0, 0.0])),   # saturated
])
def test_each_unusable_input_forces_passive(bad):
    p = replace(P, safe_wheel_momentum_limit=0.5)
    assert choose_behavior(est(), CLEAR, p) is SafeBehavior.SUN_SEEK
    assert choose_behavior(est(**bad), CLEAR, p) is SafeBehavior.PASSIVE


def test_low_power_or_disabled_wheels_force_passive():
    p = replace(P, safe_wheel_momentum_limit=0.5)
    assert choose_behavior(est(), FdirFlags(valid=True, low_power=True), p) is SafeBehavior.PASSIVE
    assert choose_behavior(est(), FdirFlags(valid=True, disabled_subsystems=["wheels"]), p) is SafeBehavior.PASSIVE


def test_sun_seek_is_loudly_unimplemented():
    safe = SafeMode(replace(P, safe_wheel_momentum_limit=0.5))
    safe.enter(0.0)
    with pytest.raises(NotImplementedError):
        safe.step(est(), CLEAR, 1.0)


def test_unusable_exit_timers_never_raise_safe_just_stays():
    for bad in (dict(safe_clear_hold_s=None), dict(safe_min_dwell_s=None)):
        safe = SafeMode(replace(P, **bad))
        safe.enter(0.0)
        run(safe, CLEAR, 10.0)
        assert safe.next_mode(None, CLEAR, 10_000.0) is None
