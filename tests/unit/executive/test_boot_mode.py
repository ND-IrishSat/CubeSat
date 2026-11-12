from dataclasses import replace

from cloversat.config.executive_params import ExecutiveParams
from cloversat.datatypes.command_types import Mode
from cloversat.datatypes.state_types import PersistedState
from cloversat.devices.interfaces.storage_interface import StorageInterface
from cloversat.executive.boot_mode import (
    BOOT_STEP_TRANSITIONS,
    BootMode,
    BootStep,
    choose_reentry_mode,
    is_boot_loop,
)
from cloversat.executive.fdir_checks import FdirFlags
from cloversat.executive.param_checks import params_checksum
from cloversat.executive.persistence import Persistence
from cloversat.executive.state_manager import ModeHandler

P = ExecutiveParams()  # 3 resets in 600 s, health wait 5 s


class FakeStorage(StorageInterface):
    def __init__(self):
        self.slots = {}

    def init(self):
        pass

    def self_test(self):
        raise NotImplementedError

    def write_copy(self, i, blob):
        self.slots[i] = blob
        return True

    def read_copy(self, i):
        return self.slots.get(i)


def make(rtc=1000.0, params=P, storage=None):
    storage = storage or FakeStorage()
    persistence = Persistence(storage)
    boot = BootMode(params, persistence, lambda: rtc)
    boot.enter(0.0)
    return boot, persistence, storage


def tick(boot, flags, t=0.0):
    nxt = boot.next_mode(None, flags, t)
    boot.step(None, flags, t)
    return nxt


VALID = FdirFlags(valid=True)


# --- pure functions ---


def test_is_boot_loop_counts_only_boots_inside_window():
    s = PersistedState(recent_reset_times=[400.0, 700.0, 900.0])
    assert is_boot_loop(s, 950.0, P)        # all three within 600 s
    assert not is_boot_loop(s, 1100.0, P)   # the first (age 700) has aged out


def test_boot_loop_clears_by_itself_as_time_passes():
    s = PersistedState(recent_reset_times=[0.0, 10.0, 20.0])
    assert is_boot_loop(s, 30.0, P)
    assert not is_boot_loop(s, 700.0, P)


def test_is_boot_loop_ignores_future_timestamps():
    s = PersistedState(recent_reset_times=[5000.0, 5001.0, 5002.0])
    assert not is_boot_loop(s, 100.0, P)


def test_reentry_mode():
    assert choose_reentry_mode(False, True) is Mode.DETUMBLE
    assert choose_reentry_mode(True, True) is Mode.SAFE
    assert choose_reentry_mode(False, False) is Mode.SAFE


def test_step_table_is_a_single_chain_ending_in_done():
    assert BOOT_STEP_TRANSITIONS[BootStep.DONE] == frozenset()
    assert set(BOOT_STEP_TRANSITIONS) == set(BootStep)


# --- the handler ---


def test_is_a_mode_handler_for_boot():
    boot, _, _ = make()
    assert isinstance(boot, ModeHandler) and boot.mode is Mode.BOOT


def test_runs_straight_through_when_fdir_already_valid_then_leaves_next_tick():
    boot, _, _ = make()
    assert tick(boot, VALID, t=1.0) is None      # first tick: runs every step
    assert boot.boot_step is BootStep.DONE
    assert tick(boot, VALID, t=2.0) is Mode.DETUMBLE


def test_waits_for_fdir_then_proceeds():
    boot, _, _ = make()
    tick(boot, FdirFlags(valid=False), t=1.0)
    assert boot.boot_step is BootStep.AWAIT_HEALTH
    assert tick(boot, FdirFlags(valid=False), t=2.0) is None
    tick(boot, VALID, t=3.0)
    assert boot.boot_step is BootStep.DONE


def test_health_wait_times_out_instead_of_hanging():
    boot, _, _ = make()
    tick(boot, FdirFlags(valid=False), t=1.0)   # AWAIT_HEALTH starts at t=1
    tick(boot, FdirFlags(valid=False), t=5.9)
    assert boot.boot_step is BootStep.AWAIT_HEALTH
    tick(boot, FdirFlags(valid=False), t=6.0)   # boot_health_wait_s = 5
    assert boot.boot_step is BootStep.DONE


def test_records_this_boot_and_saves_immediately():
    boot, persistence, _ = make(rtc=1000.0)
    tick(boot, VALID)
    assert boot.copies_saved == 3
    assert persistence.load().recent_reset_times == [1000.0]


def test_history_is_trimmed_to_window_and_capped():
    storage = FakeStorage()
    Persistence(storage).save(PersistedState(recent_reset_times=[1.0, 700.0, 790.0, 800.0]))
    boot, persistence, _ = make(rtc=1000.0, storage=storage)
    tick(boot, VALID)
    # 1.0 is outside the 600 s window; cap is boot_loop_resets = 3 entries.
    assert persistence.load().recent_reset_times == [790.0, 800.0, 1000.0]


def test_third_quick_reset_goes_to_safe():
    storage = FakeStorage()
    Persistence(storage).save(PersistedState(recent_reset_times=[900.0, 950.0]))
    boot, _, _ = make(rtc=1000.0, storage=storage)
    tick(boot, VALID, t=1.0)
    assert boot.boot_loop is True
    assert tick(boot, VALID, t=2.0) is Mode.SAFE


def test_no_valid_copy_is_reported_as_untrusted_and_still_boots():
    boot, _, _ = make()
    tick(boot, VALID)
    assert boot.state_trusted is False
    assert boot.state.deployed is False


def test_loaded_state_is_trusted_and_kept():
    storage = FakeStorage()
    Persistence(storage).save(PersistedState(deployed=True, deploy_attempts=2))
    boot, _, _ = make(storage=storage)
    tick(boot, VALID)
    assert boot.state_trusted is True
    assert boot.state.deployed is True and boot.state.deploy_attempts == 2


def test_bad_params_go_to_safe():
    boot, _, _ = make(params=replace(P, boot_loop_window_s=-1.0))
    tick(boot, VALID, t=1.0)
    assert boot.params_ok is False
    assert tick(boot, VALID, t=2.0) is Mode.SAFE


def test_no_trustworthy_clock_skips_recording_and_still_boots():
    boot, persistence, _ = make(rtc=None)
    tick(boot, VALID, t=1.0)
    assert boot.copies_saved == 0 and persistence.load() is None
    assert tick(boot, VALID, t=2.0) is Mode.DETUMBLE


def test_enter_restarts_the_sequence():
    boot, _, _ = make()
    tick(boot, VALID)
    boot.enter(10.0)
    assert boot.boot_step is BootStep.LOAD_STATE and boot.boot_loop is False


def test_records_params_checksum_baseline_for_fdir():
    boot, _, _ = make()
    tick(boot, VALID)
    assert boot.params_checksum == params_checksum(P)
