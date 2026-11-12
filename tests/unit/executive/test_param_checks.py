from dataclasses import replace

from cloversat.config.executive_params import ExecutiveParams
import struct

from cloversat.config.executive_params import WatchdogSubsystemParams
from cloversat.executive.param_checks import REQUIRED, params_checksum, validate_params


def test_default_params_are_valid():
    assert validate_params(ExecutiveParams()) == []


def test_every_required_field_missing_is_reported():
    for name in REQUIRED:
        problems = validate_params(replace(ExecutiveParams(), **{name: None}))
        assert any(name in p and "missing" in p for p in problems), name


def test_out_of_range_required_value():
    assert validate_params(replace(ExecutiveParams(), boot_loop_window_s=0.0))
    assert validate_params(replace(ExecutiveParams(), boot_loop_resets=-1))
    assert validate_params(replace(ExecutiveParams(), boot_health_wait_s=float("nan")))
    assert validate_params(replace(ExecutiveParams(), boot_loop_resets=True))


def test_optional_value_unset_is_fine_but_set_must_be_positive():
    assert validate_params(replace(ExecutiveParams(), detumble_rate_threshold=None)) == []
    assert validate_params(replace(ExecutiveParams(), detumble_rate_threshold=-0.1))


def test_checksum_is_stable_for_equal_params():
    assert params_checksum(ExecutiveParams()) == params_checksum(ExecutiveParams())


def test_checksum_catches_a_plausible_value_that_range_checks_miss():
    good, flipped = ExecutiveParams(), replace(ExecutiveParams(), boot_health_wait_s=50.0)
    assert validate_params(flipped) == []          # still "valid"
    assert params_checksum(good) != params_checksum(flipped)


def test_checksum_catches_a_single_bit_flip_in_a_float():
    bits = struct.unpack("<Q", struct.pack("<d", 600.0))[0] ^ 1
    flipped = struct.unpack("<d", struct.pack("<Q", bits))[0]
    assert params_checksum(ExecutiveParams()) != params_checksum(replace(ExecutiveParams(), boot_loop_window_s=flipped))


def test_checksum_covers_nested_watchdog_settings_and_ignores_dict_order():
    a = replace(ExecutiveParams(), watchdog={"gps": WatchdogSubsystemParams(deadline_s=1.0), "imu": WatchdogSubsystemParams()})
    b = replace(ExecutiveParams(), watchdog={"imu": WatchdogSubsystemParams(), "gps": WatchdogSubsystemParams(deadline_s=1.0)})
    c = replace(a, watchdog={**a.watchdog, "gps": WatchdogSubsystemParams(deadline_s=2.0)})
    assert params_checksum(a) == params_checksum(b)
    assert params_checksum(a) != params_checksum(c)


def test_checksum_still_computes_for_nan():
    assert isinstance(params_checksum(replace(ExecutiveParams(), boot_health_wait_s=float("nan"))), int)
