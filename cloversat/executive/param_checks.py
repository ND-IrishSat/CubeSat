"""Executive parameter validation.

Purpose:  Report which executive params are missing or out of range, and
          fingerprint them. Shared by BOOT (checked once at start) and FDIR
          (checked continuously, so SAFE stays active while a bad param remains).
Inputs:   ``ExecutiveParams``.
Outputs:  ``validate_params`` -> problem strings; empty means usable.
          ``params_checksum`` -> CRC32 of every field.

Range checks miss a plausible-but-wrong value (0.05 flipped to 0.5). The
checksum catches any change at all. BOOT records it at start; FDIR recomputes
it each tick, and a difference means the in-memory params changed after boot.
Status:   [NEW]
Port from: none

There are no fallback values here. Defaults belong in ``ExecutiveParams``
itself, so there is one place that says what a value is.

``REQUIRED`` lists the params some *implemented* code reads. A mode adds its
fields when it is implemented. Fields still waiting on an open question stay
out, or every boot would fail on them.
"""
from __future__ import annotations

import json
import zlib
from dataclasses import asdict
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from cloversat.config.executive_params import ExecutiveParams

REQUIRED = (
    "heartbeat_period_s",
    "boot_loop_resets",
    "boot_loop_window_s",
    "boot_health_wait_s",
    "safe_min_dwell_s",
    "safe_clear_hold_s",
)

# Scalar params that must be positive whenever they are set.
POSITIVE_IF_SET = (
    "detumble_rate_threshold",
    "detumble_hold_s",
    "pointing_error_tolerance",
    "rate_tolerance",
    "deploy_inhibit_s",
    "deploy_max_burn_s",
    "deploy_max_attempts",
    "control_period_s",
    "safe_slew_rate",
    "safe_gain_scale",
    "safe_wheel_momentum_limit",
)


def _positive(value: object) -> bool:
    # NaN fails `> 0`, and bool is excluded because it is an int subclass.
    return isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0


def validate_params(params: ExecutiveParams) -> list[str]:
    """Return a description of every problem found; empty if none."""
    problems = []
    for name in REQUIRED:
        value = getattr(params, name, None)
        if value is None:
            problems.append(f"{name} is missing")
        elif not _positive(value):
            problems.append(f"{name}={value!r} must be a positive number")
    for name in POSITIVE_IF_SET:
        value = getattr(params, name, None)
        if value is not None and not _positive(value):
            problems.append(f"{name}={value!r} must be a positive number")
    return problems


def params_checksum(params: ExecutiveParams) -> int:
    """CRC32 over every field, nested watchdog settings included.

    Sorted-key JSON makes it independent of dict order, and NaN and infinity
    still serialize, so a corrupted float can't make the check itself fail.
    """
    blob = json.dumps(asdict(params), sort_keys=True, default=repr).encode()
    return zlib.crc32(blob)
