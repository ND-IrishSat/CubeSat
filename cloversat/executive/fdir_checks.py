"""Fault detection, isolation, and recovery checks.

Purpose:  Check the ``HealthReport`` and ``StateEstimate`` against limits,
          decide degrade actions, and force SAFE when needed.
Inputs:   ``HealthReport``, ``StateEstimate``, ``ExecutiveParams.fdir_limits``.
Outputs:  ``FdirFlags`` (read by the state manager).
Status:   [NEW]
Port from: none

Related open questions: OQ-4 (limits), OQ-7 (SAFE).

Later:
    - South Atlantic Anomaly pass logging.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from cloversat.config.executive_params import ExecutiveParams
    from cloversat.datatypes.health_types import HealthReport
    from cloversat.datatypes.state_types import StateEstimate


class SafeReason(Enum):
    """Why FDIR wants SAFE. SAFE stays active until every reason has cleared."""

    LOW_POWER = "low_power"
    BOOT_LOOP = "boot_loop"
    SUBSYSTEM_RECOVERY = "subsystem_recovery"  # a subsystem is being restarted
    CRITICAL_FAULT = "critical_fault"
    ESTIMATOR_INVALID = "estimator_invalid"
    PARAMS_INVALID = "params_invalid"


@dataclass
class FdirFlags:
    """FDIR output for this tick.

    Attributes:
        valid: False until FDIR has evaluated its first ``HealthReport``.
            "No reasons" means nothing while this is False, so BOOT waits for
            it and SAFE will not treat an invalid tick as cleared.
        force_safe: State manager must enter SAFE. True whenever
            ``safe_reasons`` is non-empty.
        safe_reasons: Active reasons for SAFE. SAFE reads these to know what
            has to clear before it can exit.
        low_power: Battery is below the entry threshold. Hysteresis (separate
            enter and exit thresholds) is applied upstream, never in a mode.
        faults: Names of active faults.
        disabled_subsystems: Subsystems FDIR has disabled.
    """

    valid: bool = False
    force_safe: bool = False
    safe_reasons: set[SafeReason] = field(default_factory=set)
    low_power: bool = False
    faults: list[str] = field(default_factory=list)
    disabled_subsystems: list[str] = field(default_factory=list)


class FdirChecks:
    """Evaluates fault checks each tick."""

    def __init__(self, params: ExecutiveParams) -> None:
        raise NotImplementedError

    def evaluate(self, health: HealthReport, estimate: StateEstimate, t: float) -> FdirFlags:
        """Run all checks and return this tick's flags."""
        raise NotImplementedError
