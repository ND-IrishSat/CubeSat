# executive/ — Layer 1

Runs the spacecraft: starts everything up, picks the operating mode, watches
health, deploys the panels, saves state that has to survive a reset, and queues
telemetry. It reads `StateEstimate` (from estimation) and FDIR flags, and
hands control a `ModeCommand`. Mode decisions **never** use sim truth.

Each file's docstring covers its inputs, outputs and port notes. This page
explains how the files fit together.

## Startup and tasks (`main.py`)

`main.py` is the only module that loads a profile (`flight`, `sim` or
`bertha`). It builds each module with its own params object, picks the device
backend, and launches these tasks:

| Task | Rate | Does |
|---|---|---|
| Control loop | ~10–20 Hz, hard deadline | devices → estimator → state manager (mode step ~1 Hz) → controller → allocation → actuation scheduler → devices |
| Heartbeat / FDIR | ~1 Hz | `heartbeat` → `HealthReport`; `fdir_checks`; `watchdog_supervisor`; can force SAFE |
| Telemetry | event / periodic | `telemetry` builds `TelemetryRecord`s into a queue for comms |
| Slow I/O | async | GPS serial parsing, `persistence` writes; never blocks the control loop |

Tasks talk only through queues or a shared state snapshot. They never call
into each other.

## Modes (`state_manager.py` + `*_mode.py`)

`StateManager` holds one `ModeHandler` per mode, each with `enter`, `step`,
`next_mode` and `exit`. `main.py` injects the handlers, so `state_manager.py`
never imports the mode files. Exit criteria read **only** `StateEstimate` and
`FdirFlags`.

```
          ┌──────────────────────────────────────────────┐
          ▼                                              │
BOOT ─► DETUMBLE ─► ORIENT_TRANSITION ─► OPERATIONAL ────┤
 │         ▲              │                              │
 │         └── rates high ┘          any mode ─(FDIR)─► SAFE ─(OQ-7)─┘
 └─ resume point from PersistedState (skip completed steps)
```

| Mode | Behavior | Exit |
|---|---|---|
| BOOT | Record the reset cause, load `PersistedState`, verify params, restore time, run boot-loop protection | Next mode in the progression, skipping completed steps |
| DETUMBLE | Torquers only (B-cross, with B-dot as fallback) | Sensed rate below threshold on all axes for N s, with hysteresis (OQ-4) |
| ORIENT_TRANSITION | Deploy if not deployed, acquire attitude (OQ-5), get a GPS fix | Deployed + attitude valid + GPS fix |
| OPERATIONAL | SUN_POINT by default, with inertial hold in eclipse. GPS_POINT, GROUND_POINT and MOMENTUM_DUMP preempt it and always return to SUN_POINT (OQ-6) | FDIR fault → SAFE |
| SAFE | Detumble, find the sun, hold. Minimal loads, beacon | Autonomous recovery (OQ-7) |

There's no COAST mode: the eclipse hold inside SUN_POINT covers it.

## Health, FDIR and watchdog

- **`heartbeat.py`** collects power, temperatures and panel currents at 1 Hz
  into a `HealthReport`.
- **`fdir_checks.py`** checks sensor freshness, estimator sanity (covariance,
  quaternion norm), rate spikes and parameter ranges. It can mark sensors
  degraded or force SAFE.
- **`watchdog_supervisor.py`** tracks each subsystem's check-ins:
  - **HEALTHY → LATE:** a check-in is missed. This is only logged.
  - **LATE → STALLED:** N misses in a row. The supervisor calls `safe()` on the
    subsystem's actuators and restarts its task and driver.
  - **STALLED → HEALTHY or FAILED:** it recovers, or it goes FAILED after M
    failed recoveries.
  - **FAILED, non-critical:** the subsystem is disabled and the spacecraft runs
    degraded.
  - **FAILED, critical:** the supervisor saves state and reboots in a
    controlled way.

  The supervisor alone kicks the Pi's `/dev/watchdog`, and only while no
  critical subsystem is FAILED. Per-subsystem settings live in
  `ExecutiveParams.watchdog`.

## Boot, persistence, deployment

- **Persistence:** `persistence.py` saves `PersistedState` continuously as
  three checksummed copies, with atomic writes and a majority vote on read. A
  watchdog reset gets no graceful shutdown, so nothing can wait until shutdown
  to save.
- **Boot-loop protection:** `boot_mode.py` handles it, for example 3 resets in
  10 minutes → minimal SAFE and disable the implicated subsystem. With no
  uplink, this is the only way out of a boot loop.
- **Deployment:** `deployment_sequence.py` runs preconditions (inhibit timer,
  not deployed, attempts < limit) → `prepare` → verify → `deploy` → verify
  `read_deployed` → persist → retry policy, and calls `safe()` in `finally`.
  **Deployment never fires again after a reset once it's confirmed.**

## Files

| Status | Files |
|---|---|
| **Ready to start** | `state_manager.py`, `boot_mode.py`, `persistence.py`, `deployment_sequence.py`, `heartbeat.py`, `fdir_checks.py`, `watchdog_supervisor.py`, `main.py` (last, once modules exist) |
| **Blocked** | `detumble_mode.py` (OQ-4 thresholds), `orient_transition_mode.py` (OQ-5), `operational_mode.py` (OQ-6), `safe_mode.py` (OQ-7), `telemetry.py` (OQ-9 format, OQ-8) |

Good first picks: `persistence.py` (majority vote is easy to test) and
`watchdog_supervisor.py` (a small state machine).

## Legacy mapping

| Legacy (`simulator.py` state) | Now |
|---|---|
| `detumble` | DETUMBLE (exit must use sensed data, not `self.states`) |
| `idle` | SUN_POINT with eclipse hold |
| `target_point` | wheel PD behind GPS_POINT / GROUND_POINT |
| `point` (nadir) | archived |
| `demagnetize` | not a mode; part of `control/actuation_scheduler.py` |

The `check_state()` bugs (one if/elif chain; detumble exit on true state) are
marked `TODO(bug)` in `state_manager.py` and `detumble_mode.py`.
