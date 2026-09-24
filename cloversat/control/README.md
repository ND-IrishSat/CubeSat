# control/ — Layer 2

Turns a `ModeCommand` (from the executive) and a `StateEstimate` (from
estimation) into actuator commands. It's the only layer that decides torque or
dipole, and `actuation_scheduler.py` is the only module allowed to drive the
torquers.

Each file's docstring covers its inputs, outputs and port notes. This page
explains how the files fit together.

## One tick

```
ModeCommand + StateEstimate
        │
        ▼
  controller.py ── picks target + law for the mode
        │
   ┌────┴─────────────────────────┐
   │ *_target.py → AttitudeTarget │   (sun / gps / ground; slew_limiter later)
   │ *_law.py    → torque, dipole │
   └────┬─────────────────────────┘
        ▼
  ActuatorCommand {wheel_torque, dipole, source}
        │
   ┌────┴────────────────┐
   ▼                     ▼
wheel_allocation     torquer_allocation      (per-wheel cmds / volts + power cap)
   ▼                     ▼
wheel_loop (OQ-3)    actuation_scheduler ──► torquers  (+ publishes mag_clean)
   ▼
wheels
```

Wheels run every tick. Torquers only drive during the scheduler's ACTUATE
window.

## Which law and target per mode (`controller.py`)

| Mode / sub-mode | Target | Law(s) | Actuators |
|---|---|---|---|
| DETUMBLE | none | `bcross_law` (B-dot fallback if gyro invalid) | torquers only |
| ORIENT_TRANSITION | from `attitude_init` progress | detumble law or hold | torquers (wheels once attitude is valid) |
| OPERATIONAL / SUN_POINT | `sun_target` (eclipse → inertial hold) | `attitude_pd_law` (+ `gyro_comp_law` later) | wheels |
| OPERATIONAL / GPS_POINT | `gps_target` (GPS patch → zenith) | `attitude_pd_law` | wheels |
| OPERATIONAL / GROUND_POINT | `ground_target` (comm patch → station, tracking rate) | `attitude_pd_law` | wheels |
| OPERATIONAL / MOMENTUM_DUMP | keeps current target | `attitude_pd_law` + `momentum_dump_law` | wheels + torquers |
| SAFE | measured sun vector (no filter needed) | detumble law, then `attitude_pd_law` | minimal |

Actuator strategy:
- **Detumble uses torquers only.** Wheels can store momentum but not remove it,
  they'd likely saturate at tip-off rates, and they cost power before the
  panels deploy.
- **In normal operations the wheels are primary.** Torquers are only used for
  momentum dumping, inside the scheduler's windows.

## Targets and the secondary (roll) goal

Each `*_target.py` chooses a **primary** pair: a body vector (panel normal or a
patch normal, from `SpacecraftParams`) and a reference direction (the sun, the
zenith `r/|r|`, or the station direction `r_station − r`). Aligning that pair
leaves one free rotation, the roll about the primary axis. A **secondary** pair
uses that roll, for example to aim the comm patch at Earth while sun pointing.

The shared math lives once in `geometry/pointing.py`. When the secondary is
degenerate, fall back to a default roll rule. A target sets
`AttitudeTarget.valid = False` when its goal is impossible, such as sun
pointing in eclipse or a station that isn't visible.

## Actuation scheduler (torquer ↔ magnetometer)

Each cycle runs:

```
ACTUATE → DEMAG (reverse pulse for the ferro-core rods) → SETTLE (off)
        → SAMPLE (mag read, mag_clean = True) → COMPUTE (new dipole) → repeat
```

This applies in detumble too, because B-cross and B-dot need clean field
readings. `mag_clean` goes to `estimation/mag_conditioning.py` via `main.py`
(see `estimation/README.md`). Phase lengths are set in `ControlParams`. The
air-core torquer can later be compensated by subtraction instead of being
switched off.

## Files

| Work package (ID) | File | Status |
|---|---|---|
| C1 | `bcross_law.py` | Ready (port `B_dot`, fix ÷0) |
| C2 | `sun_target.py`, `gps_target.py` | Ready |
| C2 | `ground_target.py` | Blocked: HW (patch placement) |
| C3 | `attitude_pd_law.py` | Ready (PD only; add q/−q sign flip) |
| C4 | `gyro_comp_law.py` | Later |
| C5 | `slew_limiter.py` | Later |
| C6 | `momentum_dump_law.py` | Ready |
| C7 | `wheel_allocation.py` | Ready (fix `W_inv` row 3 sign) |
| C8 | `torquer_allocation.py` | Ready |
| C9 | `actuation_scheduler.py` | Ready (extract from `simulator.py`) |
| C10 | `wheel_loop.py` | Blocked: OQ-3 |
| C11 | `controller.py` | Ready once C1, C2 and C3 have signatures you can rely on |

Good first picks: C1, C6 and C7. Each is a short formula with a known legacy
bug to fix and test.

## Defaults (unless the controls team objects)

- PD only, with no integral term. The legacy PID has no anti-windup.
- Rate-limited PD slews for v1. Eigenaxis and time-optimal slews come later.
- Momentum dumping triggers on a wheel-momentum threshold with hysteresis, and
  it's allowed during sun pointing.
