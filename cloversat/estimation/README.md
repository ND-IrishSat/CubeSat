# estimation/ — Layer 3

Turns raw device samples into **one `StateEstimate`** per control tick: attitude,
rates, gyro bias, position, velocity, eclipse flag, covariance and validity
flags. Everything above this layer (control, executive) reads only that object.
Nothing here reads sim truth.

Each file's docstring covers its inputs, outputs and port notes. This page
explains how the files fit together.

## One tick (`estimator.py`)

`Estimator.step()` runs the pieces in this order:

```
clock ─► sgp4_propagator ─► sun_model / igrf_model / eclipse_model / station_model
                                 │ (reference vectors in ECI, eclipse flag)
device Samples ─► gyro_ / mag_ / sun_conditioning   (calibrate, gate, validate)
                                 │ (clean body-frame vectors)
             attitude_init (until attitude valid) ─► attitude_filter
                                 │
                                 ▼
                           StateEstimate
```

1. **Time:** `clock.py` gives monotonic time plus RTC. It syncs to GPS time when
   a fix arrives, and the GPS↔UTC offset is handled only here.
2. **Orbit:** `sgp4_propagator.py` propagates the preloaded TLE and re-seeds from
   each GPS fix. There is no uplink, so the TLE never refreshes. Running on the
   TLE alone is a *degraded* state because its error grows by about a km per day.
3. **Environment models:** these take position and time and return reference
   vectors: the sun direction (`sun_model`), the magnetic field (`igrf_model`,
   OQ-2), eclipse state with the next entry and exit (`eclipse_model`), and the
   station direction and visibility (`station_model`).
4. **Conditioning:** calibrates, gates and validity-checks each sensor. It lives
   here rather than in `devices/` because it needs estimation-layer inputs, and
   because it must behave the same for real and sim devices.
5. **Attitude:** `attitude_init.py` finds a first attitude (OQ-5), then
   `attitude_filter.py` tracks q, ω and gyro bias (OQ-1). `attitude_valid`
   stays false until initialization succeeds. While it's false, the executive
   holds in ORIENT_TRANSITION.

Detumble needs neither of these. It only uses conditioned rates and a clean
magnetometer reading.

## Cross-layer handshake: mag-clean flag

The magnetorquers corrupt magnetometer readings while they're on.
`control/actuation_scheduler.py` owns the torquers and publishes a `mag_clean`
flag during its SAMPLE window. `main.py` passes that flag into
`Estimator.step(..., mag_clean=...)`, and `mag_conditioning.py` **rejects any
mag sample not flagged clean**.

This is the one place where estimation depends on a control-layer output. It
receives the flag as data, so the layering rule still holds.

## Breaking the circular dependency

Magnetometer attitude needs position (for the reference field), GPS needs
attitude (to point its antenna at the sky), and pointing needs attitude. The
loop is broken in this order:

1. Detumble needs neither position nor attitude.
2. The preloaded TLE with SGP4 gives a position good enough for the field and
   sun models.
3. A coarse attitude comes from sun + mag (TRIAD) or mag + gyro.
4. The GPS patch only needs to point roughly skyward.
5. GPS then refines the position.

## Sun vector fallback ladder

The sun sensor gives the sun direction in the body frame; `sun_model` gives it
in the inertial frame. When the sensor can't be trusted, fall back in this
order:

1. Drop the bad sample and propagate with the gyro.
2. Exclude the bad channel.
3. Use the filter's predicted vector, `s_body = R(q)·s_eci`.
4. Search for the sun (rotate to maximize panel current).
5. Spin in SAFE to average power.

## Files

| Status | Files |
|---|---|
| **Ready to start** | `clock.py`, `sgp4_propagator.py`, `sun_model.py`, `eclipse_model.py`, `station_model.py`, `triad.py`, `mag_conditioning.py`, `gyro_conditioning.py` *(legacy LPF below stub)*, `estimator.py` |
| **Blocked** | `igrf_model.py` (OQ-2), `attitude_init.py` (OQ-5), `attitude_filter.py` (OQ-1; legacy UKF below stub), `sun_conditioning.py` (HW: sun sensor) |

Good first picks: `sun_model.py`, `eclipse_model.py` and `triad.py`. They're
small, pure functions that are easy to test.

## Watch out for

- The legacy UKF and `hfunc` take the field in **µT**. Everything new uses tesla.
- The legacy UKF state is `[q, ω]` with no gyro bias, and it propagates using
  the sim's equations of motion and the *true* field. Don't carry that
  structure over.
- The legacy `simulator.determine_attitude()` reads an undefined `self.B_true`,
  which would also be sim truth. See `TODO(bug)` in `estimator.py`.
