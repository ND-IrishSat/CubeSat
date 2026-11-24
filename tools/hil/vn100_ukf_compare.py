"""Run the bench UKF on a recorded VN-100 CSV and compare it with the VN's own filter.

Not flight code. Offline: no hardware needed, so you can rerun and tune freely.

Run from the repo root:
    python tools\\hil\\vn100_ukf_compare.py logs\\vn100\\vn100_20261123_185231_slow-rotations.csv
    python tools\\hil\\vn100_ukf_compare.py <csv> --save plots\\ukf_vs_vn.png   (for slides)

What it does:
    1. Loads the CSV written by vn100_record.py.
    2. Uses the "still" rows at the start to define the reference frame:
       B_ref, g_ref = average mag and accel directions while the board rests.
       The filter's orientation is therefore RELATIVE TO THE STARTING POSE.
    3. Runs UKF() from vn100_ukf.py on every row (gyro predicts, mag+accel correct).
    4. Also integrates the gyro alone (no corrections) to show drift.
    5. Compares both with the VN-100's onboard attitude, also made relative
       to its starting pose, using the rotation angle between the two.

Plots need matplotlib:  python -m pip install matplotlib
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))   # find vn100_ukf.py next to this file
from vn100_ukf import UKF, normalize, propagate, quat_multiply  # noqa: E402


# ---------------------------------------------------------------- helpers
def load_csv(path: str) -> dict[str, np.ndarray]:
    """CSV from vn100_record.py -> dict of column name -> numpy array."""
    with open(path, newline="") as f:
        rows = [r for r in csv.DictReader(f) if r["valid"] == "1"]
    if not rows:
        raise SystemExit(f"No valid rows in {path}")
    data = {k: np.array([float(r[k]) for r in rows]) for k in rows[0] if k != "phase"}
    data["phase"] = np.array([r["phase"] for r in rows])
    return data


def conj(q: np.ndarray) -> np.ndarray:
    """Quaternion conjugate = inverse rotation (for unit quaternions)."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    """Smallest rotation angle [deg] that takes orientation q1 to q2."""
    d = abs(float(np.dot(normalize(q1), normalize(q2))))  # abs: q and -q are the same rotation
    return float(np.degrees(2 * np.arccos(min(1.0, d))))


def tilt_deg(q: np.ndarray) -> float:
    """How far the board's z axis is tipped from where it started [deg] (ignores turning)."""
    w, x, y, z = q
    zz = 1 - 2 * (x * x + y * y)          # R[2,2]: start-z axis seen in the current pose
    return float(np.degrees(np.arccos(np.clip(zz, -1.0, 1.0))))


# ---------------------------------------------------------------- main
def main() -> int:
    ap = argparse.ArgumentParser(description="Bench UKF vs VN-100 onboard filter, from a CSV.")
    ap.add_argument("csv", help="recording from vn100_record.py")
    ap.add_argument("--q-quat", type=float, default=1e-7, help="process noise, quaternion part")
    ap.add_argument("--q-bias", type=float, default=1e-10, help="process noise, gyro bias part")
    ap.add_argument("--r-scale", type=float, default=10.0,
                    help="multiply the measured still-phase noise by this for r "
                         "(accel also sees motion, so r is inflated)")
    ap.add_argument("--alpha", type=float, default=1.0, help="UKF sigma point spread")
    ap.add_argument("--bias-init", choices=["zero", "still"], default="zero",
                    help="start the bias at 0 (shows the filter learning it) "
                         "or at the still-phase gyro average")
    ap.add_argument("--save", help="save the plot to this PNG instead of only showing it")
    ap.add_argument("--no-plot", action="store_true", help="print the summary only")
    args = ap.parse_args()

    d = load_csv(args.csv)
    t = d["t"]
    gyro = np.c_[d["gyro_x"], d["gyro_y"], d["gyro_z"]]
    mag = np.c_[d["mag_x"], d["mag_y"], d["mag_z"]]
    accel = np.c_[d["accel_x"], d["accel_y"], d["accel_z"]]
    q_vn = np.c_[d["vn_qw"], d["vn_qx"], d["vn_qy"], d["vn_qz"]]
    still = d["phase"] == "still"
    if still.sum() < 10:
        raise SystemExit("Need at least ~10 'still' rows at the start to set up the filter.")

    # ---- 1. reference frame = the resting starting pose
    B_ref = normalize(mag[still].mean(axis=0))
    g_ref = normalize(accel[still].mean(axis=0))
    bias_still = gyro[still].mean(axis=0)

    # ---- 2. noise levels from the still rows (directions, so unit-less)
    mag_dir = np.array([normalize(v) for v in mag[still]])
    acc_dir = np.array([normalize(v) for v in accel[still]])
    r_diag = np.r_[np.maximum(mag_dir.var(axis=0), 1e-6), np.maximum(acc_dir.var(axis=0), 1e-6)]
    r = np.diag(r_diag * args.r_scale)
    q = np.diag([args.q_quat] * 4 + [args.q_bias] * 3)

    # ---- 3. initial state and covariance
    bias0 = bias_still if args.bias_init == "still" else np.zeros(3)
    state = np.r_[1.0, 0.0, 0.0, 0.0, bias0]
    cov = np.diag([1e-4] * 4 + [1e-5] * 3)
    gyro_only = np.r_[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # no corrections, no bias removal

    q_vn0_inv = conj(normalize(q_vn[0]))

    n = len(t)
    err_ukf = np.zeros(n)
    err_gyro = np.zeros(n)
    tilt_err = np.zeros(n)
    bias_hist = np.zeros((n, 3))
    bias_hist[0] = state[4:]

    for k in range(1, n):
        dt = t[k] - t[k - 1]
        try:
            state, cov, _, _ = UKF(state, cov, q, r, dt, gyro[k], mag[k], accel[k],
                                   (B_ref, g_ref), alpha=args.alpha)
        except np.linalg.LinAlgError:
            print(f"UKF covariance broke at t={t[k]:.2f} s (row {k}). "
                  "Try a larger --q-quat or --r-scale.")
            return 1
        gyro_only = propagate(gyro_only, gyro[k], dt)

        # VN relative to its own starting pose: same frame as our filter
        vn_rel = quat_multiply(q_vn0_inv, normalize(q_vn[k]))
        err_ukf[k] = angle_between(state[:4], vn_rel)
        err_gyro[k] = angle_between(gyro_only[:4], vn_rel)
        tilt_err[k] = abs(tilt_deg(state[:4]) - tilt_deg(vn_rel))
        bias_hist[k] = state[4:]

    # ---- 4. summary
    move = d["phase"] == "move"
    end = d["phase"] == "end_still"
    end_settled = end & (t >= t[-1] - 5.0)          # last 5 s: board really at rest
    print(f"\nFile: {args.csv}")
    print(f"Rows: {n}   duration: {t[-1]:.1f} s   rate: {n / t[-1]:.0f} Hz")
    print(f"Reference: |B| {np.linalg.norm(mag[still].mean(0)) * 1e6:.1f} uT, "
          f"|g| {np.linalg.norm(accel[still].mean(0)):.2f} m/s^2")
    print("\nAngle between our estimate and the VN-100's (deg):")
    print(f"  {'':16s}{'UKF':>10s}{'gyro only':>12s}")
    for label, mask in (("while moving", move), ("end (settled)", end_settled)):
        if mask.any():
            print(f"  {label:16s}{err_ukf[mask].mean():10.2f}{err_gyro[mask].mean():12.2f}   (mean)")
    print(f"  {'worst overall':16s}{err_ukf.max():10.2f}{err_gyro.max():12.2f}")
    print(f"\nTilt-only difference (no heading), mean: {tilt_err[1:].mean():.2f} deg")
    print("\nGyro bias [rad/s]:")
    print(f"  UKF estimate at end : {np.round(state[4:], 4)}")
    print(f"  still-phase average : {np.round(bias_still, 4)}")

    if args.no_plot:
        return 0

    # ---- 5. plots
    try:
        import matplotlib
        if args.save:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n(matplotlib not installed: python -m pip install matplotlib  to see plots)")
        return 0

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    ax = axes[0]
    ax.plot(t, err_gyro, label="gyro only (no corrections)", color="tab:gray")
    ax.plot(t, err_ukf, label="our UKF", color="tab:blue")
    ax.set_ylabel("angle vs VN-100 [deg]")
    ax.set_title("Bench UKF vs VN-100 onboard filter")
    ax.legend(loc="upper left")

    ax = axes[1]
    ax.plot(t, tilt_err, color="tab:green")
    ax.set_ylabel("tilt difference [deg]")

    ax = axes[2]
    for i, (c, name) in enumerate(zip(("tab:red", "tab:green", "tab:blue"), "xyz")):
        ax.plot(t, bias_hist[:, i], color=c, label=f"bias {name}")
        ax.axhline(bias_still[i], color=c, linestyle="--", linewidth=0.8)
    ax.set_ylabel("gyro bias [rad/s]")
    ax.set_xlabel("time [s]")
    ax.legend(loc="upper right", fontsize=8)

    for ax in axes:                                    # mark the phases
        if still.any():
            ax.axvspan(t[0], t[still][-1], color="0.9", zorder=0)
        if end.any():
            ax.axvspan(t[end][0], t[-1], color="0.9", zorder=0)
        ax.grid(alpha=0.3)

    fig.tight_layout()
    if args.save:
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=150)
        print(f"\nSaved plot to {args.save}")
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())