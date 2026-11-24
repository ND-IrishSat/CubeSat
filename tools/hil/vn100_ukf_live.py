"""Live VN-100 viewer with our bench UKF running alongside it. Not flight code.

Two boards in the 3D view, both relative to the pose when the filter started:
    grey board  = the VN-100's own onboard filter (the reference)
    green board = our UKF (tools/hil/vn100_ukf.py) running live on raw gyro/accel/mag

Run from the repo root:
    python tools\\hil\\vn100_ukf_live.py --port COM5
    python tools\\hil\\vn100_ukf_live.py --replay logs\\vn100\\<file>.csv      (no hardware)

Start with the board RESTING STILL: the filter spends its first 2 s measuring the
reference directions (field and gravity) and the gyro bias. "Restart filter" redoes this.
"""
from __future__ import annotations

import argparse
import csv
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Optional

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtWidgets

from cloversat.datatypes.sample_types import Sample

sys.path.insert(0, str(Path(__file__).resolve().parent))      # find vn100_ukf.py next to this file
from vn100_ukf import UKF, normalize, quat_multiply, quat_to_matrix  # noqa: E402

HISTORY = 450          # points kept in each plot (~15 s at 30 Hz)
GUI_PERIOD_MS = 33     # redraw ~30 times per second
INVERT = False         # set True if the grey board rotates opposite to the real one
BOARD_OFFSET = 3.0     # boards are drawn this far either side of the centre

# VN-100 body frame: x forward, y right, z DOWN. Screen: z UP. 180 deg about x.
C_SCREEN = np.diag([1.0, -1.0, -1.0])

# dropdown text -> which measurements correct the gyro
MODES = {
    "gyro + accel + mag": "both",
    "gyro + accel (no mag)": "accel",
    "gyro only (no corrections)": "gyro",
}


# ======================= quaternion helpers =======================
def conj(q: np.ndarray) -> np.ndarray:
    """Inverse rotation of a unit quaternion."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def angle_between(q1: np.ndarray, q2: np.ndarray) -> float:
    """Smallest rotation [deg] between two orientations."""
    d = abs(float(np.dot(normalize(q1), normalize(q2))))
    return float(np.degrees(2 * np.arccos(min(1.0, d))))


def tilt_deg(q: np.ndarray) -> float:
    """How far the board is tipped from its starting pose [deg], ignoring turning."""
    w, x, y, z = q
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def to_transform(R: np.ndarray, offset=(0.0, 0.0, 0.0)) -> pg.Transform3D:
    """3x3 rotation + screen offset -> 4x4 pyqtgraph transform."""
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = offset
    return pg.Transform3D(*[float(v) for v in M.flatten()])


# ======================= 3D model =======================
def make_board(body_rgba, size=(3.0, 2.0, 0.6)) -> gl.GLMeshItem:
    """Flat box standing in for the board. Red face = front (+x), blue = top."""
    sx, sy, sz = (s / 2 for s in size)
    verts = np.array([
        [-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
        [-sx, -sy,  sz], [sx, -sy,  sz], [sx, sy,  sz], [-sx, sy,  sz],
    ])
    faces = np.array([
        [0, 1, 2], [0, 2, 3],   # bottom
        [4, 6, 5], [4, 7, 6],   # top
        [0, 4, 5], [0, 5, 1],   # side
        [2, 6, 7], [2, 7, 3],   # side
        [1, 5, 6], [1, 6, 2],   # front (+x)
        [0, 3, 7], [0, 7, 4],   # back
    ])
    colors = np.tile(body_rgba, (12, 1)).astype(float)
    colors[2:4] = [0.2, 0.5, 0.9, 1.0]     # top: blue
    colors[8:10] = [0.9, 0.3, 0.2, 1.0]    # front: red
    md = gl.MeshData(vertexes=verts, faces=faces, faceColors=colors)
    return gl.GLMeshItem(meshdata=md, smooth=False, drawEdges=True, edgeColor=(0, 0, 0, 1))


def make_axes(length: float = 2.2) -> list[gl.GLLinePlotItem]:
    """Body axes: red = x, green = y, blue = z (z points DOWN out of the board)."""
    items = []
    for i, color in enumerate([(1, 0, 0, 1), (0, 1, 0, 1), (0, 0.5, 1, 1)]):
        tip = C_SCREEN[:, i] * length
        items.append(gl.GLLinePlotItem(pos=np.array([[0, 0, 0], tip]),
                                       color=color, width=3, antialias=True))
    return items


# ======================= live UKF =======================
class LiveUkf:
    """Wraps UKF() for live use: 2 s still start-up, then one step per sample.

    Thread-safe: the reader thread calls update(); the GUI calls reset(),
    set_mode() and snapshot().
    """

    INIT_SECONDS = 2.0
    IGNORE = 1e6                       # "this sensor is useless" noise level
    R_SCALE = 10.0                     # same default as vn100_ukf_compare.py
    Q = np.diag([1e-7] * 4 + [1e-10] * 3)

    def __init__(self, mode: str = "both") -> None:
        self._lock = threading.Lock()
        self.mode = mode
        self.generation = 0            # +1 every time a start-up finishes
        self._reset_locked()

    def _reset_locked(self) -> None:
        self._buf: list = []
        self._t0: Optional[float] = None
        self._t_now: Optional[float] = None
        self._t_last: Optional[float] = None
        self._r_base = None
        self.refs = None
        self.state = None
        self.cov = None
        self.ready = False
        self.note = ""

    def reset(self) -> None:
        with self._lock:
            self._reset_locked()

    def set_mode(self, mode: str) -> None:
        with self._lock:
            self.mode = mode

    def _r(self) -> np.ndarray:
        r = self._r_base.copy()
        if self.mode in ("accel", "gyro"):
            r[0:3, 0:3] = np.eye(3) * self.IGNORE      # ignore mag
        if self.mode == "gyro":
            r[3:6, 3:6] = np.eye(3) * self.IGNORE      # ignore accel too
        return r

    def update(self, gyro: np.ndarray, mag: np.ndarray, accel: np.ndarray, t: float) -> None:
        with self._lock:
            self._t_now = t
            if not self.ready:                         # start-up: collect still samples
                if self._t0 is None:
                    self._t0 = t
                self._buf.append((gyro, mag, accel))
                if t - self._t0 >= self.INIT_SECONDS and len(self._buf) >= 10:
                    self._finish_init(t)
                return
            dt = t - self._t_last
            self._t_last = t
            if not 0.0 < dt < 0.5:                     # skip gaps / repeated samples
                return
            try:
                self.state, self.cov, _, _ = UKF(self.state, self.cov, self.Q, self._r(), dt,
                                                 gyro, mag, accel, self.refs)
            except np.linalg.LinAlgError:
                self._reset_locked()
                self.note = "filter restarted (covariance broke): hold still"

    def _finish_init(self, t: float) -> None:
        G = np.array([b[0] for b in self._buf])
        M = np.array([b[1] for b in self._buf])
        A = np.array([b[2] for b in self._buf])
        self.refs = (normalize(M.mean(axis=0)), normalize(A.mean(axis=0)))
        mag_dir = np.array([normalize(v) for v in M])
        acc_dir = np.array([normalize(v) for v in A])
        r_diag = np.r_[np.maximum(mag_dir.var(axis=0), 1e-6), np.maximum(acc_dir.var(axis=0), 1e-6)]
        self._r_base = np.diag(r_diag * self.R_SCALE)
        self.state = np.r_[1.0, 0.0, 0.0, 0.0, G.mean(axis=0)]   # start bias = still average
        # The bias was just measured while still, so tell the filter it's already
        # accurate (small variance). Otherwise, without mag, it wanders the z bias,
        # which it can't observe, and heading drifts badly (tested: 16.8 -> 4.9 deg).
        self.cov = np.diag([1e-4] * 4 + [1e-7] * 3)
        self._t_last = t
        self.ready = True
        self.generation += 1
        moved = np.degrees(G.std(axis=0)).max()
        self.note = ("board moved during start-up: press Restart and hold still"
                     if moved > 1.0 else "")

    def snapshot(self) -> dict:
        with self._lock:
            if not self.ready:
                progress = 0.0
                if self._t0 is not None and self._t_now is not None:
                    progress = min(1.0, (self._t_now - self._t0) / self.INIT_SECONDS)
                return {"ready": False, "progress": progress, "note": self.note,
                        "generation": self.generation}
            return {"ready": True, "q": self.state[:4].copy(), "bias": self.state[4:].copy(),
                    "note": self.note, "generation": self.generation}


# ======================= replay source (no hardware) =======================
class ReplayImu:
    """Plays a vn100_record.py CSV at real speed. Same methods as Vn100Driver."""

    def __init__(self, path: str, speed: float = 1.0) -> None:
        with open(path, newline="") as f:
            rows = [r for r in csv.DictReader(f) if r["valid"] == "1"]
        if not rows:
            raise ValueError(f"no valid rows in {path}")

        def col(*keys):
            return np.array([[float(r[k]) for k in keys] for r in rows])

        self.t = col("t")[:, 0]
        self.gyro = col("gyro_x", "gyro_y", "gyro_z")
        self.mag = col("mag_x", "mag_y", "mag_z")
        self.accel = col("accel_x", "accel_y", "accel_z")
        self.q = col("vn_qw", "vn_qx", "vn_qy", "vn_qz")
        self.speed = speed
        self.model = f"REPLAY {Path(path).name}"
        self.i = -1
        self._wall0: Optional[float] = None

    def init(self) -> None:
        self.i = -1
        self._wall0 = None

    def close(self) -> None:
        pass

    def read_attitude(self) -> Sample:
        self.i += 1
        if self.i >= len(self.t):
            raise RuntimeError("replay finished")
        if self._wall0 is None:
            self._wall0 = time.monotonic() - self.t[0] / self.speed
        wait = self._wall0 + self.t[self.i] / self.speed - time.monotonic()
        if wait > 0:
            time.sleep(wait)                      # play back at recorded speed
        return Sample(self.q[self.i], float(self.t[self.i]), True)

    def read_all(self):
        t = float(self.t[self.i])
        return (Sample(self.gyro[self.i], t, True), Sample(self.mag[self.i], t, True),
                Sample(self.accel[self.i], t, True))


# ======================= background reader =======================
class ImuPoller:
    """Reads the IMU nonstop in a thread, runs the UKF on every sample,
    and keeps the newest result for the GUI."""

    def __init__(self, imu, estimator: LiveUkf) -> None:
        self.imu = imu
        self.estimator = estimator
        self.error: Optional[str] = None
        self._lock = threading.Lock()
        self._latest = None
        self._count = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                q = self.imu.read_attitude()
                gyro, mag, accel = self.imu.read_all()
                if gyro.valid and mag.valid and accel.valid:
                    self.estimator.update(gyro.value, mag.value, accel.value, gyro.t)
                est = self.estimator.snapshot()
                with self._lock:
                    self._latest = (q, gyro, mag, accel, est)
                    self._count += 1
        except Exception as e:
            self.error = str(e) if "replay finished" in str(e) else f"{type(e).__name__}: {e}"

    def get(self):
        with self._lock:
            return self._latest, self._count

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.imu.close()


# ======================= window =======================
class Viewer(QtWidgets.QWidget):
    def __init__(self, poller: ImuPoller, estimator: LiveUkf, title: str) -> None:
        super().__init__()
        self.setWindowTitle(title)
        self.poller = poller
        self.estimator = estimator
        self.last_count = 0
        self.rate_count = 0
        self.rate_t = time.monotonic()
        self.t_start: Optional[float] = None
        self.gen_seen = 0
        self.q_vn0: Optional[np.ndarray] = None       # VN pose when the filter started
        self.q_vn_first: Optional[np.ndarray] = None  # VN pose before the filter is ready

        # ---- left: legend + 3D view ----
        left = QtWidgets.QVBoxLayout()
        legend = QtWidgets.QLabel("GREY board = VN-100 onboard filter (reference)     "
                                  "GREEN board = our UKF")
        legend.setStyleSheet("font-size: 14px; font-weight: bold;")
        left.addWidget(legend)
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=16, elevation=20, azimuth=-90)
        grid = gl.GLGridItem()
        grid.setSize(16, 16)
        grid.translate(0, 0, -2.5)
        self.view.addItem(grid)
        self.board_vn = make_board([0.55, 0.55, 0.58, 1.0])
        self.board_ukf = make_board([0.25, 0.7, 0.35, 1.0])
        self.axes_vn = make_axes()
        self.axes_ukf = make_axes()
        for item in [self.board_vn, self.board_ukf, *self.axes_vn, *self.axes_ukf]:
            self.view.addItem(item)
        self._draw(self.board_vn, self.axes_vn, np.eye(3), -BOARD_OFFSET)
        self._draw(self.board_ukf, self.axes_ukf, np.eye(3), +BOARD_OFFSET)
        left.addWidget(self.view, 1)

        # ---- right: readouts ----
        right = QtWidgets.QVBoxLayout()
        self.status = QtWidgets.QLabel("waiting for data...")
        self.status.setWordWrap(True)
        self.status.setStyleSheet("font-size: 14px; font-weight: bold; color: orange;")
        right.addWidget(self.status)
        readouts = QtWidgets.QGridLayout()
        self.values: dict[str, QtWidgets.QLabel] = {}
        names = ["UKF vs VN", "Tilt diff", "Rate", "|accel|", "|B|", "Bias |b|"]
        for i, name in enumerate(names):
            title_lbl = QtWidgets.QLabel(name)
            title_lbl.setStyleSheet("color: gray;")
            value = QtWidgets.QLabel("--")
            value.setStyleSheet("font-size: 18px; font-weight: bold;")
            readouts.addWidget(title_lbl, (i // 3) * 2, i % 3)
            readouts.addWidget(value, (i // 3) * 2 + 1, i % 3)
            self.values[name] = value
        right.addLayout(readouts)

        # ---- right: plots ----
        self.t_buf: deque = deque(maxlen=HISTORY)
        self.err_buf = {k: deque(maxlen=HISTORY) for k in ("total", "tilt")}
        self.bias_buf = [deque(maxlen=HISTORY) for _ in range(3)]
        self.gyro_buf = [deque(maxlen=HISTORY) for _ in range(3)]

        pw = pg.PlotWidget(title="UKF vs VN-100 [deg]")
        pw.addLegend()
        self.err_curves = {
            "total": pw.plot(pen=pg.mkPen("c", width=2), name="total angle"),
            "tilt": pw.plot(pen=pg.mkPen("g", width=2), name="tilt only"),
        }
        right.addWidget(pw)

        pw = pg.PlotWidget(title="UKF gyro bias estimate [deg/s]")
        pw.addLegend()
        self.bias_curves = [pw.plot(pen=pg.mkPen(c, width=2), name=n) for c, n in zip("rgb", "xyz")]
        right.addWidget(pw)

        pw = pg.PlotWidget(title="Gyro [deg/s]")
        pw.addLegend()
        self.gyro_curves = [pw.plot(pen=pg.mkPen(c, width=2), name=n) for c, n in zip("rgb", "xyz")]
        right.addWidget(pw)

        # ---- right: controls ----
        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Correct the gyro with:"))
        self.mode_box = QtWidgets.QComboBox()
        self.mode_box.addItems(list(MODES))
        self.mode_box.currentTextChanged.connect(lambda text: self.estimator.set_mode(MODES[text]))
        controls.addWidget(self.mode_box, 1)
        restart = QtWidgets.QPushButton("Restart filter (hold still 2 s)")
        restart.clicked.connect(self.restart)
        controls.addWidget(restart)
        right.addLayout(controls)

        layout = QtWidgets.QHBoxLayout(self)
        layout.addLayout(left, 3)
        layout.addLayout(right, 2)
        self.resize(1500, 900)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_view)
        self.timer.start(GUI_PERIOD_MS)

    # ------------------------------------------------------------------
    def _draw(self, board, axes, R: np.ndarray, y_offset: float) -> None:
        T = to_transform(C_SCREEN @ R @ C_SCREEN, (0.0, y_offset, 0.0))
        board.setTransform(T)
        for a in axes:
            a.setTransform(T)

    def _clear_history(self) -> None:
        for buf in [self.t_buf, *self.err_buf.values(), *self.bias_buf, *self.gyro_buf]:
            buf.clear()
        self.t_start = None

    def restart(self) -> None:
        self.estimator.reset()
        self.q_vn0 = None
        self.q_vn_first = None
        self._clear_history()

    def update_view(self) -> None:
        if self.poller.error:
            self.status.setText(f"SOURCE STOPPED: {self.poller.error}")
            return
        latest, count = self.poller.get()
        if latest is None or count == self.last_count:
            return
        self.rate_count += count - self.last_count
        self.last_count = count
        q, gyro, mag, accel, est = latest

        # ---- the moment a start-up finishes, both boards start from "level"
        if est["ready"] and est["generation"] != self.gen_seen and q.valid:
            self.gen_seen = est["generation"]
            self.q_vn0 = normalize(q.value)
            self._clear_history()

        # ---- grey board: VN-100, relative to the filter's starting pose
        vn_rel = None
        if q.valid:
            if self.q_vn_first is None:
                self.q_vn_first = normalize(q.value)
            ref = self.q_vn0 if self.q_vn0 is not None else self.q_vn_first
            vn_rel = quat_multiply(conj(ref), normalize(q.value))
            R_vn = quat_to_matrix(vn_rel)
            self._draw(self.board_vn, self.axes_vn, R_vn.T if INVERT else R_vn, -BOARD_OFFSET)

        if gyro.valid and mag.valid and accel.valid:
            self.values["|accel|"].setText(f"{np.linalg.norm(accel.value):.2f} m/s²")
            self.values["|B|"].setText(f"{np.linalg.norm(mag.value) * 1e6:.1f} µT")

        # ---- green board + comparison, once the filter is running
        if est["ready"]:
            q_ukf = est["q"]
            self._draw(self.board_ukf, self.axes_ukf, quat_to_matrix(q_ukf), +BOARD_OFFSET)
            if vn_rel is not None and gyro.valid:
                total = angle_between(q_ukf, vn_rel)
                tilt = abs(tilt_deg(q_ukf) - tilt_deg(vn_rel))
                bias_dps = np.degrees(est["bias"])
                self.values["UKF vs VN"].setText(f"{total:5.1f}°")
                self.values["Tilt diff"].setText(f"{tilt:5.1f}°")
                self.values["Bias |b|"].setText(f"{np.linalg.norm(bias_dps):.2f}°/s")

                if self.t_start is None:
                    self.t_start = gyro.t
                self.t_buf.append(gyro.t - self.t_start)
                self.err_buf["total"].append(total)
                self.err_buf["tilt"].append(tilt)
                gyro_dps = np.degrees(gyro.value)
                for i in range(3):
                    self.bias_buf[i].append(bias_dps[i])
                    self.gyro_buf[i].append(gyro_dps[i])
                t = np.array(self.t_buf)
                for key, curve in self.err_curves.items():
                    curve.setData(t, np.array(self.err_buf[key]))
                for i in range(3):
                    self.bias_curves[i].setData(t, np.array(self.bias_buf[i]))
                    self.gyro_curves[i].setData(t, np.array(self.gyro_buf[i]))
            self.status.setText(est["note"] or f"Filter running ({self.mode_box.currentText()}).")
        else:
            self._draw(self.board_ukf, self.axes_ukf, np.eye(3), +BOARD_OFFSET)
            for name in ("UKF vs VN", "Tilt diff", "Bias |b|"):
                self.values[name].setText("--")
            self.status.setText(f"Starting filter: keep the board STILL... "
                                f"{est['progress'] * 100:3.0f}%   {est['note']}")

        # ---- once a second: sample rate
        now = time.monotonic()
        if now - self.rate_t >= 1.0:
            self.values["Rate"].setText(f"{self.rate_count / (now - self.rate_t):.0f} Hz")
            self.rate_count, self.rate_t = 0, now

    def closeEvent(self, event) -> None:
        self.timer.stop()
        self.poller.stop()
        super().closeEvent(event)


def main() -> int:
    ap = argparse.ArgumentParser(description="VN-100 viewer with our UKF running live.")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="live VN-100 on this port, e.g. COM5")
    src.add_argument("--replay", help="play back a CSV from vn100_record.py instead")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed (1 = real time)")
    args = ap.parse_args()

    if args.replay:
        imu = ReplayImu(args.replay, speed=args.speed)
    else:
        from cloversat.config.device_params import Vn100Params
        from cloversat.devices.hardware.vn100_driver import Vn100Driver
        imu = Vn100Driver(Vn100Params(port=args.port, baud=args.baud))
    try:
        imu.init()
    except Exception as e:
        print(f"Could not start the data source: {e}")
        return 1
    print(f"Source: {imu.model}")

    app = pg.mkQApp("VN-100 + UKF")
    estimator = LiveUkf()
    poller = ImuPoller(imu, estimator)
    poller.start()
    win = Viewer(poller, estimator, f"VN-100 + UKF live ({imu.model})")
    win.show()
    app.exec()
    return 0


if __name__ == "__main__":
    sys.exit(main())