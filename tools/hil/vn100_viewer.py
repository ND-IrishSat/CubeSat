"""Live 3D viewer for the VN-100 testbed IMU. Not flight code.  (SKELETON: fill in the TODOs)

Run from the repo root:
    python tools\\hil\\training\\vn100_viewer_skeleton.py --port COM5

Fill in every `pass  # TODO`. Delete the `pass` once you've written the code.
Until a function is filled in it returns None, so the program will error
where that result is used. That's expected; work top to bottom.

Conventions:
    - Quaternions are [w, x, y, z], scalar first.
    - VN-100 body frame: x forward, y right, z DOWN.
    - Driver readings are Sample objects: .value (numpy array), .t (seconds), .valid (bool).
    - Units: gyro rad/s, mag tesla, accel m/s^2.
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from collections import deque
from typing import Optional

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtWidgets

from cloversat.config.device_params import Vn100Params
from cloversat.devices.hardware.vn100_driver import Vn100Driver

HISTORY = 300          # points kept in each plot (~10 s at 30 Hz)
GUI_PERIOD_MS = 33     # redraw ~30 times per second
INVERT = False         # set True if the model rotates opposite to the board

# VN-100 body frame: x forward, y right, z DOWN. Screen: z UP. 180 deg about x.
C_SCREEN = np.diag([1.0, -1.0, -1.0])


# ======================= math helpers =======================
def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """[w, x, y, z] -> 3x3 rotation matrix (body -> NED)."""
    # 1. unpack w, x, y, z from q
    # 2. return the 3x3 rotation matrix as np.array([[...], [...], [...]])
    #    (diagonal: 1 - 2*(y*y + z*z) ...; off-diagonal: 2*(x*y - w*z) ...)
    pass  # TODO


def matrix_to_euler_deg(R: np.ndarray) -> tuple[float, float, float]:
    """Rotation matrix (body -> NED) -> (roll, pitch, yaw) in degrees, aerospace ZYX order."""
    # roll  = atan2(R[2,1], R[2,2])
    # pitch = -asin(R[2,0])        (clip R[2,0] to [-1, 1] first)
    # yaw   = atan2(R[1,0], R[0,0])
    # convert to degrees, return as three floats
    pass  # TODO


def to_transform(R: np.ndarray) -> pg.Transform3D:
    """3x3 rotation -> 4x4 pyqtgraph transform."""
    # 1. M = 4x4 identity
    # 2. put R in the top-left 3x3 of M
    # 3. return pg.Transform3D(*[float(v) for v in M.flatten()])
    pass  # TODO


# ======================= 3D model =======================
def make_board(size=(3.0, 2.0, 0.6)) -> gl.GLMeshItem:
    """Flat box standing in for the board. Red face = front (+x), blue = top."""
    sx, sy, sz = (s / 2 for s in size)
    # 8 corners of the box (given)
    verts = np.array([
        [-sx, -sy, -sz], [sx, -sy, -sz], [sx, sy, -sz], [-sx, sy, -sz],
        [-sx, -sy,  sz], [sx, -sy,  sz], [sx, sy,  sz], [-sx, sy,  sz],
    ])
    # 12 triangles, 2 per face, as indexes into verts (given)
    faces = np.array([
        [0, 1, 2], [0, 2, 3],   # bottom
        [4, 6, 5], [4, 7, 6],   # top
        [0, 4, 5], [0, 5, 1],   # side
        [2, 6, 7], [2, 7, 3],   # side
        [1, 5, 6], [1, 6, 2],   # front (+x)
        [0, 3, 7], [0, 7, 4],   # back
    ])
    # 1. colors: one RGBA row per triangle (12 rows), grey by default (np.tile)
    # 2. make the top triangles (rows 2-3) blue and the front ones (rows 8-9) red
    # 3. md = gl.MeshData(vertexes=verts, faces=faces, faceColors=colors)
    # 4. return gl.GLMeshItem(meshdata=md, smooth=False, drawEdges=True, edgeColor=(0, 0, 0, 1))
    pass  # TODO


def make_axes(length: float = 2.5) -> list[gl.GLLinePlotItem]:
    """Body axes: red = x, green = y, blue = z (z points DOWN out of the board)."""
    # for each axis i (0, 1, 2) with its color (red, green, blue):
    #     tip = C_SCREEN[:, i] * length
    #     make gl.GLLinePlotItem(pos=np.array([[0, 0, 0], tip]), color=..., width=3, antialias=True)
    # return the list of 3 line items
    pass  # TODO


# ======================= background reader =======================
class ImuPoller:
    """Reads the IMU nonstop in a thread; the GUI grabs the newest reading."""

    def __init__(self, imu: Vn100Driver) -> None:
        # store: the imu, an error message (None), a threading.Lock,
        # the latest reading (None), a reading counter (0), a threading.Event to stop,
        # and a threading.Thread(target=self._run, daemon=True)
        pass  # TODO

    def start(self) -> None:
        # start the thread
        pass  # TODO

    def _run(self) -> None:
        # try:
        #     loop until the stop event is set:
        #         q = read_attitude(), then gyro, mag, accel = read_all()
        #         under the lock: save (q, gyro, mag, accel) and add 1 to the counter
        # except Exception as e:
        #     save f"{type(e).__name__}: {e}" as the error message
        pass  # TODO

    def get(self):
        # under the lock: return (latest reading, counter)
        pass  # TODO

    def stop(self) -> None:
        # set the stop event, join the thread (timeout=2.0), close the imu
        pass  # TODO


# ======================= window =======================
class Viewer(QtWidgets.QWidget):
    def __init__(self, poller: ImuPoller, port: str) -> None:
        super().__init__()
        self.setWindowTitle(f"VN-100 live viewer ({port})")
        self.poller = poller
        self.R0: Optional[np.ndarray] = None   # pose captured by "Zero"
        self.last_count = 0
        self.rate_count = 0
        self.rate_t = time.monotonic()
        self.rate_hz = 0.0
        self.t_start: Optional[float] = None

        # ---- left: 3D view ----
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=10, elevation=20, azimuth=-60)
        # TODO: add a gl.GLGridItem() moved down with .translate(0, 0, -2)
        # TODO: self.board = make_board(), add it to self.view
        # TODO: self.axes = make_axes(), add each one to self.view

        # ---- right: readouts, plots, buttons ----
        right = QtWidgets.QVBoxLayout()

        readouts = QtWidgets.QGridLayout()
        self.values: dict[str, QtWidgets.QLabel] = {}
        names = ["Roll", "Pitch", "Yaw", "|accel|", "|B|", "Rate"]
        for i, name in enumerate(names):
            title = QtWidgets.QLabel(name)
            title.setStyleSheet("color: gray;")
            value = QtWidgets.QLabel("--")
            value.setStyleSheet("font-size: 18px; font-weight: bold;")
            readouts.addWidget(title, (i // 3) * 2, i % 3)
            readouts.addWidget(value, (i // 3) * 2 + 1, i % 3)
            self.values[name] = value
        right.addLayout(readouts)

        # rolling buffers: self.t_buf = times, self.bufs[key][i] = values,
        # key in "gyro"/"mag"/"accel", i = 0,1,2 for x,y,z
        self.t_buf: deque = deque(maxlen=HISTORY)
        self.bufs = {k: [deque(maxlen=HISTORY) for _ in range(3)] for k in ("gyro", "mag", "accel")}
        self.curves = {}
        for key, title in (("gyro", "Gyro [deg/s]"), ("mag", "Mag [uT]"), ("accel", "Accel [m/s^2]")):
            pw = pg.PlotWidget(title=title)
            pw.addLegend()
            self.curves[key] = [pw.plot(pen=pg.mkPen(c, width=2), name=n) for c, n in zip("rgb", "xyz")]
            right.addWidget(pw)

        self.status = QtWidgets.QLabel("waiting for data...")
        right.addWidget(self.status)
        zero_btn = QtWidgets.QPushButton("Zero orientation (make current pose 'level')")
        # TODO: connect the button's clicked signal to self.zero
        right.addWidget(zero_btn)

        layout = QtWidgets.QHBoxLayout(self)
        layout.addWidget(self.view, 3)
        layout.addLayout(right, 2)
        self.resize(1400, 850)

        # TODO: make a QtCore.QTimer, connect its timeout to self.update_view,
        #       start it with GUI_PERIOD_MS (store it as self.timer)

    def zero(self) -> None:
        # clear the saved zero pose so the next reading becomes the new reference
        pass  # TODO

    def update_view(self) -> None:
        # 1. if the poller has an error: show it in self.status and return
        # 2. latest, count = poller.get(); return if nothing new (None, or same count as last time)
        # 3. add the new readings to self.rate_count, remember count in self.last_count
        # 4. q, gyro, mag, accel = latest

        # ---- plots + health readouts (only if gyro, mag and accel are all valid) ----
        # 5. first sample: remember its time in self.t_start
        # 6. append (gyro.t - self.t_start) to self.t_buf
        # 7. append x/y/z of gyro in deg/s, mag in uT (* 1e6), accel in m/s^2 to self.bufs
        # 8. setData(t, values) on every curve in self.curves
        # 9. show |accel| (m/s²) and |B| (µT) with np.linalg.norm in self.values

        # ---- 3D board + roll/pitch/yaw (only if q is valid) ----
        # 10. R = quat_to_matrix(q.value); if INVERT, use R.T instead
        # 11. roll, pitch, yaw = matrix_to_euler_deg(R); show them in self.values
        # 12. if self.R0 is None: self.R0 = R
        # 13. T = to_transform(C_SCREEN @ (self.R0.T @ R) @ C_SCREEN)
        # 14. setTransform(T) on self.board and every item in self.axes

        # ---- once a second: update rate + status ----
        # 15. if at least 1 s since self.rate_t:
        #         rate = self.rate_count / elapsed time; reset rate_count and rate_t
        #         show it in self.values["Rate"]; set status to "OK" or "INVALID SAMPLES"
        pass  # TODO

    def closeEvent(self, event) -> None:
        # stop the timer, stop the poller (closes the serial port), then:
        super().closeEvent(event)


def main() -> int:
    # 1. argparse: --port (required), --baud (int, default 115200)
    # 2. imu = Vn100Driver(Vn100Params(port=..., baud=...))
    # 3. try imu.init(); on any exception print a message and return 1
    # 4. app = pg.mkQApp("VN-100 viewer")
    # 5. make an ImuPoller and start it
    # 6. make the Viewer, .show() it, then app.exec()
    # 7. return 0
    pass  # TODO


if __name__ == "__main__":
    sys.exit(main())