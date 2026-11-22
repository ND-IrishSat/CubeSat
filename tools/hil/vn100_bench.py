"""Bench check for the VN-100 testbed IMU. Not flight code."""
import argparse
import time

import numpy as np

from cloversat.config.device_params import Vn100Params
from cloversat.devices.hardware.vn100_driver import Vn100Driver


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hz", type=float, default=10.0)
    args = ap.parse_args()

    imu = Vn100Driver(Vn100Params(port=args.port, baud=args.baud, sample_rate_hz=args.hz))
    imu.init()
    print("Connected:", imu.model)
    print("Self test:", imu.self_test())

    try:
        while True:
            gyro, mag, accel = imu.read_all()
            if not gyro.valid:
                print("invalid sample")
            else:
                print(f"t={gyro.t:10.3f}  "
                      f"gyro[rad/s]={np.round(gyro.value, 4)}  "
                      f"mag[uT]={np.round(mag.value * 1e6, 2)}  "
                      f"accel[m/s^2]={np.round(accel.value, 3)}")
            time.sleep(1.0 / args.hz)
    except KeyboardInterrupt:
        pass
    finally:
        imu.close()


if __name__ == "__main__":
    main()