"""File-backed simulated gyroscope.

Purpose:  Implement the simulated device behind ``GyroInterface``.
Inputs:   A CSV file containing ``timestamp``, ``gyro_x``, ``gyro_y``, and
          ``gyro_z`` columns. Gyro values are in rad/s.
Outputs:  ``Sample[Vec3]`` gyro readings in rad/s.
Status:   development
"""
from __future__ import annotations

import csv
from pathlib import Path
from typing import Optional, Union

import numpy as np

from cloversat.datatypes.sample_types import Sample, SelfTestResult
from cloversat.datatypes.vector_types import Vec3
from cloversat.devices.interfaces.gyro_interface import GyroInterface


class GyroSim(GyroInterface):
    """Simulated rate gyro."""

    def __init__(self, file_path: Union[str, Path]) -> None:
        self._file_path = Path(file_path)
        self._samples: list[Sample[Vec3]] = []
        self._index = 0
        self._initialized = False

    def init(self) -> None:
        """Initialize the simulated device."""
        if self._initialized:
            return

        with self._file_path.open(newline="") as file:
            reader = csv.DictReader(file)
            if reader.fieldnames is None:
                raise ValueError(f"gyro file has no header: {self._file_path}")

            timestamp_column = self._timestamp_column(reader.fieldnames)
            required_columns = {timestamp_column, "gyro_x", "gyro_y", "gyro_z"}
            missing = required_columns.difference(reader.fieldnames)
            if missing:
                names = ", ".join(sorted(missing))
                raise ValueError(f"gyro file is missing columns: {names}")

            self._samples = [
                self._parse_row(row, row_number, timestamp_column)
                for row_number, row in enumerate(reader, start=2)
            ]

        if not self._samples:
            raise ValueError(f"gyro file contains no samples: {self._file_path}")
        self._initialized = True

    def self_test(self) -> SelfTestResult:
        """Run the simulated device's self-test."""
        if not self._initialized:
            return SelfTestResult(False, code=1, detail="gyro is not initialized")
        return SelfTestResult(True)

    def read_gyro(self) -> Sample[Vec3]:
        """Return simulated body rates ``omega_body`` [rad/s]."""
        if not self._initialized:
            raise RuntimeError("gyro must be initialized before reading")
        if self._index >= len(self._samples):
            raise RuntimeError("gyro file has no unread samples")

        sample = self._samples[self._index]
        self._index += 1
        return sample

    def _timestamp_column(self, fieldnames: list[str]) -> str:
        for name in ("timestamp", "t", "elapsed_s"):
            if name in fieldnames:
                return name
        raise ValueError("gyro file must contain timestamp, t, or elapsed_s")

    def _parse_row(self, row: dict[str, Optional[str]], row_number: int, timestamp_column: str,) -> Sample[Vec3]:
        try:
            timestamp_text = row[timestamp_column]
            gyro_x_text = row["gyro_x"]
            gyro_y_text = row["gyro_y"]
            gyro_z_text = row["gyro_z"]

            timestamp = float(timestamp_text or "")
            gyro_x = float(gyro_x_text or "")
            gyro_y = float(gyro_y_text or "")
            gyro_z = float(gyro_z_text or "")

            value = np.array([gyro_x, gyro_y, gyro_z], dtype=np.float64)
        except (TypeError, ValueError) as error:
            raise ValueError(f"invalid gyro sample on CSV row {row_number}") from error

        if not np.isfinite(timestamp) or not np.all(np.isfinite(value)):
            raise ValueError(f"non-finite gyro sample on CSV row {row_number}")
        return Sample(value=value, t=timestamp)

#simulated file io
#feed file with timestamped data
#look at vectornav for formatting