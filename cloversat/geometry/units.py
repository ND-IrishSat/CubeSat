"""Unit conversions.

Purpose:  Convert legacy / datasheet units to SI at module boundaries.
Inputs:   Scalars or numpy arrays.
Outputs:  Same shape, converted.
Status:   [NEW]
Port from: none. Legacy code mixes uT and T and deg/rad throughout
           (see legacy/params.py ``DEGREES``, ``CONSTANT_B_FIELD_MAG``).

Flight code is SI everywhere (docs/conventions.md). Use these only when
reading datasheet values or legacy data.

Related open questions: none.
"""
from __future__ import annotations

from typing import TypeVar

import numpy as np
import numpy.typing as npt

Scalar = TypeVar("Scalar", float, npt.NDArray[np.float64])


def ut_to_t(x: Scalar) -> Scalar:
    """Microtesla -> tesla."""
    return x*1e-6


def t_to_ut(x: Scalar) -> Scalar:
    """Tesla -> microtesla."""
    return x * 1e6


def deg_to_rad(x: Scalar) -> Scalar:
    """Degrees -> radians."""
    return x * (np.pi / 180.0)


def rad_to_deg(x: Scalar) -> Scalar:
    """Radians -> degrees."""
    return x * (180.0 / np.pi)


def rpm_to_rad_s(x: Scalar) -> Scalar:
    """Revolutions per minute -> rad/s."""
    return x * (2.0 * np.pi / 60.0)


def rad_s_to_rpm(x: Scalar) -> Scalar:
    """rad/s -> revolutions per minute."""
    return x * (60.0 / (2.0 * np.pi))

def gauss_to_t(x: Scalar) -> Scalar:
    """Gauss -> tesla (1 G = 1e-4 T). VN-100 reports magnetic field in gauss."""
    return x * 1e-4

