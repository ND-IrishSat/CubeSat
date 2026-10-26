import numpy as np
import pytest

from cloversat.control.bcross_law import bcross_dipole, bdot_dipole

def test_bcross_value_and_damping():
    B = np.array([0.0, 0.0, 2e-5])
    omega = np.array([0.1, 0.0, 0.0])

    m = bcross_dipole(omega, B, gain=1e-4)

    np.testing.assert_allclose(m, [0.0, -0.5, 0.0], atol=1e-12)
    assert m.shape == (3,)
    assert m.dtype == np.float64
    assert np.dot(omega, np.cross(m, B)) < 0.0


def test_bcross_zero_field_and_invalid_shape():
    np.testing.assert_array_equal(
        bcross_dipole(np.array([0.1, 0.0, 0.0]), np.zeros(3), 1e-4),
        np.zeros(3),
    )
    with pytest.raises(ValueError):
        bcross_dipole(np.zeros(2), np.ones(3), 1e-4)


def test_bdot_two_point_value_and_sign():
    history = [
        np.array([-1e-6, 0.0, 2e-5]),
        np.array([0.0, 0.0, 2e-5]),
    ]

    m = bdot_dipole(history, sample_dt=1.0, gain=1e-4)

    np.testing.assert_allclose(m, [-0.25, 0.0, 0.0], atol=1e-12)
    assert m.shape == (3,)
    assert m.dtype == np.float64


def test_bdot_five_point_value_and_sign():
    history = [
        np.array([x * 1e-6, 0.0, 2e-5])
        for x in (-4, -3, -2, -1, 0)
    ]

    np.testing.assert_allclose(
        bdot_dipole(history, sample_dt=1.0, gain=1e-4),
        [-0.25, 0.0, 0.0],
        atol=1e-12,
    )


def test_bdot_insufficient_history_and_zero_field():
    np.testing.assert_array_equal(bdot_dipole([], 1.0, 1e-4), np.zeros(3))
    np.testing.assert_array_equal(
        bdot_dipole([np.ones(3)], 1.0, 1e-4), np.zeros(3)
    )
    np.testing.assert_array_equal(
        bdot_dipole([np.ones(3), np.zeros(3)], 1.0, 1e-4),
        np.zeros(3),
    )


@pytest.mark.parametrize("sample_dt", [0.0, -1.0, np.nan])
def test_bdot_rejects_invalid_sample_spacing(sample_dt):
    with pytest.raises(ValueError):
        bdot_dipole([np.ones(3), np.ones(3)], sample_dt, 1e-4)


def test_bdot_rejects_invalid_sample_shape():
    with pytest.raises(ValueError):
        bdot_dipole([np.ones(3), np.ones(2)], 1.0, 1e-4)