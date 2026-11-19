"""Two-vector alignment.

Purpose:  Build the attitude that aligns a primary body axis with a primary
          reference direction, and gets a secondary body axis as close as
          possible to a secondary reference direction.
Inputs:   Unit-ish vectors: two in body frame, two in inertial frame.
Outputs:  ``Quat`` (body <- inertial).
Status:   [NEW]
Port from: none. See legacy/Utils/transformations.py ``vector_to_quaternion``
           for the old single-vector version (copied into
           ``geometry/quaternion.py``'s legacy block).

Used by every pointing target in ``control/*_target.py`` (e.g. panel normal ->
sun with a patch aimed at Earth in free roll).

Degenerate case: if the primary and secondary references (or body vectors)
are parallel, the secondary constraint is undefined. Fall back to any
perpendicular secondary and report it via the ``degenerate`` flag.

Related open questions: none.
"""
from __future__ import annotations

from cloversat.datatypes.vector_types import Quat, Vec3
import numpy as np


def align(
    primary_body: Vec3,
    primary_ref: Vec3,
    secondary_body: Vec3,
    secondary_ref: Vec3,
) -> Quat:
    """Attitude that puts ``primary_body`` on ``primary_ref`` exactly and
    ``secondary_body`` as close as possible to ``secondary_ref``.

    Refs are in the inertial frame. Uses the degenerate fallback when the
    secondary pair is parallel to the primary pair.
    """
    primary_body = normalize(primary_body)
    primary_ref = normalize(primary_ref)
    secondary_body = normalize(secondary_body)
    secondary_ref = normalize(secondary_ref)

    if is_degenerate(primary_body, secondary_body):
        secondary_body = fallback_secondary(primary_body)

    if is_degenerate(primary_ref, secondary_ref):
        secondary_ref = fallback_secondary(primary_ref)

    raise NotImplementedError


def is_degenerate(primary: Vec3, secondary: Vec3, tol_rad: float = 1e-3) -> bool:
    """
    Check whether two unit vectors are nearly collinear.
    Returns true if ``primary`` and ``secondary`` are within ``tol_rad`` of parallel

    Purpose:
        A secondary vector is needed to constrain rotation
        around the primary pointing axis.

        For example, consider a pencil pointing along +Y.
        Rotating the pencil around its own axis does not
        change its primary pointing direction. A secondary
        vector attached to the side of the pencil allows us
        to distinguish between these orientations.

        If the secondary vector is collinear with the primary,
        it provides no additional orientation information.

    Assumes both input vectors are unit vectors.
    """
    cross = np.cross(primary, secondary)
    return np.linalg.norm(cross) < np.sin(tol_rad)


def fallback_secondary(primary: Vec3) -> Vec3:
    """
    Helper for align() that generates a replacement secondary vector when the original one is degenerate.

    Cross product needs a second vector to find a vector perpendicular to the primary vector passed in

    Any unit vector perpendicular to ``primary`` (deterministic choice).
    """
    primary = normalize(primary)
    reference = np.array([1.0, 0.0, 0.0])       # Start by assigning +X direction to be our reference. Arbitrary choice
    
    if is_degenerate(primary, reference):       # If the primary vector is near parallel to the reference, the cross product is the zero vector which doesn't help 
        reference = np.array([0.0, 1.0, 0.0])   # pick +Y as the reserve/backup reference vector    

    perpendicular = np.cross(primary, reference)    # Find a vector perpendicular to the primary
    return normalize(perpendicular)

"""
Helper function: normalize a vector so it's magnitude is 1
"""
def normalize(v: Vec3) -> Vec3:
    """Normalize a vector to unit magnitude."""
    v = np.asarray(v, dtype=float)      # convert Vec3 to numpy array so we can do np operations on it
    
    # For robustness
    if v.shape != (3,):
        raise ValueError("Expected a 3D vector")
    # For robustness
    if not np.all(np.isfinite(v)):
        raise ValueError("Vector contains non-finite values")

    magnitude = np.linalg.norm(v)     # Calculate the magnitude

    if magnitude == 0 or not np.isfinite(magnitude):
        raise ValueError("Invalid vector magnitude")

    return v / magnitude