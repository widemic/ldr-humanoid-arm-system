#!/usr/bin/env python3
"""
Decompose Essential Matrix into Rotation and Translation
Standard computer vision implementation
"""

import numpy as np


def decomposeEssentialMatrix(E):
    """
    Decompose essential matrix E into rotation R and translation t.

    Returns 4 possible solutions: [R1, R2], t
    where t is the unit translation vector (up to scale)

    Args:
        E: 3x3 essential matrix

    Returns:
        Rots: list of 2 rotation matrices [R1, R2]
        u3: 3x1 translation direction (unit vector)
    """
    # SVD decomposition
    U, S, Vt = np.linalg.svd(E)

    # Ensure proper rotation (det = 1)
    if np.linalg.det(U) < 0:
        U = -U
    if np.linalg.det(Vt) < 0:
        Vt = -Vt

    # Translation is the last column of U
    u3 = U[:, 2].reshape(3, 1)

    # Rotation matrix construction
    W = np.array([
        [0, -1, 0],
        [1, 0, 0],
        [0, 0, 1]
    ])

    # Two possible rotations
    R1 = U @ W @ Vt
    R2 = U @ W.T @ Vt

    # Ensure proper rotation matrices (det = 1)
    if np.linalg.det(R1) < 0:
        R1 = -R1
    if np.linalg.det(R2) < 0:
        R2 = -R2

    return [R1, R2], u3
