#!/usr/bin/env python3
"""
Linear Triangulation - DLT method for 3D point reconstruction
Standard computer vision implementation
"""

import numpy as np


def linearTriangulation(p1, p2, M1, M2):
    """
    Triangulate 3D points from 2D correspondences using DLT (Direct Linear Transform).

    Args:
        p1: 2xN array of points in first image (normalized or pixel coordinates)
        p2: 2xN array of points in second image (normalized or pixel coordinates)
        M1: 3x4 camera projection matrix for first view [K1 | 0] or [K1*R1 | K1*t1]
        M2: 3x4 camera projection matrix for second view [K2*R2 | K2*t2]

    Returns:
        P: 4xN array of homogeneous 3D points [X, Y, Z, W]
           To get 3D coordinates: P[:3] / P[3]
    """
    N = p1.shape[1]
    P = np.zeros((4, N))

    for i in range(N):
        # Build the linear system A * X = 0
        # where X is the 3D point in homogeneous coordinates
        A = np.zeros((4, 4))

        # Equations from first image
        # x1 * P[2,:] - P[0,:] = 0
        A[0] = p1[0, i] * M1[2, :] - M1[0, :]

        # y1 * P[2,:] - P[1,:] = 0
        A[1] = p1[1, i] * M1[2, :] - M1[1, :]

        # Equations from second image
        # x2 * P[2,:] - P[0,:] = 0
        A[2] = p2[0, i] * M2[2, :] - M2[0, :]

        # y2 * P[2,:] - P[1,:] = 0
        A[3] = p2[1, i] * M2[2, :] - M2[1, :]

        # Solve using SVD
        # The solution is the last column of V (corresponding to smallest singular value)
        _, _, Vt = np.linalg.svd(A)
        X = Vt[-1]

        # Store homogeneous 3D point
        P[:, i] = X

    return P
