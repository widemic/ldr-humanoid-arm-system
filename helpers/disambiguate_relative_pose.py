#!/usr/bin/env python3
"""
Disambiguate Relative Pose - Select correct R,t from 4 possibilities
Standard computer vision implementation
"""

import numpy as np


def disambiguateRelativePose(Rots, u3, p1, p2, K1, K2):
    """
    Disambiguate between the 4 possible relative pose solutions.

    Given two rotation matrices and one translation direction,
    there are 4 possible [R|t] combinations. We test which one
    results in 3D points in front of both cameras.

    Args:
        Rots: list of 2 rotation matrices [R1, R2]
        u3: 3x1 translation direction (unit vector, can be +u3 or -u3)
        p1: 2xN points in first image (normalized coordinates)
        p2: 2xN points in second image (normalized coordinates)
        K1: 3x3 camera matrix for first camera
        K2: 3x3 camera matrix for second camera

    Returns:
        R: 3x3 correct rotation matrix
        T: 3x1 correct translation vector
    """
    # Create 4 possible pose combinations
    poses = []
    for R in Rots:
        poses.append((R, u3))    # +translation
        poses.append((R, -u3))   # -translation

    # Camera matrices
    M1 = K1 @ np.hstack([np.eye(3), np.zeros((3, 1))])  # [I | 0]

    max_valid = 0
    best_pose = None

    # Test each pose
    for R, t in poses:
        M2 = K2 @ np.hstack([R, t])  # [R | t]

        # Triangulate points
        valid_count = 0

        # For each point correspondence
        for i in range(p1.shape[1]):
            # Triangulate using DLT
            A = np.zeros((4, 4))
            A[0] = p1[0, i] * M1[2, :] - M1[0, :]
            A[1] = p1[1, i] * M1[2, :] - M1[1, :]
            A[2] = p2[0, i] * M2[2, :] - M2[0, :]
            A[3] = p2[1, i] * M2[2, :] - M2[1, :]

            # Solve for 3D point
            _, _, Vt = np.linalg.svd(A)
            X = Vt[-1]
            X = X / X[3]  # Normalize

            # Check if point is in front of both cameras
            # Camera 1: Z > 0
            if X[2] > 0:
                # Camera 2: transformed point has positive Z
                X_cam2 = R @ X[:3].reshape(3, 1) + t
                if X_cam2[2] > 0:
                    valid_count += 1

        # Keep the pose with most valid points
        if valid_count > max_valid:
            max_valid = valid_count
            best_pose = (R, t)

    if best_pose is None:
        # Fallback to first pose if none are valid
        return Rots[0], u3

    return best_pose[0], best_pose[1]
