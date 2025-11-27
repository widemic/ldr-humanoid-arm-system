#!/usr/bin/env python3
"""
Visual Odometry implementation matching yv1es/visual-odometry-from-scratch
Adapted for webcam/IP camera stream with real-time visualization.
"""

import os
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
from dataclasses import dataclass
from typing import List, Tuple, Optional

# Import helper functions from repo
from helpers.decompose_essential_matrix import decomposeEssentialMatrix
from helpers.disambiguate_relative_pose import disambiguateRelativePose
from helpers.linear_triangulation import linearTriangulation


@dataclass
class State:
    """State containing keypoints, landmarks, and candidates"""
    i: int  # Frame number

    P: np.ndarray  # Keypoints (2, K) - tracked 2D points with 3D landmarks
    X: np.ndarray  # Landmarks (3, K) - 3D points in world coordinates

    C: np.ndarray  # Candidate keypoints (2, M) - current positions
    F: np.ndarray  # Candidate first observations (2, M) - where first seen
    Tau: np.ndarray  # Candidate first camera poses (12, M) - [R|t] when first seen


class VisualOdometry:
    """Visual Odometry matching the reference implementation"""

    def __init__(self, K: np.ndarray, show_viz: bool = True):
        self.K = K
        self.K_inv = np.linalg.inv(K)
        self.show_viz = show_viz

        # Parameters (optimized for better performance)
        self.K_BOOTSTRAP = 15  # Frames for bootstrapping
        self.MIN_LOCALIZATION_K = 5  # Minimum keypoints for PnP
        self.PNP_RANSAC_ITERATIONS = 3000
        self.MIN_INLIERS = 6  # Reduced from 10 to handle low keypoint situations
        self.MAX_TRACKED_KEYPOINTS = 5000
        self.MAX_CANDIDATES = 5000
        self.TRIANGULATION_ANGLE_THRESHOLD = np.deg2rad(1.0)  # Reduced from 1.5° to 1.0° for more triangulations
        self.MIN_BASELINE_METERS = 0.01  # Reduced from 2cm to 1cm - more lenient
        self.KLT_WINDOW = 21  # Increased from 5 to 21 for better tracking
        self.KLT_PYR_LEVELS = 3  # Reduced from 5 to 3 for faster processing
        self.FRAME_SKIP = 5  # Increased from 2 to 5 - skip more frames for larger baseline
        self.frame_counter = 0  # Track frames for skipping

        # State
        self.state: Optional[State] = None
        self.pose_history: List[np.ndarray] = []
        self.trajectory: List[Tuple[float, float]] = [(0.0, 0.0)]
        self.kp_counts: List[int] = []
        self.cand_counts: List[int] = []

        # Current absolute pose in world frame
        self.current_pose = np.eye(4)  # World-to-camera transformation

        # Bootstrap images buffer
        self.bootstrap_images: List[np.ndarray] = []
        self.bootstrapped = False

        # Visualization
        if self.show_viz:
            self._init_viz()

    def _init_viz(self):
        """Initialize matplotlib visualization"""
        plt.ion()
        fig = plt.figure(figsize=(14, 7))
        gs = gridspec.GridSpec(2, 2, width_ratios=[1.2, 1.5], height_ratios=[1.2, 1])

        self.ax_img = fig.add_subplot(gs[0, 0])
        self.ax_img.set_title("Frame and Keypoints")
        self.ax_img.axis("off")

        self.ax_counts = fig.add_subplot(gs[1, 0])
        self.ax_counts.set_title("Keypoints and Candidate Count")
        self.ax_counts.set_xlabel("Frame")
        self.ax_counts.set_ylabel("Count")

        self.ax_traj = fig.add_subplot(gs[:, 1])
        self.ax_traj.set_title("Local Trajectory (300 poses) and Landmarks")
        self.ax_traj.set_xlabel("X")
        self.ax_traj.set_ylabel("Z")
        self.ax_traj.set_aspect("equal")
        self.ax_traj.grid(True, linestyle="--", alpha=0.3)

        plt.tight_layout()
        self.fig = fig

    def bootstrap(self, images: List[np.ndarray]) -> bool:
        """Bootstrap VO using Shi-Tomasi + KLT + 8-point RANSAC"""
        print("-" * 50)
        print(f"Bootstrapping with {len(images)} images")

        # Detect corners in first frame
        init_pts = cv2.goodFeaturesToTrack(
            images[0],
            maxCorners=3000,
            qualityLevel=0.001,
            minDistance=5.0
        )
        if init_pts is None:
            print("Failed to detect corners in first frame")
            return False

        init_pts = init_pts.reshape(-1, 2)
        print(f"Detected {len(init_pts)} initial corners")

        # Track through all bootstrap frames
        pts_tracked = init_pts.reshape(-1, 1, 2)
        img_prev = images[0]
        for i in range(1, len(images)):
            pts_tracked, status, _ = cv2.calcOpticalFlowPyrLK(
                img_prev, images[i], pts_tracked, None,
                winSize=(self.KLT_WINDOW, self.KLT_WINDOW),
                maxLevel=self.KLT_PYR_LEVELS
            )
            status = status.reshape(-1)
            pts_tracked = pts_tracked[status == 1]
            img_prev = images[i]

        pts_last = pts_tracked.squeeze(1)
        print(f"After KLT tracking, {len(pts_last)} corners remain to frame {len(images)-1}")

        # Single-pass check
        pts_single, status_single, _ = cv2.calcOpticalFlowPyrLK(
            images[0], images[-1], init_pts.reshape(-1, 1, 2), None,
            winSize=(self.KLT_WINDOW, self.KLT_WINDOW),
            maxLevel=self.KLT_PYR_LEVELS
        )
        status_single = status_single.reshape(-1)
        p0_in = init_pts[status_single == 1]
        pN_in = pts_single[status_single == 1].squeeze(1)
        print(f"Single-pass KLT: {len(pN_in)} / {len(init_pts)} survive to last frame")

        # Find fundamental matrix with RANSAC
        F, mask_F = cv2.findFundamentalMat(
            p0_in, pN_in,
            cv2.FM_8POINT + cv2.FM_RANSAC,
            ransacReprojThreshold=1.0,
            confidence=0.99
        )

        if F is None:
            print("Failed to find fundamental matrix")
            return False

        mask_F = mask_F.ravel()
        inliers = mask_F.sum()
        print(f"Found F with {inliers}/{len(mask_F)} inliers")

        pts1 = p0_in[mask_F == 1]
        pts2 = pN_in[mask_F == 1]
        pts1_h = np.r_[pts1.T, np.ones((1, inliers))]
        pts2_h = np.r_[pts2.T, np.ones((1, inliers))]

        # Essential matrix
        E = self.K.T @ F @ self.K
        print("\nE:\n", E)

        # Decompose using repo's method
        Rots, u3 = decomposeEssentialMatrix(E)
        R, t = disambiguateRelativePose(Rots, u3, pts1_h, pts2_h, self.K, self.K)

        # Triangulate using repo's linear method
        M1 = self.K @ np.eye(3, 4)
        M2 = self.K @ np.c_[R, t]
        X = linearTriangulation(pts1_h, pts2_h, M1, M2)[:-1]  # Remove homogeneous coordinate

        # Filter points behind cameras
        ok1 = X[2, :] > 0
        X_cam2 = R @ X + t[:, None]
        ok2 = X_cam2[2, :] > 0
        valid = ok1 & ok2

        X = X[:, valid]
        P = pts2_h[:-1, valid]

        print(f"Triangulated {X.shape[1]} landmarks")

        # Initialize state
        self.state = State(
            i=len(images) - 1,
            P=P,  # (2, K)
            X=X,  # (3, K)
            C=np.array([]).reshape((2, 0)),  # (2, 0)
            F=np.array([]).reshape((2, 0)),  # (2, 0)
            Tau=np.array([]).reshape((12, 0))  # (12, 0)
        )

        # Store pose
        pose = np.zeros((3, 4))
        pose[:3, :3] = R
        pose[:3, 3] = t.ravel()
        self.pose_history.append(pose)

        # Camera center for trajectory
        cam_center = -R.T @ t.ravel()
        self.trajectory.append((cam_center[0], cam_center[2]))

        print(f"\nBootstrap landmarks: X=[{X[0].min():.2f}, {X[0].max():.2f}], "
              f"Y=[{X[1].min():.2f}, {X[1].max():.2f}], "
              f"Z=[{X[2].min():.2f}, {X[2].max():.2f}]")
        print(f"Bootstrap camera 2 position: ({cam_center[0]:.3f}, {cam_center[1]:.3f}, {cam_center[2]:.3f})")

        self.bootstrapped = True
        print("Bootstrap complete!")
        return True

    def process_frame(self, image_curr: np.ndarray, image_prev: np.ndarray):
        """Process one frame (continuous operation)"""
        if not self.bootstrapped or self.state is None:
            raise RuntimeError("Must bootstrap before processing frames")

        # Frame skipping to accumulate motion
        self.frame_counter += 1
        if self.frame_counter % self.FRAME_SKIP != 0:
            return  # Skip this frame

        self.state.i += 1
        print(f"\n{'*' * 60}\nProcessing frame {self.state.i} (actual frame {self.frame_counter})\n{'*' * 60}")

        # ========== LOCALIZATION ==========
        print("\n......... Localizing camera pose .........")

        p_prev = self.state.P.T.reshape(-1, 1, 2).astype(np.float32)
        num_keypoints = self.state.P.shape[1]

        if num_keypoints < self.MIN_LOCALIZATION_K:
            print(f"WARNING: Not enough keypoints: {num_keypoints}/{self.MIN_LOCALIZATION_K}")
            return

        # KLT tracking
        p_curr, status, _ = cv2.calcOpticalFlowPyrLK(
            image_prev, image_curr, p_prev, None,
            winSize=(self.KLT_WINDOW, self.KLT_WINDOW),
            maxLevel=self.KLT_PYR_LEVELS
        )
        status = status.reshape(-1)
        p_curr_klt = p_curr[status == 1].reshape(-1, 2)
        X_klt = self.state.X[:, status == 1]
        print(f"KLT found {np.sum(status == 1)} keypoint correspondences")

        # PnP RANSAC with optimized parameters
        print(f"Running PnP RANSAC on {X_klt.shape[1]} correspondences")
        print(f"Landmark stats: X=[{X_klt[0].min():.2f}, {X_klt[0].max():.2f}], "
              f"Y=[{X_klt[1].min():.2f}, {X_klt[1].max():.2f}], "
              f"Z=[{X_klt[2].min():.2f}, {X_klt[2].max():.2f}]")

        # Check if landmarks are degenerate (all at same point)
        landmark_spread = np.max([
            X_klt[0].max() - X_klt[0].min(),
            X_klt[1].max() - X_klt[1].min(),
            X_klt[2].max() - X_klt[2].min()
        ])
        if landmark_spread < 0.05:  # Less than 5cm spread
            print(f"WARNING: Landmarks are degenerate (spread={landmark_spread*100:.2f}cm)! Need to re-bootstrap.")
            print("Please move camera more to accumulate sufficient parallax.")
            return

        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            X_klt.T,  # (N, 3)
            p_curr_klt,  # (N, 2)
            self.K,
            distCoeffs=None,
            flags=cv2.SOLVEPNP_ITERATIVE,  # ITERATIVE for stability with many points
            reprojectionError=4.0,  # Balanced threshold (was 8.0, now 4.0)
            confidence=0.99,
            iterationsCount=self.PNP_RANSAC_ITERATIONS
        )

        if not success or inliers is None or len(inliers) <= self.MIN_INLIERS:
            print(f"PnP RANSAC failed! Inliers: {len(inliers) if inliers is not None else 0}")
            return

        inliers = inliers.flatten()
        R, _ = cv2.Rodrigues(rvec)
        pose_curr = np.zeros((3, 4))
        pose_curr[:3, :3] = R
        pose_curr[:3, 3] = tvec.flatten()
        self.pose_history.append(pose_curr)

        # Update trajectory
        cam_center = -R.T @ tvec.flatten()
        self.trajectory.append((cam_center[0], cam_center[2]))
        print(f"PnP tvec: ({tvec[0, 0]:.3f}, {tvec[1, 0]:.3f}, {tvec[2, 0]:.3f})")
        print(f"Camera center: ({cam_center[0]:.3f}, {cam_center[1]:.3f}, {cam_center[2]:.3f})")
        print(f"Trajectory point: ({cam_center[0]:.3f}, {cam_center[2]:.3f})")
        print(f"R determinant: {np.linalg.det(R):.3f}")

        # Filter outliers to maintain healthy landmark set
        inlier_ratio = len(inliers) / num_keypoints
        p1_inliers = p_curr_klt[inliers]
        X_inliers = X_klt[:, inliers]
        self.state.P = p1_inliers.T
        self.state.X = X_inliers
        print(f"Found pose with {len(inliers)}/{num_keypoints} inliers ({inlier_ratio*100:.1f}%)")

        if inlier_ratio < 0.5:
            print(f"WARNING: Low inlier ratio ({inlier_ratio*100:.1f}%) - tracking may be degrading")

        # ========== TRIANGULATION ==========
        print("\n......... Triangulating new landmarks .........")

        if self.state.C.shape[1] > 0:
            # Track candidates
            c_prev = self.state.C.T.reshape(-1, 1, 2).astype(np.float32)
            c_curr, status_klt, _ = cv2.calcOpticalFlowPyrLK(
                image_prev, image_curr, c_prev, None,
                winSize=(self.KLT_WINDOW, self.KLT_WINDOW),
                maxLevel=self.KLT_PYR_LEVELS
            )
            status_klt = status_klt.reshape(-1)
            c_prev_klt = c_prev[status_klt == 1].reshape(-1, 2)
            c_curr_klt = c_curr[status_klt == 1].reshape(-1, 2)
            F_klt = self.state.F[:, status_klt == 1]
            Tau_klt = self.state.Tau[:, status_klt == 1]
            print(f"KLT found {np.sum(status_klt == 1)} candidate correspondences")

            # Compute triangulation angles
            R_end = pose_curr[:, :3]
            t_end = pose_curr[:, 3]
            angles = []

            for i_cand in range(c_curr_klt.shape[0]):
                M_start = Tau_klt[:, i_cand].reshape(3, 4)
                R_start = M_start[:, :3]

                x_start, y_start = F_klt[0, i_cand], F_klt[1, i_cand]
                d_start_cam = self.K_inv @ np.array([x_start, y_start, 1.0])
                d_start_world = R_start.T @ d_start_cam

                x_end, y_end = c_curr_klt[i_cand, 0], c_curr_klt[i_cand, 1]
                d_end_cam = self.K_inv @ np.array([x_end, y_end, 1.0])
                d_end_world = R_end.T @ d_end_cam

                d_start_norm = d_start_world / np.linalg.norm(d_start_world)
                d_end_norm = d_end_world / np.linalg.norm(d_end_world)

                dot = np.clip(np.dot(d_start_norm, d_end_norm), -1.0, 1.0)
                angle = np.arccos(dot)
                angles.append(angle)

            angles = np.array(angles)
            to_triangulate = np.where(angles > self.TRIANGULATION_ANGLE_THRESHOLD)[0]
            if len(angles) > 0:
                print(f"Angle stats: min={np.rad2deg(angles.min()):.3f}°, max={np.rad2deg(angles.max()):.3f}°, mean={np.rad2deg(angles.mean()):.3f}°")
            print(f"{len(to_triangulate)} candidates have large enough angle (threshold={np.rad2deg(self.TRIANGULATION_ANGLE_THRESHOLD):.2f}°)")

            if len(to_triangulate) > 0:
                M_end = self.K @ pose_curr

                # Group by M_start
                from collections import defaultdict
                pose_dict = defaultdict(list)
                for idx in to_triangulate:
                    M_start = self.K @ Tau_klt[:, idx].reshape((3, 4))
                    M_start_key = tuple(M_start.flatten())
                    pose_dict[M_start_key].append(idx)

                # Triangulate each group
                for M_start_key, group_idx in pose_dict.items():
                    M_start_pose = Tau_klt[:, group_idx[0]].reshape(3, 4)  # [R|t] without K
                    M_start = self.K @ M_start_pose
                    c_start_group = F_klt[:, group_idx]
                    c_curr_group = c_curr_klt[group_idx].T

                    # Check baseline distance between cameras
                    R_start = M_start_pose[:, :3]
                    t_start = M_start_pose[:, 3]
                    R_end = pose_curr[:, :3]
                    t_end = pose_curr[:, 3]

                    # Camera centers in world frame
                    C_start = -R_start.T @ t_start
                    C_end = -R_end.T @ t_end
                    baseline = np.linalg.norm(C_end - C_start)

                    # Skip if baseline is too small (use the configurable threshold)
                    if baseline < self.MIN_BASELINE_METERS:
                        print(f"  Skipping group: baseline too small ({baseline*100:.1f}cm, need >{self.MIN_BASELINE_METERS*100:.1f}cm)")
                        continue

                    print(f"  ✓ Triangulating {len(group_idx)} points with baseline {baseline*100:.2f}cm")

                    # Use linear triangulation like in bootstrap
                    c_start_h = np.vstack([c_start_group, np.ones(c_start_group.shape[1])])
                    c_curr_h = np.vstack([c_curr_group, np.ones(c_curr_group.shape[1])])

                    X_new_h = linearTriangulation(c_start_h, c_curr_h, M_start, M_end)
                    X_new = X_new_h[:3, :]

                    # Check depth in both cameras
                    C1 = R_start @ X_new + t_start.reshape(3, 1)
                    C2 = R_end @ X_new + t_end.reshape(3, 1)

                    # Also check reasonable depth (between 0.1m and 50m)
                    mask_in_front = (C1[2, :] > 0.1) & (C2[2, :] > 0.1) & (C1[2, :] < 50) & (C2[2, :] < 50)

                    X_new = X_new[:, mask_in_front]
                    c_curr_filtered = c_curr_group[:, mask_in_front]

                    # Add to state
                    if X_new.shape[1] > 0:
                        print(f"  Adding {X_new.shape[1]} new landmarks (filtered {np.sum(~mask_in_front)} behind camera)")
                        print(f"  Landmark X: [{X_new[0].min():.1f}, {X_new[0].max():.1f}], "
                              f"Y: [{X_new[1].min():.1f}, {X_new[1].max():.1f}], "
                              f"Z: [{X_new[2].min():.1f}, {X_new[2].max():.1f}]")
                    self.state.P = np.hstack((self.state.P, c_curr_filtered))
                    self.state.X = np.hstack((self.state.X, X_new))

                # Remove triangulated candidates
                mask_keep = np.ones(len(c_prev_klt), dtype=bool)
                mask_keep[to_triangulate] = False
                c_curr_klt = c_curr_klt[mask_keep]
                F_klt = F_klt[:, mask_keep]
                Tau_klt = Tau_klt[:, mask_keep]

            # Update candidates
            self.state.C = c_curr_klt.T
            self.state.F = F_klt
            self.state.Tau = Tau_klt

        # ========== ADD NEW CANDIDATES ==========
        print("\n......... Adding fresh candidates .........")

        if self.state.C.shape[1] < self.MAX_CANDIDATES:
            corners = cv2.goodFeaturesToTrack(
                image_curr,
                maxCorners=5000,  # Increased from 3000
                qualityLevel=0.005,  # Reduced from 0.01 to detect more features
                minDistance=8.0  # Reduced from 10.0 for denser coverage
            )

            if corners is not None:
                corners = corners.reshape(-1, 2)
                print(f"Detected {len(corners)} potential candidates")

                # Filter by distance to existing points
                if self.state.C.size > 0:
                    existing_points = np.hstack([self.state.P, self.state.C]).T
                    dist_matrix = np.linalg.norm(
                        corners[:, None, :] - existing_points[None, :, :], axis=2
                    )
                    min_dist = 3.0
                    valid_mask = np.all(dist_matrix > min_dist, axis=1)
                    distinct_corners = corners[valid_mask]
                else:
                    distinct_corners = corners

                if distinct_corners.size > 0:
                    distinct_corners = distinct_corners[:self.MAX_CANDIDATES - self.state.C.shape[1]]
                    print(f"Adding {len(distinct_corners)} fresh candidates")

                    # Store pose for each candidate
                    new_poses = np.tile(pose_curr.flatten()[:, None], (1, len(distinct_corners)))
                    self.state.C = np.hstack((self.state.C, distinct_corners.T))
                    self.state.F = np.hstack((self.state.F, distinct_corners.T))
                    self.state.Tau = np.hstack((self.state.Tau, new_poses))

        # Status summary
        num_keypoints = self.state.P.shape[1]
        num_candidates = self.state.C.shape[1]
        print(f"\n{'='*60}")
        print(f"STATUS: {num_keypoints} keypoints | {num_candidates} candidates")

        # Health check
        if num_keypoints < 20:
            print(f"🚨 CRITICAL: Very low keypoint count ({num_keypoints})!")
            print(f"   ACTION: Move camera FASTER or change viewing direction!")
            print(f"   TIP: Current FRAME_SKIP={self.FRAME_SKIP}, increase if camera moves slowly")
        elif num_keypoints < 50:
            print(f"⚠️  WARNING: Low keypoint count ({num_keypoints}) - move camera more!")
        elif num_keypoints > 200:
            print(f"✓ Healthy keypoint count ({num_keypoints})")

        if num_candidates < 100:
            print(f"⚠️  WARNING: Low candidate count ({num_candidates})")

        print(f"{'='*60}")

        # Update visualization
        self.kp_counts.append(num_keypoints)
        self.cand_counts.append(num_candidates)

        if self.show_viz:
            self._update_viz(image_curr)

    def _update_viz(self, image: np.ndarray):
        """Update matplotlib visualization"""
        # Image with keypoints
        self.ax_img.clear()
        self.ax_img.set_title(f"Frame {self.state.i}, {self.state.P.shape[1]} keypoints")
        self.ax_img.imshow(image, cmap="gray")
        if self.state.P.shape[1] > 0:
            self.ax_img.scatter(self.state.P[0, :], self.state.P[1, :], s=6, c="yellow", marker="o")
        self.ax_img.axis("off")

        # Counts
        self.ax_counts.clear()
        self.ax_counts.set_title("Keypoints / Inliers")
        self.ax_counts.set_xlabel("Frame")
        self.ax_counts.set_ylabel("Count")
        self.ax_counts.plot(self.kp_counts, color="blue", label="Keypoints")
        self.ax_counts.plot(self.cand_counts, color="green", label="Candidates")
        self.ax_counts.legend(loc="upper right")

        # Trajectory and landmarks
        self.ax_traj.clear()
        self.ax_traj.set_title("Local Trajectory (300 poses) and Landmarks")
        self.ax_traj.set_xlabel("X")
        self.ax_traj.set_ylabel("Z")

        if len(self.trajectory) > 1:
            xs = [p[0] for p in self.trajectory[-300:]]
            zs = [p[1] for p in self.trajectory[-300:]]
            self.ax_traj.plot(xs, zs, color="red", linewidth=2, label="Camera Trajectory")

            curr_x, curr_z = xs[-1], zs[-1]
            self.ax_traj.scatter(curr_x, curr_z, c="red", s=50, marker="o", zorder=5)

            # Plot landmarks
            if self.state.X.shape[1] > 0:
                landmarks = self.state.X
                dists = np.linalg.norm(landmarks[:2, :] - np.array([[curr_x], [curr_z]]), axis=0)
                close_mask = dists < 50
                close_landmarks = landmarks[:, close_mask]

                if close_landmarks.shape[1] > 0:
                    self.ax_traj.scatter(
                        close_landmarks[0, :], close_landmarks[2, :],
                        c="blue", s=10, alpha=0.6, label="Landmarks"
                    )

            radius = 10.0
            self.ax_traj.set_xlim(curr_x - radius, curr_x + radius)
            self.ax_traj.set_ylim(curr_z - radius, curr_z + radius)
            self.ax_traj.legend(loc="upper left")

        self.ax_traj.grid(True, linestyle="--", alpha=0.3)
        self.ax_traj.set_aspect("equal")

        plt.pause(0.001)


def main():
    # Parse environment variables
    video_source = os.environ.get("VIDEO_SOURCE", "0")
    if video_source.isdigit():
        video_source = int(video_source)

    focal = float(os.environ.get("FOCAL", "800"))

    # Open video capture
    cap = cv2.VideoCapture(video_source)
    if not cap.isOpened():
        print(f"Cannot open video source: {video_source}")
        sys.exit(1)

    ret, frame = cap.read()
    if not ret:
        print("Cannot read initial frame")
        sys.exit(1)

    h, w = frame.shape[:2]
    cx = float(os.environ.get("CX", str(w / 2)))
    cy = float(os.environ.get("CY", str(h / 2)))

    K = np.array([[focal, 0, cx], [0, focal, cy], [0, 0, 1]], dtype=np.float64)

    # Initialize VO
    vo = VisualOdometry(K, show_viz=True)

    print(f"Visual Odometry initialized")
    print(f"Camera intrinsics:\n{K}")
    print(f"Collecting {vo.K_BOOTSTRAP} frames for bootstrapping...")

    # Collect bootstrap images
    bootstrap_images = []
    while len(bootstrap_images) < vo.K_BOOTSTRAP:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        bootstrap_images.append(gray)
        cv2.imshow("Collecting bootstrap frames", frame)
        cv2.waitKey(1)

    cv2.destroyWindow("Collecting bootstrap frames")

    if len(bootstrap_images) < vo.K_BOOTSTRAP:
        print("Not enough bootstrap frames")
        sys.exit(1)

    # Bootstrap
    if not vo.bootstrap(bootstrap_images):
        print("Bootstrap failed")
        sys.exit(1)

    # Continuous operation
    print("\nStarting continuous operation. Press 'q' to quit.")
    ret, frame_prev = cap.read()
    if ret:
        gray_prev = cv2.cvtColor(frame_prev, cv2.COLOR_BGR2GRAY)

    while True:
        ret, frame_curr = cap.read()
        if not ret:
            print("Stream ended")
            break

        gray_curr = cv2.cvtColor(frame_curr, cv2.COLOR_BGR2GRAY)

        try:
            vo.process_frame(gray_curr, gray_prev)
        except Exception as e:
            print(f"Error processing frame: {e}")
            import traceback
            traceback.print_exc()

        gray_prev = gray_curr

        cv2.imshow("Visual Odometry", frame_curr)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    plt.show(block=True)


if __name__ == "__main__":
    main()
