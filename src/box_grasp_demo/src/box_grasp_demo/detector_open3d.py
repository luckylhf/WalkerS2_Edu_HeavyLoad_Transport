"""Open3D-based upright box detector.

This module keeps the same public API as :mod:`detector` but implements the
geometry pipeline with Open3D primitives:

* ``PointCloud.segment_plane`` finds the horizontal support surface.
* ``PointCloud.cluster_dbscan`` groups above-surface points by their XY
  footprint, so a hollow open carton (rim + walls) stays one cluster even when
  a one-voxel sensor gap would split it under strict 3-D connectivity.
* ``PointCloud.get_minimal_oriented_bounding_box`` recovers the carton yaw and
  extents, which are then matched against the known box dimensions.

The intended input is an ``N x 3`` cloud in a frame whose +Z axis points
upwards (normally ``base_link`` after TF).
"""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import open3d as o3d
except Exception as exc:  # pragma: no cover - exercised only on import
    o3d = None
    _OPEN3D_IMPORT_ERROR = exc


@dataclass
class DetectorConfig:
    # Known carton size.  The detector matches a cluster against these values.
    box_length: float = 0.40
    box_width: float = 0.30
    box_height: float = 0.20
    dimension_tolerance: float = 0.05

    # Support-plane fitting and object extraction.
    plane_distance: float = 0.015
    plane_tilt_deg: float = 25.0
    plane_iterations: int = 200
    min_plane_inliers: int = 80
    max_object_height: float = 0.50

    # Open3D clustering parameters.  ``dbscan_min_points`` controls core-point
    # density; the resulting clusters are then filtered by
    # ``min_component_points``, so a sparse carton is not discarded as noise.
    min_component_points: int = 20
    dbscan_eps: float = 0.035
    dbscan_min_points: int = 5

    # Carton pose constraints.
    upright_box: bool = True
    max_box_tilt_deg: float = 8.0
    top_outlier_margin: float = 0.015
    measure_actual_height: bool = False

    # Grasp API.
    side_clearance: float = 0.018
    pregrasp_distance: float = 0.05
    tool_contact_below_top: float = 0.05
    grasp_long_edge: bool = False

    # Height prior used to select the table plane.
    support_height_min: Optional[float] = None
    support_height_max: Optional[float] = None


@dataclass
class SupportSurface:
    center: np.ndarray
    rotation: np.ndarray
    dimensions: np.ndarray
    support_plane: np.ndarray
    point_count: int


@dataclass
class BoxDetection:
    """Detected box expressed in the requested output frame."""

    center: np.ndarray
    # Columns are long horizontal axis, short horizontal axis, and up axis.
    rotation: np.ndarray
    dimensions: np.ndarray
    support_plane: np.ndarray  # n.x + c = 0, n points upwards
    score: float
    point_count: int

    @property
    def normal(self) -> np.ndarray:
        return self.rotation[:, 2]


class BoxDetector:
    """Known-size upright-box detector implemented with Open3D."""

    def __init__(self, config: Optional[DetectorConfig] = None, logger=None):
        if o3d is None:
            raise ImportError(
                "open3d is required for detector_open3d; "
                f"import failed with: {_OPEN3D_IMPORT_ERROR}"
            )
        self.config = config or DetectorConfig()
        self._logger = logger

    def _log(self, msg: str):
        if self._logger:
            self._logger.info(msg)

    def detect(self, points: np.ndarray) -> Optional[BoxDetection]:
        """Detect one known-size box from an Nx3 point cloud."""
        cfg = self.config
        p = np.asarray(points, dtype=np.float64)
        if p.ndim != 2 or p.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        p = p[np.all(np.isfinite(p), axis=1)]
        if len(p) < cfg.min_plane_inliers + cfg.min_component_points:
            return None

        plane = self._fit_support_plane(p)
        if plane is None:
            return None
        normal, offset = plane

        signed = p @ normal + offset
        object_points = p[(signed > cfg.plane_distance) &
                          (signed < cfg.max_object_height)]
        if len(object_points) < cfg.min_component_points:
            return None

        best = None
        for cluster in self._cluster_by_footprint(object_points):
            candidate = self._score_cluster(cluster, normal, offset)
            if candidate is not None and (best is None or candidate.score < best.score):
                best = candidate
        return best

    def detect_support_surface(self, points: np.ndarray,
                               roi_min: Optional[np.ndarray] = None,
                               roi_max: Optional[np.ndarray] = None) -> Optional[SupportSurface]:
        """Fit a table/platform plane, optionally inside a 3-D ROI."""
        cfg = self.config
        p = np.asarray(points, dtype=np.float64)
        p = p[np.all(np.isfinite(p), axis=1)]
        if roi_min is not None:
            p = p[np.all(p >= np.asarray(roi_min), axis=1)]
        if roi_max is not None:
            p = p[np.all(p <= np.asarray(roi_max), axis=1)]
        if len(p) < cfg.min_plane_inliers:
            return None

        plane = self._fit_support_plane(p)
        if plane is None:
            return None
        normal, offset = plane
        inliers = p[np.abs(p @ normal + offset) < cfg.plane_distance]
        if len(inliers) < cfg.min_plane_inliers:
            return None

        plane_point = -offset * normal
        rel = inliers - plane_point
        horizontal = rel - np.outer(rel @ normal, normal)
        _, vectors = np.linalg.eigh(horizontal.T @ horizontal)
        axis_long = vectors[:, -1]
        axis_long -= normal * np.dot(axis_long, normal)
        axis_long /= max(np.linalg.norm(axis_long), 1e-9)
        axis_short = np.cross(normal, axis_long)
        axis_short /= max(np.linalg.norm(axis_short), 1e-9)
        if axis_short[1] < 0 or (abs(axis_short[1]) < 1e-6 and axis_short[0] < 0):
            axis_long, axis_short = -axis_long, -axis_short
        rotation = np.column_stack((axis_long, axis_short, normal))
        coordinates = rel @ rotation
        bounds_min, bounds_max = coordinates.min(axis=0), coordinates.max(axis=0)
        center = plane_point + rotation @ np.array(
            [(bounds_min[0] + bounds_max[0]) / 2.0,
             (bounds_min[1] + bounds_max[1]) / 2.0, 0.0])
        return SupportSurface(center, rotation, np.r_[np.ptp(coordinates[:, :2]), 0.0],
                              np.r_[normal, offset], len(inliers))

    def _fit_support_plane(self, p: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
        """Fit the dominant horizontal support plane with Open3D RANSAC."""
        cfg = self.config
        if len(p) < 3:
            return None

        fit_points = p
        if cfg.upright_box and (cfg.support_height_min is not None or
                                cfg.support_height_max is not None):
            mask = np.ones(len(p), dtype=bool)
            if cfg.support_height_min is not None:
                mask &= p[:, 2] >= cfg.support_height_min
            if cfg.support_height_max is not None:
                mask &= p[:, 2] <= cfg.support_height_max
            fit_points = p[mask]
            if len(fit_points) < 3:
                fit_points = p

        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(fit_points))
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=cfg.plane_distance,
            ransac_n=3,
            num_iterations=cfg.plane_iterations,
        )
        normal = np.asarray(plane_model[:3], dtype=np.float64)
        offset = float(plane_model[3])
        if np.dot(normal, np.array([0.0, 0.0, 1.0])) < 0.0:
            normal = -normal
            offset = -offset
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            return None
        normal /= norm
        offset /= norm

        z_axis = np.array([0.0, 0.0, 1.0])
        if np.dot(normal, z_axis) < np.cos(np.deg2rad(cfg.plane_tilt_deg)):
            return None
        inlier_indices = np.asarray(inliers, dtype=np.int64)
        if len(inlier_indices) < cfg.min_plane_inliers:
            return None

        inlier_points = fit_points[inlier_indices]
        median_z = float(np.median(inlier_points[:, 2]))
        if cfg.support_height_min is not None and median_z < cfg.support_height_min - 0.03:
            return None
        if cfg.support_height_max is not None and median_z > cfg.support_height_max + 0.03:
            return None

        # Refine the normal from all inliers (PCA) for a less noisy offset.
        mean = inlier_points.mean(axis=0)
        _, _, vh = np.linalg.svd(inlier_points - mean, full_matrices=False)
        refined = vh[-1]
        if np.dot(refined, z_axis) < 0.0:
            refined = -refined
        refined /= max(np.linalg.norm(refined), 1e-9)
        refined_offset = -float(np.dot(refined, mean))
        # 平面方程 refined·x + refined_offset = 0，normal 朝上时
        # 桌面在 base_link 系的高度 z ≈ -refined_offset / refined_z。
        table_z = -refined_offset / float(refined[2]) if abs(refined[2]) > 1e-6 else -refined_offset
        self._log(
            f"Open3D 支撑平面: normal={np.round(refined, 3)}, "
            f"桌面Z(base_link系)={table_z:.3f}m, "
            f"offset={refined_offset:.3f}")
        return refined, refined_offset

    def _cluster_by_footprint(self, points: np.ndarray):
        """Cluster above-surface points by XY footprint using DBSCAN.

        Projecting to the ground plane means a hollow carton's rim, walls and
        interior contents all fall in the same footprint blob, while genuinely
        separate objects stay apart.
        """
        cfg = self.config
        xy = points[:, :2]
        pcd = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(np.c_[xy, np.zeros(len(xy))]))
        labels = np.asarray(
            pcd.cluster_dbscan(eps=cfg.dbscan_eps,
                               min_points=cfg.dbscan_min_points,
                               print_progress=False),
            dtype=np.int64,
        )
        clusters = []
        for label in np.unique(labels):
            if label < 0:
                continue
            cluster = points[labels == label]
            if len(cluster) >= cfg.min_component_points:
                clusters.append(cluster)
        return clusters

    def _estimate_tilted_up(self, points, axis_long, center_xy):
        """upright_box 下估计箱体竖轴（允许小倾角，超限判失败）。

        在长/短轴两侧边带各取顶部高分位点，用两侧顶面高度差分估计
        顶面沿两轴方向的斜率，构造竖轴（旧版 detector.py 同款算法）。
        倾角超过 max_box_tilt_deg 时返回 None（该聚类判定失败，不截断）。
        max_box_tilt_deg <= 0 时直接返回竖直 Z 轴。
        """
        cfg = self.config
        if cfg.max_box_tilt_deg <= 0.0:
            return np.array([0.0, 0.0, 1.0])

        # 初始短轴（水平，右手系）仅用于边带划分。
        axis_long = axis_long - np.array([0.0, 0.0, 1.0]) * axis_long[2]
        axis_long /= max(np.linalg.norm(axis_long), 1e-9)
        axis_short = np.cross(np.array([0.0, 0.0, 1.0]), axis_long)

        band = 0.03  # 边带半宽（m）
        u_i = points[:, :2] @ axis_long[:2]
        v_i = points[:, :2] @ axis_short[:2]
        cu = float(center_xy @ axis_long[:2])
        cv = float(center_xy @ axis_short[:2])

        a = b = None
        # 长轴 ± 边带，顶部 30% 分位差分估计长轴方向顶面斜率 b。
        u_neg = points[np.abs(u_i - (cu - cfg.box_length / 2.0)) <= band]
        u_pos = points[np.abs(u_i - (cu + cfg.box_length / 2.0)) <= band]
        if len(u_neg) >= 3 and len(u_pos) >= 3:
            u_neg = u_neg[np.argsort(u_neg[:, 2])][-max(3, len(u_neg) * 3 // 10):]
            u_pos = u_pos[np.argsort(u_pos[:, 2])][-max(3, len(u_pos) * 3 // 10):]
            u_neg_c = u_neg[:, :2] @ axis_long[:2]
            u_pos_c = u_pos[:, :2] @ axis_long[:2]
            denom = float(u_pos_c.mean() - u_neg_c.mean())
            if abs(denom) > 0.05:
                b = float((u_pos[:, 2].mean() - u_neg[:, 2].mean()) / denom)
        # 短轴 ± 边带，顶部 10% 分位差分估计短轴方向顶面斜率 a。
        v_neg = points[np.abs(v_i - (cv - cfg.box_width / 2.0)) <= band]
        v_pos = points[np.abs(v_i - (cv + cfg.box_width / 2.0)) <= band]
        if len(v_neg) >= 3 and len(v_pos) >= 3:
            v_neg = v_neg[np.argsort(v_neg[:, 2])][-max(3, len(v_neg) // 10):]
            v_pos = v_pos[np.argsort(v_pos[:, 2])][-max(3, len(v_pos) // 10):]
            v_neg_c = v_neg[:, :2] @ axis_short[:2]
            v_pos_c = v_pos[:, :2] @ axis_short[:2]
            denom = float(v_pos_c.mean() - v_neg_c.mean())
            if abs(denom) > 0.05:
                a = float((v_pos[:, 2].mean() - v_neg[:, 2].mean()) / denom)

        if a is None and b is None:
            return np.array([0.0, 0.0, 1.0])
        if a is None:
            a = 0.0
        if b is None:
            b = 0.0
        # u/v 轴在水平面内旋转，斜率投影回世界 XY 构造顶面法向。
        g = b * axis_long[:2] + a * axis_short[:2]
        n_top = np.array([-g[0], -g[1], 1.0])
        n_top /= max(np.linalg.norm(n_top), 1e-9)
        tilt = float(np.arccos(np.clip(n_top[2], -1.0, 1.0)))
        max_tilt = np.deg2rad(cfg.max_box_tilt_deg)
        if tilt > max_tilt:
            self._log(
                f"箱子倾斜 {np.rad2deg(tilt):.1f}° 超过允许值 "
                f"{cfg.max_box_tilt_deg:.1f}°，该聚类判定失败")
            return None
        return n_top

    def _score_cluster(self, points: np.ndarray, normal: np.ndarray, offset: float):
        cfg = self.config
        expected = np.array([cfg.box_length, cfg.box_width, cfg.box_height])

        # Remove contents protruding through an open top before fitting the box.
        support_z = -(normal[0] * points[:, 0] +
                      normal[1] * points[:, 1] + offset) / normal[2]
        heights = points[:, 2] - support_z
        points = points[heights <= cfg.box_height + cfg.top_outlier_margin]
        if len(points) < cfg.min_component_points:
            return None

        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        try:
            # The minimal-volume OBB is fitted to the convex hull, so it is
            # insensitive to unequal sampling between faces.  PCA OBB
            # (``get_oriented_bounding_box``) is biased by dense top/side
            # faces and inflates the footprint, so it is only a fallback.
            obb = pcd.get_minimal_oriented_bounding_box()
        except Exception:
            try:
                obb = pcd.get_oriented_bounding_box()
            except Exception:
                return None
        extent = np.asarray(obb.extent, dtype=np.float64)
        rotation = np.asarray(obb.R, dtype=np.float64)

        # Reorder the OBB axes into [long horizontal, short horizontal, up].
        up_similarity = np.abs(rotation.T @ normal)
        i_up = int(np.argmax(up_similarity))
        horizontal = [i for i in range(3) if i != i_up]
        i_long = horizontal[int(np.argmax(extent[horizontal]))]
        i_short = horizontal[int(np.argmin(extent[horizontal]))]

        footprint_error = np.array([
            abs(extent[i_long] - cfg.box_length) / cfg.box_length,
            abs(extent[i_short] - cfg.box_width) / cfg.box_width,
        ])
        if np.any(footprint_error > cfg.dimension_tolerance):
            return None

        center_xy = np.asarray(obb.center[:2], dtype=np.float64)

        axis_long = rotation[:, i_long].copy()
        if cfg.upright_box:
            # 箱子平放于桌面：Rx=Ry≈0。max_box_tilt_deg > 0 时允许顶面
            # 小倾角：由两侧边带顶部高分位差分估计竖轴；超过允许角度
            # 直接判定该聚类失败（不截断、不执行）。
            axis_up = self._estimate_tilted_up(points, axis_long, center_xy)
            if axis_up is None:
                return None
        else:
            axis_up = rotation[:, i_up].copy()
            if np.dot(axis_up, normal) < 0.0:
                axis_up = -axis_up

        # Deterministic left/right assignment in the robot frame: short axis
        # points toward +X (forward), long axis completes the right-handed set.
        axis_long -= axis_up * np.dot(axis_long, axis_up)
        axis_long /= max(np.linalg.norm(axis_long), 1e-9)
        axis_short = np.cross(axis_up, axis_long)
        axis_short /= max(np.linalg.norm(axis_short), 1e-9)
        if axis_short[0] < 0 or (abs(axis_short[0]) < 1e-6 and axis_short[1] < 0):
            axis_short = -axis_short
            axis_long = -axis_long
        if np.dot(np.cross(axis_long, axis_short), axis_up) < 0.0:
            axis_short = -axis_short
        rotation_matrix = np.column_stack((axis_long, axis_short, axis_up))

        support_at_center = -(normal[0] * center_xy[0] +
                              normal[1] * center_xy[1] + offset) / normal[2]

        if cfg.measure_actual_height:
            measured_height = float(extent[i_up])
            height = max(measured_height, 0.02)
            dimensions = np.array([cfg.box_length, cfg.box_width, height])
        else:
            height = cfg.box_height
            dimensions = expected.copy()
        center = np.array([center_xy[0], center_xy[1], support_at_center + height / 2.0])
        support_plane = np.array([0.0, 0.0, 1.0, -support_at_center])
        score = float(np.mean(footprint_error)) - min(len(points), 3000) / 300000.0
        return BoxDetection(center, rotation_matrix, dimensions, support_plane,
                            score, len(points))

    def grasp_centers(self, detection: BoxDetection):
        """Return left/right grasp and pregrasp points (same API as detector)."""
        cfg = self.config
        u, v, n = detection.rotation.T
        contact_height = detection.dimensions[2] / 2.0 - cfg.tool_contact_below_top

        if cfg.grasp_long_edge:
            half_width = detection.dimensions[1] / 2.0 + cfg.side_clearance
            plus = detection.center + v * half_width + n * contact_height
            minus = detection.center - v * half_width + n * contact_height
            if v[1] >= 0:
                left_grasp, right_grasp = plus, minus
                left_inward, right_inward = -v, v
            else:
                left_grasp, right_grasp = minus, plus
                left_inward, right_inward = v, -v
        else:
            half_length = detection.dimensions[0] / 2.0 + cfg.side_clearance
            plus = detection.center + u * half_length + n * contact_height
            minus = detection.center - u * half_length + n * contact_height
            if u[1] >= 0:
                left_grasp, right_grasp = plus, minus
                left_inward, right_inward = -u, u
            else:
                left_grasp, right_grasp = minus, plus
                left_inward, right_inward = u, -u
        left_pre = left_grasp - left_inward * cfg.pregrasp_distance
        right_pre = right_grasp - right_inward * cfg.pregrasp_distance
        return {
            "left": (left_grasp, left_inward, left_pre),
            "right": (right_grasp, right_inward, right_pre),
        }
