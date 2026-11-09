"""main_agi_fast 的 KNN + 体素加权 DBSCAN；所有距离统一为米。"""
from __future__ import annotations

from dataclasses import dataclass
import time
import numpy as np


@dataclass(frozen=True)
class CloudFilter:
    mode: str = 'fast_voxel'
    voxel_size_m: float = 0.01
    dbscan_eps_m: float = 0.15
    min_samples: int = 60
    neighbors: int = 10
    std_ratio: float = 5.0
    min_cluster_ratio: float = 0.10
    fast_fallback: bool = True

    def __post_init__(self):
        if self.mode not in ('fast_voxel', 'legacy_dbscan'):
            raise ValueError('点云过滤 mode 必须为 fast_voxel 或 legacy_dbscan')
        for key in ('voxel_size_m', 'dbscan_eps_m', 'std_ratio'):
            if not np.isfinite(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError(f'{key} 必须为正的有限数')
        for key in ('min_samples', 'neighbors'):
            value = getattr(self, key)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f'{key} 必须为正整数')
        if not 0 <= self.min_cluster_ratio <= 1:
            raise ValueError('min_cluster_ratio 必须在 [0, 1] 内')
        if not isinstance(self.fast_fallback, bool):
            raise ValueError('fast_fallback 必须为布尔值')


def _largest_cluster(points, config, weighted=False):
    from sklearn.cluster import DBSCAN
    if weighted:
        indices = np.floor(points / config.voxel_size_m).astype(np.int64)
        _, inverse, counts = np.unique(indices, axis=0, return_inverse=True, return_counts=True)
        sums = np.zeros((len(counts), 3), dtype=np.float32)
        np.add.at(sums, inverse, points)
        centers = sums / counts[:, None]
        if len(centers) == 1:
            return np.ones(len(points), dtype=bool), len(centers)
        labels = DBSCAN(eps=config.dbscan_eps_m, min_samples=config.min_samples,
                        n_jobs=-1).fit(centers, sample_weight=counts).labels_
        valid = labels >= 0
        if not valid.any():
            raise ValueError('体素加权 DBSCAN 未找到有效聚类')
        best = np.argmax(np.bincount(labels[valid], weights=counts[valid]))
        return labels[inverse] == best, len(centers)
    labels = DBSCAN(eps=config.dbscan_eps_m, min_samples=config.min_samples).fit(points).labels_
    valid = labels >= 0
    # 与参考脚本一致：普通 DBSCAN 全是噪点时保留 KNN 结果。
    if not valid.any():
        return np.ones(len(points), dtype=bool), None
    return labels == np.argmax(np.bincount(labels[valid])), None


def filter_point_cloud(points, config=None):
    """返回原点数组中的保留下标，保证相机点与机器人点一一对应。"""
    started = time.perf_counter()
    config = CloudFilter(**dict(config or {}))
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('点云必须是 N×3')
    keep = np.flatnonzero(np.isfinite(points).all(axis=1))
    cloud = np.ascontiguousarray(points[keep], dtype=np.float32)
    if not len(cloud):
        raise ValueError('点云不包含有效有限值')
    stats = {'configured_mode': config.mode, 'input_point_count': len(points),
             'finite_point_count': len(cloud), 'mode_used': config.mode}
    # N<=k 时不能请求 k+1 个邻居；少量点直接保留。
    if len(cloud) > config.neighbors:
        from sklearn.neighbors import NearestNeighbors
        distances, _ = NearestNeighbors(n_neighbors=config.neighbors + 1).fit(cloud).kneighbors(cloud)
        mean = distances[:, 1:].mean(axis=1)
        selected = mean <= mean.mean() + config.std_ratio * mean.std()
        keep, cloud = keep[selected], cloud[selected]
    if not len(cloud):
        raise ValueError('KNN 过滤后点云为空')
    stats['knn_point_count'] = len(cloud)
    if config.mode == 'fast_voxel':
        try:
            selected, voxels = _largest_cluster(cloud, config, weighted=True)
            ratio = float(selected.mean())
            if ratio < config.min_cluster_ratio:
                raise ValueError(f'最大簇占比 {ratio:.3f} 小于 {config.min_cluster_ratio}')
            stats.update(voxel_count=voxels, selected_cluster_ratio=ratio)
        except (ValueError, RuntimeError) as exc:
            if not config.fast_fallback:
                raise
            selected, _ = _largest_cluster(cloud, config)
            stats.update(mode_used='legacy_dbscan_fallback', fallback_reason=str(exc))
    else:
        selected, _ = _largest_cluster(cloud, config)
    keep = keep[selected]
    stats.update(output_point_count=len(keep), elapsed_ms=(time.perf_counter() - started) * 1000)
    return keep, stats
