"""Latest TF graph, accumulated across messages; no historical interpolation."""
from __future__ import annotations

from collections import deque
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation


class TransformBuffer:
    def __init__(self):
        self._edges = {}
        self._lock = threading.RLock()

    def clear(self):
        with self._lock:
            self._edges.clear()

    def update(self, message, *, static=False):
        now = time.monotonic()
        for item in message.get('transforms', []):
            child = str(item.get('child_frame_id', '')).strip('/')
            header = item.get('header') or {}
            parent = str(header.get('frame_id', '')).strip('/')
            if not child or not parent or child == parent:
                continue
            try:
                value = item['transform']
                xyz = [float(value['translation'][k]) for k in 'xyz']
                quat = [float(value['rotation'][k]) for k in 'xyzw']
                if not np.isfinite(xyz + quat).all() or np.linalg.norm(quat) < 1e-8:
                    raise ValueError('Invalid TF')
                matrix = np.eye(4)
                matrix[:3, :3] = Rotation.from_quat(quat).as_matrix()
                matrix[:3, 3] = xyz
            except (KeyError, TypeError, ValueError):
                with self._lock:
                    self._edges.pop(child, None)
                continue
            stamp = header.get('stamp') or {}
            stamp_ns = int(stamp.get('sec', stamp.get('secs', 0))) * 10**9 + int(stamp.get('nanosec', stamp.get('nsecs', 0)))
            with self._lock:
                old = self._edges.get(child)
                # Replayed dynamic samples must not become fresh just by repetition.
                if old and not static and not old[3] and stamp_ns and stamp_ns <= old[4]:
                    continue
                self._edges[child] = (parent, matrix, now, static, stamp_ns)

    def lookup(self, source, target, max_age_s=None):
        source, target = source.strip('/'), target.strip('/')
        if not source or not target:
            raise ValueError('TF 坐标系不能为空')
        if source == target:
            return np.eye(4)
        now = time.monotonic()
        with self._lock:
            edges = dict(self._edges)
        graph = {}
        for child, (parent, matrix, received, static, _) in edges.items():
            if not static and max_age_s is not None and now - received > max_age_s:
                continue
            graph.setdefault(child, []).append((parent, matrix))
            graph.setdefault(parent, []).append((child, np.linalg.inv(matrix)))
        queue = deque([(source, np.eye(4))])
        visited = {source}
        while queue:
            frame, accumulated = queue.popleft()
            for neighbor, transform in graph.get(frame, []):
                if neighbor in visited:
                    continue
                result = transform @ accumulated
                if neighbor == target:
                    return result
                visited.add(neighbor)
                queue.append((neighbor, result))
        raise LookupError(f'TF 不连通或动态变换已过期: {source} → {target}')
