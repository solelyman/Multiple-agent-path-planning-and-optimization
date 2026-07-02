import numpy as np


def path_length(path_msg):
    poses = path_msg.poses
    length = 0.0
    for i in range(1, len(poses)):
        p1 = poses[i - 1].pose.position
        p2 = poses[i].pose.position
        length += np.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)
    return length


def sample_path_xy(path_msg, n_samples):
    poses = path_msg.poses
    if not poses:
        return np.zeros((0, 2), dtype=float)

    n_samples = max(2, int(n_samples))
    points = np.array([[p.pose.position.x, p.pose.position.y] for p in poses], dtype=float)
    if len(points) == 1:
        return np.repeat(points, n_samples, axis=0)

    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(distances)))
    total = cumulative[-1]
    if total < 1e-6:
        return np.repeat(points[:1], n_samples, axis=0)

    samples = np.linspace(0.0, total, n_samples)
    sampled = np.zeros((n_samples, 2), dtype=float)
    for i, s in enumerate(samples):
        idx = np.searchsorted(cumulative, s, side='right') - 1
        idx = min(max(idx, 0), len(points) - 2)
        segment_len = max(cumulative[idx + 1] - cumulative[idx], 1e-6)
        ratio = (s - cumulative[idx]) / segment_len
        sampled[i] = (1.0 - ratio) * points[idx] + ratio * points[idx + 1]
    return sampled
