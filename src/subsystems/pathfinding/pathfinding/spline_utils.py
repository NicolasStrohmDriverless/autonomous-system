import numpy as np
from scipy.interpolate import splprep, splev


def spline_path(path, samples=None, smoothing=0.0):
    """Return B-spline interpolation of ``path`` with ``samples`` points."""
    if len(path) < 3:
        return path
    arr = np.array(path).T
    k = min(3, len(path) - 1)
    tck, _ = splprep(arr, s=smoothing, k=k)
    if samples is None:
        samples = len(path)
    u_new = np.linspace(0, 1, samples)
    out = splev(u_new, tck)
    return list(zip(out[0], out[1]))
