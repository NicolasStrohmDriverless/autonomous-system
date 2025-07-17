import numpy as np
from enum import Enum
import math

class Mode(Enum):
    """ 
    Possible modes for how Voronoi regions are selected.
    
    1. Expand:
        Find closest nodes around starting node.
        Results in roundish track shapes.
    
    2. Extend:
        Find nodes closest to line extending from starting node.
        Results in elongated track shapes.
        
    3. Random:
        Select all regions randomly.
        Results in large track shapes.
    """
    EXPAND = 1
    EXTEND = 2
    RANDOM = 3

class SimType(Enum):
    """ Selection between output format for different simulators.

    1. FSSIM:
        Output FSSIM compatible .yaml file.
    2. FSDS:
        Output FSDS compatible .csv file 
    """
    FSSIM = 1
    FSDS = 2
    GPX = 3 

def closest_node(node, nodes, k):
    """
    Returns the index of the k-th closest node.
    
    Args:
        node (numpy.ndarray): Node to find k-th closest node to.
        nodes (numpy.ndarray): Available nodes.
        k (int): Number which determines which closest node to return.
    
    Returns:
        int: Index of k-th closest node.
    """
    deltas = nodes - node
    distance = np.einsum('ij,ij->i', deltas, deltas)
    return np.argpartition(distance, k)[k]

def clockwise_sort(p):
    """
    Sorts nodes in clockwise order.
    
    Args:
        p (numpy.ndarray): Points to sort.
    
    Returns:
        numpy.ndarray: Clockwise sorted points.
    """
    d = p - np.mean(p, axis=0)
    s = np.arctan2(d[:,0], d[:,1])
    return p[np.argsort(s),:]

def curvature(dx_dt, d2x_dt2, dy_dt, d2y_dt2):
    """
    Calculates the curvature along a line.
    
    Args:
        dx_dt (numpy.ndarray): First derivative of x.
        d2x_dt2 (numpy.ndarray): Second derivative of x.
        dy_dt (numpy.ndarray): First derivative of y.
        d2y_dt2 (numpy.ndarray): Second derivative of y.
    
    Returns:
        np.ndarray: Curvature along line.
    """
    return (dx_dt**2 + dy_dt**2)**-1.5 * (dx_dt * d2y_dt2 - dy_dt * d2x_dt2)

def arc_length(x, y, R):
    """
    Calculates the arc length between to points based on the radius of curvature of the path segment.
    
    Args:
        x (numpy.ndarray): X-coordinates.
        y (numpy.ndarray): Y-coordinates.
        R (numpy.ndarray): Radius of curvature of track segment in meters.
    Returns:
        (float): Arc length in meters.
    """
    x0, x1 = x[:-1], x[1:]
    y0, y1 = y[:-1], y[1:]   
    R = R[:-1]
    
    distance = np.sqrt((x1 - x0)**2 + (y1 - y0)**2)
    theta = 2 * np.arcsin(0.5 * distance / R)
    arc_length = R * theta
    return arc_length

def transformation_matrix(displacement, angle):
    """
    Translate, then rotate around origin.
    
    Args:
        displacement (tuple): Distance to translate along both axes.
        angle (float): Angle in radians to rotate.
    
    Returns:
        numpy.ndarray: 3x3 transformation matrix.
    """
    h, k = displacement
    c, s = np.cos(angle), np.sin(angle)
    
    M = np.array([
        [c,    -s,      h * c - k * s],
        [s,     c,      h * s + k * c],
        [0,     0,            1      ]
    ])
    return M

import yaml
from shapely.geometry import LineString


def load_yaml_track(path: str, num_center_points: int = 400):
    """Load a track yaml file and align it to the time keeping gate.

    Parameters
    ----------
    path: str
        Path to the track yaml file.
    num_center_points: int
        Number of points to sample for the generated centerline.

    Returns
    -------
    tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]
        Arrays for left cones, right cones (x, y, color), centerline (x, y)
        and the time keeping cones (x, y, color).
    """
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)["track"]

    def _parse(key: str):
        cones = []
        for item in data.get(key, []):
            pos = item.get("position", [0.0, 0.0, 0.0])
            cls = item.get("class", "unknown")
            cones.append([float(pos[0]), float(pos[1]), cls])
        return np.array(cones, dtype=object)

    cones_left = _parse("left")
    cones_right = _parse("right")
    cones_time = _parse("time_keeping")

    if len(cones_time) >= 2:
        gate_center = (
            cones_time[0, :2].astype(float) + cones_time[1, :2].astype(float)
        ) / 2.0
        gate_vec = (
            cones_time[1, :2].astype(float) - cones_time[0, :2].astype(float)
        )
        track_angle = math.atan2(gate_vec[1], gate_vec[0]) + math.pi / 2
    else:
        gate_center = np.zeros(2)
        track_angle = 0.0

    c, s = math.cos(-track_angle), math.sin(-track_angle)

    def _transform(arr: np.ndarray) -> np.ndarray:
        if arr.size == 0:
            return arr
        xy = arr[:, :2].astype(float) - gate_center
        arr[:, 0] = xy[:, 0] * c - xy[:, 1] * s
        arr[:, 1] = xy[:, 0] * s + xy[:, 1] * c
        return arr

    cones_left = _transform(cones_left)
    cones_right = _transform(cones_right)
    cones_time = _transform(cones_time)

    line_left = (
        LineString(cones_left[:, :2].astype(float)) if len(cones_left) > 1 else None
    )
    line_right = (
        LineString(cones_right[:, :2].astype(float)) if len(cones_right) > 1 else None
    )

    center = []
    for i in range(num_center_points):
        t = i / (num_center_points - 1)
        if line_left is not None:
            p_l = line_left.interpolate(t * line_left.length)
        else:
            p_l = None
        if line_right is not None:
            p_r = line_right.interpolate(t * line_right.length)
        else:
            p_r = None
        if p_l and p_r:
            center.append([(p_l.x + p_r.x) / 2.0, (p_l.y + p_r.y) / 2.0])
        elif p_l:
            center.append([p_l.x, p_l.y])
        elif p_r:
            center.append([p_r.x, p_r.y])
    centerline = np.array(center, dtype=float)

    if len(centerline):
        idx = np.argmin(np.linalg.norm(centerline[:, :2] - [0.0, 0.0], axis=1))
        centerline = np.vstack([centerline[idx:], centerline[:idx]])

    return cones_left, cones_right, centerline, cones_time
