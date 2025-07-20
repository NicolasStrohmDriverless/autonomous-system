import numpy as np
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from pathfinding.spline_utils import spline_path

def test_spline_path_basic():
    path = [(0, 0), (1, 0), (2, 2), (3, 3)]
    result = spline_path(path, samples=8)
    assert isinstance(result, list)
    assert len(result) == 8
    assert np.allclose(result[0], path[0])
    assert np.allclose(result[-1], path[-1])
