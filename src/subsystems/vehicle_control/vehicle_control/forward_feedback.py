import math
from typing import Tuple


def kinematic_step_deg(
    x: float,
    y: float,
    yaw_deg: float,
    speed: float,
    steering_deg: float,
    dt: float,
    wheel_base: float,
) -> Tuple[float, float, float]:
    """Integrate vehicle pose assuming Ackermann steering.

    Parameters
    ----------
    x, y, yaw_deg : float
        Current position and yaw in degrees.
    speed : float
        Forward speed in metres per second.
    steering_deg : float
        Steering angle in degrees.
    dt : float
        Time step in seconds.
    wheel_base : float
        Vehicle wheel base in metres.

    Returns
    -------
    Tuple[float, float, float]
        ``(x_new, y_new, yaw_deg_new)`` after ``dt`` seconds.
    """
    yaw_rad = math.radians(yaw_deg)
    steer_rad = math.radians(steering_deg)
    x_new = x + math.sin(yaw_rad) * speed * dt
    y_new = y + math.cos(yaw_rad) * speed * dt
    yaw_new = yaw_deg + math.degrees(
        (speed / wheel_base) * math.tan(steer_rad) * dt
    )
    return x_new, y_new, yaw_new


def forward_feedback_control(
    x: float,
    y: float,
    theta: float,
    path_point: Tuple[float, float],
    v: float,
    L: float,
    fps: float,
) -> Tuple[float, float, float, float]:
    """Compute new pose and steering angle using forward feedback.

    Parameters
    ----------
    x, y, theta : float
        Current position ``(x, y)`` and orientation ``theta`` in radians.
    path_point : Tuple[float, float]
        Target point on the planned path reachable in the next step.
    v : float
        Current vehicle speed in metres per second.
    L : float
        Wheelbase of the vehicle in metres.
    fps : float
        Simulation framerate. ``dt`` is derived as ``1 / fps``.

    Returns
    -------
    Tuple[float, float, float, float]
        ``(x_new, y_new, theta_new, delta)`` where ``delta`` is the steering
        angle required to reach ``path_point``. The remaining values represent
        the predicted vehicle pose after one timestep.
    """
    dt = 1.0 / fps
    s = v * dt  # distance travelled in one timestep

    target_x, target_y = path_point

    dx = target_x - x
    dy = target_y - y
    alpha = math.atan2(dy, dx) - theta

    delta = math.atan2(2 * L * math.sin(alpha), s) if s != 0 else 0.0

    x_new = x + v * math.cos(theta) * dt
    y_new = y + v * math.sin(theta) * dt
    theta_new = theta + (v / L) * math.tan(delta) * dt

    return x_new, y_new, theta_new, delta
