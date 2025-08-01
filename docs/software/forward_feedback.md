# Forward Feedback Control

The forward feedback controller computes steering and speed commands so that the vehicle reaches the next point on a path.
It uses a simple kinematic bicycle model with time step `dt = 1 / fps`.

```python
from vehicle_control.forward_feedback import forward_feedback_control

x_new, y_new, theta_new, delta = forward_feedback_control(
    x, y, theta, target_point, v, wheel_base, fps
)
```

To integrate the vehicle pose directly using a steering command in degrees, use
``kinematic_step_deg``:

```python
from vehicle_control.forward_feedback import kinematic_step_deg

x_new, y_new, yaw_new = kinematic_step_deg(
    x, y, yaw_deg, speed, steering_deg, dt, wheel_base
)
```

The steering angle `delta` is calculated using the pure pursuit rule:
`delta = atan2(2 * L * sin(alpha), s)` where `alpha` is the angle from the
current heading to the target point and `s` is the distance travelled in one
step. The new pose `(x_new, y_new, theta_new)` is then obtained by integrating
the vehicle motion over `dt`.

The simulation framerate can be adjusted at runtime. The pathfinding node
publishes the current frame rate on the ``/path_inference_fps`` topic and the
detection node adapts its forward prediction timer accordingly.
