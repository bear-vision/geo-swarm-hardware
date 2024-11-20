import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# Example start and goal positions
start_x, start_y, start_z = 0, 0, 0  # Start position
goal_x, goal_y, goal_z = 10, 10, 10  # Goal position

# Example times (for simplicity, we'll assume linear time progression)
t = np.linspace(0, 1, 100)  # Normalized time from 0 to 1

# Set velocities and accelerations at start and end
start_velocity = 0  # Initial velocity
goal_velocity = 0  # Final velocity (linear velocity)
start_acceleration = 0  # Initial acceleration (constant)
goal_acceleration = 0  # Final acceleration (constant)

# Set the positions as simple linear interpolation between start and goal
# These would be the boundary conditions for cubic splines
positions = np.array([start_x, goal_x])
velocities = np.array([start_velocity, goal_velocity])
accelerations = np.array([start_acceleration, goal_acceleration])

# Create a cubic spline for each dimension
cs_x = CubicSpline([0, 1], positions, bc_type=((1, velocities[0]), (1, velocities[1])))
cs_y = CubicSpline([0, 1], positions, bc_type=((1, velocities[0]), (1, velocities[1])))
cs_z = CubicSpline([0, 1], positions, bc_type=((1, velocities[0]), (1, velocities[1])))

# Get the values of position, velocity, and acceleration for each time step
x_values = cs_x(t)
y_values = cs_y(t)
z_values = cs_z(t)

# Plot the results
plt.figure(figsize=(12, 6))

# Position plot
plt.subplot(2, 2, 1)
plt.plot(t, x_values, label="X Position")
plt.plot(t, y_values, label="Y Position")
plt.plot(t, z_values, label="Z Position")
plt.title("Position Along the Path")
plt.legend()

# Velocity plot (first derivative of position)
plt.subplot(2, 2, 2)
plt.plot(t, cs_x.derivative(1)(t), label="X Velocity")
plt.plot(t, cs_y.derivative(1)(t), label="Y Velocity")
plt.plot(t, cs_z.derivative(1)(t), label="Z Velocity")
plt.title("Velocity Along the Path")
plt.legend()

# Acceleration plot (second derivative of position)
plt.subplot(2, 2, 3)
plt.plot(t, cs_x.derivative(2)(t), label="X Acceleration")
plt.plot(t, cs_y.derivative(2)(t), label="Y Acceleration")
plt.plot(t, cs_z.derivative(2)(t), label="Z Acceleration")
plt.title("Acceleration Along the Path")
plt.legend()

plt.tight_layout()
plt.show()
