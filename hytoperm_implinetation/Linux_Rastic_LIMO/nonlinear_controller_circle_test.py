import numpy as np
import matplotlib.pyplot as plt

# Parameters
R = 5.0  # Radius of the circle
T = 20.0  # Total simulation time (seconds)
dt = 0.01  # Time step
N = int(T / dt)

# Controller gains
k_x = 1.0
k_y = 5.0
k_theta = 8.0

# Initialize state: (x, y, theta)
x = 0.0
y = 0.0
theta = 0.0

# Store history for plotting
x_hist = []
y_hist = []
theta_hist = []

x_d_hist = []
y_d_hist = []
theta_d_hist = []

e_x_hist = []
e_y_hist = []
e_theta_hist = []

time = np.linspace(0, T, N)

def wrap_angle(angle):
    """Wrap angle to [-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

for t in time:
    # Desired trajectory and derivatives
    x_d = R * np.cos(t)
    y_d = R * np.sin(t)
    dx_d = -R * np.sin(t)
    dy_d = R * np.cos(t)

    # Desired heading and its derivative
    theta_d = np.arctan2(dy_d, dx_d)

    # Compute time derivative of theta_d numerically
    # Approximate with finite difference since we know the analytical form
    # dtheta_d/dt = (x'' y' - y'' x')/(x'^2 + y'^2)
    ddx_d = -R * np.cos(t)
    ddy_d = -R * np.sin(t)
    numerator = ddx_d * dy_d - ddy_d * dx_d
    denominator = dx_d**2 + dy_d**2
    w_d = numerator / denominator

    # Desired speed
    v_d = np.sqrt(dx_d**2 + dy_d**2)

    # Tracking error in robot frame
    dx = x_d - x
    dy = y_d - y
    e_x =  np.cos(theta)*dx + np.sin(theta)*dy
    e_y = -np.sin(theta)*dx + np.cos(theta)*dy
    e_theta = wrap_angle(theta_d - theta)

    # Controller (Kanayama)
    # Handle division by zero for small e_theta
    if abs(e_theta) < 1e-6:
        sin_over_theta = 1.0
    else:
        sin_over_theta = np.sin(e_theta)/e_theta

    v = v_d * np.cos(e_theta) + k_x * e_x
    w = w_d + k_y * v_d * sin_over_theta * e_y + k_theta * e_theta

    # Simulate unicycle dynamics (Euler integration)
    x += v * np.cos(theta) * dt
    y += v * np.sin(theta) * dt
    theta += w * dt
    theta = wrap_angle(theta)

    # Store data
    x_hist.append(x)
    y_hist.append(y)
    theta_hist.append(theta)

    x_d_hist.append(x_d)
    y_d_hist.append(y_d)
    theta_d_hist.append(theta_d)

    e_x_hist.append(e_x)
    e_y_hist.append(e_y)
    e_theta_hist.append(e_theta)

# Convert lists to arrays for plotting
x_hist = np.array(x_hist)
y_hist = np.array(y_hist)
theta_hist = np.array(theta_hist)

x_d_hist = np.array(x_d_hist)
y_d_hist = np.array(y_d_hist)
theta_d_hist = np.array(theta_d_hist)

e_x_hist = np.array(e_x_hist)
e_y_hist = np.array(e_y_hist)
e_theta_hist = np.array(e_theta_hist)

# Plot results
plt.figure(figsize=(12,5))

plt.subplot(1,2,1)
plt.plot(x_d_hist, y_d_hist, 'r--', label='Desired trajectory')
plt.plot(x_hist, y_hist, 'b-', label='Robot trajectory')
plt.axis('equal')
plt.xlabel('x')
plt.ylabel('y')
plt.title('Unicycle Tracking Trajectory')
plt.legend()
plt.grid()

plt.subplot(1,2,2)
plt.plot(time, e_x_hist, label='$e_x$ (m)')
plt.plot(time, e_y_hist, label='$e_y$ (m)')
plt.plot(time, e_theta_hist, label=r'$e_{\theta}$ (rad)')
plt.xlabel('Time [s]')
plt.ylabel('Tracking errors')
plt.title('Tracking Errors vs Time')
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()

