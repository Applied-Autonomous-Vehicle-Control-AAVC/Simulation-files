# stanley_path_tracking.py
# Python 3.6–3.9 compatible
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation

# ----------------------------
# Global settings (feel free to tweak)
# ----------------------------
SEED = 7              # for reproducible "random" global path
DT = 0.05             # simulation time step [s]
TOTAL_TIME = 60.0     # total sim time [s]
V = 15.0               # constant forward speed [m/s]
L = 2.7               # wheelbase [m]
K_STANLEY = 1.2       # Stanley gain
SOFTENING = 1e-6      # prevents atan2 blowups
VEH_LEN = 4.5         # vehicle rectangle length [m]
VEH_WID = 1.9         # vehicle rectangle width [m]

# ----------------------------
# Helper functions
# ----------------------------
def wrap_to_pi(angle):
    """Wrap angle to [-pi, pi]."""
    a = (angle + np.pi) % (2 * np.pi) - np.pi
    return a

def make_smooth_global_path(num_pts=1200, span=180.0):
    """
    Generate a smooth, reproducible 'random' 2D path without external packages.
    The path is parametric: x(t), y(t) with smooth curvature.
    """
    rng = np.random.RandomState(SEED)
    t = np.linspace(0.0, 2.0 * np.pi, num_pts)

    # Build smooth signals by summing sinusoids with random phases/amplitudes
    def smooth_series(freqs, amp_scale):
        y = np.zeros_like(t)
        for f in freqs:
            phase = rng.uniform(0, 2*np.pi)
            amp = amp_scale * rng.uniform(0.4, 1.0)
            y += amp * np.sin(f * t + phase)
        return y

    # Base track: progress along x, wiggle in both x and y for a wandering path
    x = np.linspace(0, span, num_pts) + smooth_series([1, 2.5], amp_scale=2.0)
    y = smooth_series([0.8, 1.6, 2.2], amp_scale=6.0)

    # Compute headings (yaw) from finite differences for each path point
    dx = np.gradient(x)
    dy = np.gradient(y)
    yaw = np.arctan2(dy, dx)

    # Arc-length parameter s for convenience
    ds = np.hypot(dx, dy)
    s = np.cumsum(ds)
    s -= s[0]

    return x, y, yaw, s

def nearest_path_index(px, py, path_x, path_y):
    """
    Return index of nearest path point to (px, py).
    (We do not *visualize* this; it’s only internal for control.)
    """
    dx = path_x - px
    dy = path_y - py
    d2 = dx * dx + dy * dy
    return int(np.argmin(d2))

def stanley_control(px, py, yaw, v, path_x, path_y, path_yaw, k=1.0):
    """
    Stanley steering law. Returns steering angle [rad] and target path yaw [rad].
    """
    target_idx = nearest_path_index(px, py, path_x, path_y)
    # Heading error
    path_heading = path_yaw[target_idx]
    heading_error = wrap_to_pi(path_heading - yaw)

    # Cross-track error sign using a simple lateral projection
    dx = path_x[target_idx] - px
    dy = path_y[target_idx] - py
    # Vehicle heading unit vector normal (to the left)
    # Left normal of vehicle heading yaw is [ -sin(yaw), cos(yaw) ]
    cte = dx * (-math.sin(yaw)) + dy * (math.cos(yaw))

    # Stanley term
    steer_correction = math.atan2(k * cte, v + SOFTENING)
    delta = wrap_to_pi(heading_error + steer_correction)
    return delta, path_heading

def vehicle_step(x, y, yaw, v, delta, L, dt):
    """
    Kinematic bicycle model with small-angle-safe update.
    """
    # Limit realistic steering angle
    max_steer = np.deg2rad(35.0)
    delta = float(np.clip(delta, -max_steer, max_steer))

    # Update pose
    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    yaw += v / L * math.tan(delta) * dt
    yaw = wrap_to_pi(yaw)
    return x, y, yaw, delta

def vehicle_rectangle(x, y, yaw, length, width):
    """
    Return rectangle (as polygon) representing vehicle body centered at (x,y)
    with heading yaw. Rectangle is centered at the rear-axle + length/2 ahead.
    """
    # Place rectangle so rear axle is at (x, y), extending length forward
    cx = x + (length / 2.0) * math.cos(yaw)
    cy = y + (length / 2.0) * math.sin(yaw)

    # Rectangle corners in local frame (centered)
    hl = length / 2.0
    hw = width / 2.0
    # local coordinates (counter-clockwise)
    pts = np.array([
        [ hl,  hw],
        [ hl, -hw],
        [-hl, -hw],
        [-hl,  hw],
        [ hl,  hw],  # close polygon
    ])

    # Rotation
    R = np.array([
        [ math.cos(yaw), -math.sin(yaw)],
        [ math.sin(yaw),  math.cos(yaw)]
    ])
    world = (R @ pts.T).T
    world[:, 0] += cx
    world[:, 1] += cy
    return world

# ----------------------------
# Build the global path
# ----------------------------
PATH_X, PATH_Y, PATH_YAW, PATH_S = make_smooth_global_path()

# ----------------------------
# Simulation state
# ----------------------------
# Start near the first path point, slightly offset and misaligned
x = PATH_X[0] - 2.0
y = PATH_Y[0] - 3.0
yaw = PATH_YAW[0] + np.deg2rad(30.0)

history_x = [x]
history_y = [y]
history_delta = []

# ----------------------------
# Matplotlib animation setup
# ----------------------------
plt.close("all")
fig, ax = plt.subplots(figsize=(9, 6))
fig.canvas.manager.set_window_title("Stanley Path Tracking (2D)")

# Plot full global path (requirement #2 & #4)
global_path_line, = ax.plot(PATH_X, PATH_Y, linestyle='-', linewidth=2.0, alpha=0.6, label="Global Path")

# Vehicle body (rectangle)
veh_poly, = ax.plot([], [], linewidth=2.0, label="Vehicle Body")

# Vehicle heading as a short line/arrow
heading_line, = ax.plot([], [], linewidth=2.0, linestyle='--', alpha=0.8, label="Vehicle Heading")

# Trace of traveled path
trace_line, = ax.plot([], [], linewidth=2.0, alpha=0.9, label="Traveled Path")

# Add text box for telemetry
telemetry = ax.text(0.02, 0.98, "", ha='left', va='top', transform=ax.transAxes, fontsize=10,
                    bbox=dict(boxstyle='round', alpha=0.2, pad=0.4))

# Legend (requirement: show critical components)
legend = ax.legend(loc="upper right", framealpha=0.9)

# Nice view limits around the whole path
pad = 10.0
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(np.min(PATH_X) - pad, np.max(PATH_X) + pad)
ax.set_ylim(np.min(PATH_Y) - pad, np.max(PATH_Y) + pad)
ax.set_title("2D Vehicle Path Tracking with Stanley Controller")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")

# ----------------------------
# Animation update function
# ----------------------------
sim_time = 0.0

def init_anim():
    trace_line.set_data([], [])
    veh_poly.set_data([], [])
    heading_line.set_data([], [])
    telemetry.set_text("")
    return global_path_line, trace_line, veh_poly, heading_line, telemetry

def animate(frame_idx):
    global x, y, yaw, sim_time

    # Compute steering using Stanley
    delta, path_heading = stanley_control(x, y, yaw, V, PATH_X, PATH_Y, PATH_YAW, k=K_STANLEY)

    # Step vehicle dynamics
    x, y, yaw, delta = vehicle_step(x, y, yaw, V, delta, L, DT)
    sim_time += DT

    # Append history
    history_x.append(x)
    history_y.append(y)
    history_delta.append(delta)

    # Update vehicle polygon
    poly = vehicle_rectangle(x, y, yaw, VEH_LEN, VEH_WID)
    veh_poly.set_data(poly[:, 0], poly[:, 1])

    # Update heading line (short line from center of rear axle)
    head_len = VEH_LEN * 0.8
    hx0, hy0 = x, y
    hx1 = x + head_len * math.cos(yaw)
    hy1 = y + head_len * math.sin(yaw)
    heading_line.set_data([hx0, hx1], [hy0, hy1])

    # Update trace
    trace_line.set_data(history_x, history_y)

    # Update telemetry text
    telemetry.set_text(
        "Time: {:.1f} s\nSpeed: {:.1f} m/s\nSteer: {: .1f}°\nStanley k: {:.2f}".format(
            sim_time, V, np.rad2deg(delta), K_STANLEY
        )
    )

    # Return artists to blit
    return global_path_line, trace_line, veh_poly, heading_line, telemetry

# ----------------------------
# Run the animation
# ----------------------------
frames = int(TOTAL_TIME / DT)
ani = animation.FuncAnimation(fig, animate, init_func=init_anim, frames=frames,
                              interval=DT * 1000.0, blit=True, repeat=False)

# Requirement #5: show a window with the animation/plot
if __name__ == "__main__":
    # Note: saving (e.g., with ani.save) is optional and not required here.
    plt.show()
