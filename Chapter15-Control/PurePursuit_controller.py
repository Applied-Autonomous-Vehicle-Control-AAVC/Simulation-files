# pure_pursuit_path_tracking.py
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

# Pure Pursuit parameters
LD_MIN = 6.0          # minimum lookahead distance [m]
LD_GAIN = 0.1         # (optional) speed-proportional term: Ld = LD_MIN + LD_GAIN * v

# Vehicle shape
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
    Generate a smooth, reproducible 'random' 2D path.
    Returns arrays: x, y, yaw, s (arc length).
    """
    rng = np.random.RandomState(SEED)
    t = np.linspace(0.0, 2.0 * np.pi, num_pts)

    def smooth_series(freqs, amp_scale):
        y = np.zeros_like(t)
        for f in freqs:
            phase = rng.uniform(0, 2*np.pi)
            amp = amp_scale * rng.uniform(0.4, 1.0)
            y += amp * np.sin(f * t + phase)
        return y

    x = np.linspace(0, span, num_pts) + smooth_series([1, 2.5], amp_scale=2.0)
    y = smooth_series([0.8, 1.6, 2.2], amp_scale=6.0)

    dx = np.gradient(x)
    dy = np.gradient(y)
    yaw = np.arctan2(dy, dx)

    ds = np.hypot(dx, dy)
    s = np.cumsum(ds)
    s -= s[0]
    return x, y, yaw, s

def nearest_path_index(px, py, path_x, path_y):
    """
    Return index of nearest path point to (px, py).
    (Used internally only; not visualized.)
    """
    dx = path_x - px
    dy = path_y - py
    d2 = dx * dx + dy * dy
    return int(np.argmin(d2))

def pure_pursuit_target_index(px, py, path_x, path_y, path_s, lookahead_dist):
    """
    Pick a target point on the path at least 'lookahead_dist' ahead of the
    nearest index along arc length s.
    """
    nidx = nearest_path_index(px, py, path_x, path_y)
    # Walk forward in s until we are lookahead_dist ahead (or end of path)
    s0 = path_s[nidx]
    tidx = nidx
    while tidx < len(path_s) - 1 and (path_s[tidx] - s0) < lookahead_dist:
        tidx += 1
    return tidx

def pure_pursuit_control(px, py, yaw, v, path_x, path_y, path_s, L, ld_min, ld_gain):
    """
    Pure Pursuit steering:
        delta = atan2(2 * L * sin(alpha) / Ld)
    where alpha is the angle from the vehicle to the target point in the vehicle frame.
    """
    Ld = max(ld_min, ld_min + ld_gain * v)  # simple speed-proportional Ld
    tidx = pure_pursuit_target_index(px, py, path_x, path_y, path_s, Ld)

    tx = path_x[tidx]
    ty = path_y[tidx]

    # Transform: compute angle from vehicle to target in world, then relative to yaw
    alpha = math.atan2(ty - py, tx - px) - yaw
    alpha = wrap_to_pi(alpha)

    # Steering law (bicycle model geometry)
    # Guard small Ld
    Ld_eff = max(1e-6, Ld)
    delta = math.atan2(2.0 * L * math.sin(alpha) / Ld_eff, 1.0)

    return delta, Ld, (tx, ty), tidx

def vehicle_step(x, y, yaw, v, delta, L, dt):
    """
    Kinematic bicycle model update.
    """
    max_steer = np.deg2rad(35.0)
    delta = float(np.clip(delta, -max_steer, max_steer))

    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    yaw += v / L * math.tan(delta) * dt
    yaw = wrap_to_pi(yaw)
    return x, y, yaw, delta

def vehicle_rectangle(x, y, yaw, length, width):
    """
    Return rectangle polygon representing vehicle body, rear axle at (x, y).
    """
    cx = x + (length / 2.0) * math.cos(yaw)
    cy = y + (length / 2.0) * math.sin(yaw)

    hl = length / 2.0
    hw = width / 2.0
    pts = np.array([
        [ hl,  hw],
        [ hl, -hw],
        [-hl, -hw],
        [-hl,  hw],
        [ hl,  hw],
    ])

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
fig.canvas.manager.set_window_title("Pure Pursuit Path Tracking (2D)")

# Plot full global path (requirements #2 & #4)
global_path_line, = ax.plot(PATH_X, PATH_Y, linestyle='-', linewidth=2.0, alpha=0.6, label="Global Path")

# Vehicle body (rectangle)
veh_poly, = ax.plot([], [], linewidth=2.0, label="Vehicle Body")

# Vehicle heading line
heading_line, = ax.plot([], [], linewidth=2.0, linestyle='--', alpha=0.8, label="Vehicle Heading")

# Traveled path
trace_line, = ax.plot([], [], linewidth=2.0, alpha=0.9, label="Traveled Path")

# Lookahead target point (NOT the closest-point visualization)
lookahead_pt, = ax.plot([], [], marker='o', markersize=6, linestyle='None', label="Lookahead Target")

# Telemetry box
telemetry = ax.text(0.02, 0.98, "", ha='left', va='top', transform=ax.transAxes, fontsize=10,
                    bbox=dict(boxstyle='round', alpha=0.2, pad=0.4))

# Legend (critical components)
legend = ax.legend(loc="upper right", framealpha=0.9)

# Nice view limits around whole path
pad = 10.0
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(np.min(PATH_X) - pad, np.max(PATH_X) + pad)
ax.set_ylim(np.min(PATH_Y) - pad, np.max(PATH_Y) + pad)
ax.set_title("2D Vehicle Path Tracking with Pure Pursuit Controller")
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
    lookahead_pt.set_data([], [])
    telemetry.set_text("")
    return global_path_line, trace_line, veh_poly, heading_line, lookahead_pt, telemetry

def animate(frame_idx):
    global x, y, yaw, sim_time

    # Pure Pursuit control
    delta, Ld, (tx, ty), tidx = pure_pursuit_control(
        x, y, yaw, V, PATH_X, PATH_Y, PATH_S, L, LD_MIN, LD_GAIN
    )

    # Step vehicle
    x, y, yaw, delta = vehicle_step(x, y, yaw, V, delta, L, DT)
    sim_time += DT

    # Append history
    history_x.append(x)
    history_y.append(y)
    history_delta.append(delta)

    # Update vehicle polygon
    poly = vehicle_rectangle(x, y, yaw, VEH_LEN, VEH_WID)
    veh_poly.set_data(poly[:, 0], poly[:, 1])

    # Heading line
    head_len = VEH_LEN * 0.8
    hx0, hy0 = x, y
    hx1 = x + head_len * math.cos(yaw)
    hy1 = y + head_len * math.sin(yaw)
    heading_line.set_data([hx0, hx1], [hy0, hy1])

    # Traveled path
    trace_line.set_data(history_x, history_y)

    # Lookahead target (for clarity; not a closest-point visualization)
    lookahead_pt.set_data([tx], [ty])

    # Telemetry
    telemetry.set_text(
        "Time: {:.1f} s\nSpeed: {:.1f} m/s\nSteer: {: .1f}°\nLookahead Ld: {:.2f} m".format(
            sim_time, V, np.rad2deg(delta), Ld
        )
    )

    return global_path_line, trace_line, veh_poly, heading_line, lookahead_pt, telemetry

# ----------------------------
# Run the animation
# ----------------------------
frames = int(TOTAL_TIME / DT)
ani = animation.FuncAnimation(fig, animate, init_func=init_anim, frames=frames,
                              interval=DT * 1000.0, blit=True, repeat=False)

if __name__ == "__main__":
    # Requirement #5: show animation/plot window
    plt.show()
