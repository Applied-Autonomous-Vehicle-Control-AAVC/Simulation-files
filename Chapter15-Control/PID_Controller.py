# pid_cte_path_tracking.py
# Python 3.6–3.9 compatible
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation

# ----------------------------
# Global settings (tweak freely)
# ----------------------------
SEED = 7              # reproducible global path
DT = 0.05             # sim time step [s]
TOTAL_TIME = 60.0     # total sim time [s]
V = 8.0               # constant forward speed [m/s]
L = 2.7               # wheelbase [m]

# PID (on lateral cross-track error only)
KP = 0.6
KI = 0.02
KD = 0.8
I_CLAMP = np.deg2rad(25.0)   # anti-windup clamp for the integral term (in "steer-equivalent" units)

# Steering limits
MAX_STEER_DEG = 35.0

# Vehicle shape
VEH_LEN = 4.5
VEH_WID  = 1.9

# ----------------------------
# Helpers
# ----------------------------
def wrap_to_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

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
    Internal only: index of path point nearest to (px, py).
    (Not visualized to satisfy the 'no closest point visualization' requirement.)
    """
    dx = path_x - px
    dy = path_y - py
    d2 = dx * dx + dy * dy
    return int(np.argmin(d2))

def cross_track_error(px, py, yaw, path_x, path_y):
    """
    Signed lateral error: positive if path is to the left of vehicle heading.
    Uses the nearest point to define the error sign via the vehicle's left normal.
    """
    idx = nearest_path_index(px, py, path_x, path_y)
    dx = path_x[idx] - px
    dy = path_y[idx] - py
    # Left normal of vehicle heading: [-sin(yaw), cos(yaw)]
    cte = dx * (-math.sin(yaw)) + dy * ( math.cos(yaw))
    return cte

def pid_cte_steering(cte, cte_rate, integ, kp, ki, kd):
    """
    PID on cross-track error -> steering command (radians).
    """
    u_p = kp * cte
    u_i = ki * integ
    u_d = kd * cte_rate
    delta = u_p + u_i + u_d
    return delta, u_p, u_i, u_d

def vehicle_step(x, y, yaw, v, delta, L, dt):
    """
    Kinematic bicycle model update from rear axle pose.
    """
    max_steer = np.deg2rad(MAX_STEER_DEG)
    delta = float(np.clip(delta, -max_steer, max_steer))
    x += v * math.cos(yaw) * dt
    y += v * math.sin(yaw) * dt
    yaw += v / L * math.tan(delta) * dt
    yaw = wrap_to_pi(yaw)
    return x, y, yaw, delta

def vehicle_rectangle(x, y, yaw, length, width):
    """
    Rectangle polygon (rear axle at (x,y), extending forward).
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
# Build global path
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
history_cte = []

cte_prev = 0.0
integ_cte = 0.0
sim_time = 0.0

# ----------------------------
# Plot / Animation setup
# ----------------------------
plt.close("all")
fig, ax = plt.subplots(figsize=(9, 6))
fig.canvas.manager.set_window_title("PID (CTE-only) Path Tracking (2D)")

# Full global path (requirements #2 & #4)
global_path_line, = ax.plot(PATH_X, PATH_Y, linestyle='-', linewidth=2.0, alpha=0.6, label="Global Path")

# Vehicle body
veh_poly, = ax.plot([], [], linewidth=2.0, label="Vehicle Body")

# Vehicle heading
heading_line, = ax.plot([], [], linewidth=2.0, linestyle='--', alpha=0.8, label="Vehicle Heading")

# Traveled path
trace_line, = ax.plot([], [], linewidth=2.0, alpha=0.9, label="Traveled Path")

# Telemetry box
telemetry = ax.text(0.02, 0.98, "", ha='left', va='top', transform=ax.transAxes, fontsize=10,
                    bbox=dict(boxstyle='round', alpha=0.2, pad=0.4))

# Legend
legend = ax.legend(loc="upper right", framealpha=0.9)

# Axes settings
pad = 10.0
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(np.min(PATH_X) - pad, np.max(PATH_X) + pad)
ax.set_ylim(np.min(PATH_Y) - pad, np.max(PATH_Y) + pad)
ax.set_title("2D Vehicle Path Tracking with PID (Cross-Track Error Only)")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")

# ----------------------------
# Animation functions
# ----------------------------
def init_anim():
    trace_line.set_data([], [])
    veh_poly.set_data([], [])
    heading_line.set_data([], [])
    telemetry.set_text("")
    return global_path_line, trace_line, veh_poly, heading_line, telemetry

def animate(frame_idx):
    global x, y, yaw, sim_time, cte_prev, integ_cte

    # --- Compute CTE and its rate ---
    cte = cross_track_error(x, y, yaw, PATH_X, PATH_Y)
    cte_rate = (cte - cte_prev) / DT

    # Integrate with simple anti-windup clamp in "steer-equivalent" space
    # We clamp the integral contribution indirectly by clamping the accumulator
    # so that KI * integ stays within +/- I_CLAMP.
    integ_cte += cte * DT
    max_integ = I_CLAMP / max(1e-6, KI)
    integ_cte = float(np.clip(integ_cte, -max_integ, max_integ))

    # --- PID controller on CTE only (no heading term) ---
    delta, u_p, u_i, u_d = pid_cte_steering(cte, cte_rate, integ_cte, KP, KI, KD)

    # --- Vehicle step ---
    x, y, yaw, delta = vehicle_step(x, y, yaw, V, delta, L, DT)
    sim_time += DT

    # --- Bookkeeping ---
    cte_prev = cte
    history_x.append(x)
    history_y.append(y)
    history_delta.append(delta)
    history_cte.append(cte)

    # --- Draw vehicle rectangle ---
    poly = vehicle_rectangle(x, y, yaw, VEH_LEN, VEH_WID)
    veh_poly.set_data(poly[:, 0], poly[:, 1])

    # --- Heading line ---
    head_len = VEH_LEN * 0.8
    hx0, hy0 = x, y
    hx1 = x + head_len * math.cos(yaw)
    hy1 = y + head_len * math.sin(yaw)
    heading_line.set_data([hx0, hx1], [hy0, hy1])

    # --- Traveled path ---
    trace_line.set_data(history_x, history_y)

    # --- Telemetry ---
    telemetry.set_text(
        "Time: {:.1f} s\nSpeed: {:.1f} m/s\nSteer: {: .1f}°\nCTE: {: .2f} m\nPID: P({:.2f}) I({:.2f}) D({:.2f})".format(
            sim_time, V, np.rad2deg(delta), cte, u_p, u_i, u_d
        )
    )

    return global_path_line, trace_line, veh_poly, heading_line, telemetry

# ----------------------------
# Run the animation
# ----------------------------
frames = int(TOTAL_TIME / DT)
ani = animation.FuncAnimation(
    fig, animate, init_func=init_anim, frames=frames, interval=DT * 1000.0, blit=True, repeat=False
)

if __name__ == "__main__":
    # Requirement #5: show an animation/plot window
    plt.show()
