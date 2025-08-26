# mpc_kinematic_path_tracking.py
# Python 3.6–3.9 compatible
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation

# ----------------------------
# Global / Simulation settings
# ----------------------------
SEED = 7               # reproducible "random" global path
DT = 0.05              # [s] simulation time step
TOTAL_TIME = 60.0      # [s]
V = 6.0                # [m/s] constant forward speed

# Vehicle physical parameters (typical compact car-ish)
L = 2.7                # [m]   wheelbase

# Vehicle drawing
VEH_LEN = 4.5
VEH_WID  = 1.9

# MPC settings (time-varying linear MPC with least-squares solve, no constraints)
N = 20                 # horizon length (steps)
Q_xy = 2.0             # weight on x,y error
Q_yaw = 0.5            # weight on yaw error
Qf_xy = 6.0            # terminal weights
Qf_yaw = 2.0
R_steer = 0.05         # weight on steering effort
dR_steer = 0.1         # (optional) weight on steering rate (Δδ); set 0 to disable

# Steering limits (applied after solve)
MAX_STEER_DEG = 35.0

# ----------------------------
# Helpers
# ----------------------------
def wrap_to_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def make_smooth_global_path(num_pts=1200, span=180.0):
    """
    Generate a smooth, reproducible 'random' 2D path.
    Returns: x, y, yaw, s (arc length).
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
    """Internal only: index of path point nearest to (px, py). (Not visualized.)"""
    dx = path_x - px
    dy = path_y - py
    return int(np.argmin(dx*dx + dy*dy))

def vehicle_rectangle(x, y, yaw, length, width):
    """Return polygon for vehicle body, rear axle at (x,y)."""
    cx = x + (length / 2.0) * math.cos(yaw)
    cy = y + (length / 2.0) * math.sin(yaw)
    hl = length / 2.0
    hw = width / 2.0
    pts = np.array([[ hl,  hw],
                    [ hl, -hw],
                    [-hl, -hw],
                    [-hl,  hw],
                    [ hl,  hw]])
    R = np.array([[ math.cos(yaw), -math.sin(yaw)],
                  [ math.sin(yaw),  math.cos(yaw)]])
    world = (R @ pts.T).T
    world[:, 0] += cx
    world[:, 1] += cy
    return world

# ----------------------------
# Kinematic model (discrete)
# ----------------------------
def step_kinematic(x, y, yaw, v, delta, L, dt):
    max_steer = np.deg2rad(MAX_STEER_DEG)
    delta = float(np.clip(delta, -max_steer, max_steer))
    x   += v * math.cos(yaw) * dt
    y   += v * math.sin(yaw) * dt
    yaw += v / L * math.tan(delta) * dt
    yaw  = wrap_to_pi(yaw)
    return x, y, yaw, delta

def linearize_AB(psi_r, L, v, dt):
    """
    Linearize the discrete model x+ = f(x,u) around reference yaw psi_r and delta_r=0.
    States: [x, y, yaw]^T, Control: u = delta.
    A = ∂f/∂x, B = ∂f/∂u (Euler discretization already embedded above).
    """
    A = np.array([[1.0, 0.0, -v*dt*math.sin(psi_r)],
                  [0.0, 1.0,  v*dt*math.cos(psi_r)],
                  [0.0, 0.0,  1.0]])
    B = np.array([[0.0],
                  [0.0],
                  [v*dt/L]])
    return A, B

def build_timevarying_prediction(psi_ref_seq, A_list, B_list):
    """
    Build stacked prediction matrices for a time-varying linear system:
        x_{k+1} = A_k x_k + B_k u_k
    Returns:
      Phi_stack: stack of per-step state transition applied to x0 (shape: (N*nx, nx))
      Gamma:     block-lower-triangular input map (shape: (N*nx, N*nu))
    """
    nx = 3
    nu = 1
    N = len(A_list)
    Phi_rows = []
    # cumulative product for each step i: A_{i-1} ... A_0
    Ak_prod = np.eye(nx)
    Phi_each = []
    for k in range(N):
        Ak_prod = A_list[k] @ Ak_prod
        Phi_each.append(Ak_prod.copy())
        Phi_rows.append(Ak_prod)
    Phi_stack = np.vstack(Phi_rows)  # (N*nx, nx)

    # Build Gamma
    Gamma = np.zeros((N*nx, N*nu))
    for i in range(N):          # row block (predict x_{i+1})
        # contribution from u_j, j=0..i
        Aj_prod = np.eye(nx)
        for j in range(i, -1, -1):
            # product A_i A_{i-1} ... A_{j+1}
            Aj_prod = np.eye(nx)
            for k in range(i, j, -1):
                Aj_prod = A_list[k] @ Aj_prod
            Bij = Aj_prod @ B_list[j]
            Gamma[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = Bij
    return Phi_stack, Gamma

def form_cost_matrices(N, nx, nu, Q, Qf, R, dR=None):
    """
    Build block-diagonal Qbar (for x1..xN) and Rbar (for u0..u_{N-1}).
    Optionally augment R with first-order difference penalty dR on Δu.
    """
    # State weights
    Q_blocks = [Q] * (N-1) + [Qf]
    Qbar = np.zeros((N*nx, N*nx))
    for i,Qi in enumerate(Q_blocks):
        Qbar[i*nx:(i+1)*nx, i*nx:(i+1)*nx] = Qi

    # Control weights
    Rbar = np.zeros((N*nu, N*nu))
    for i in range(N):
        Rbar[i*nu:(i+1)*nu, i*nu:(i+1)*nu] = R

    # Δu penalty (tridiagonal structure)
    if dR is not None and dR > 0.0 and N > 1:
        for i in range(N):
            Rbar[i*nu:(i+1)*nu, i*nu:(i+1)*nu] += dR
            if i > 0:
                Rbar[i*nu:(i+1)*nu, (i-1)*nu:i*nu] += -dR
                Rbar[(i-1)*nu:i*nu, i*nu:(i+1)*nu] += -dR
                Rbar[(i-1)*nu:i*nu, (i-1)*nu:i*nu] += dR
    return Qbar, Rbar

# ----------------------------
# Build the global path
# ----------------------------
PATH_X, PATH_Y, PATH_YAW, PATH_S = make_smooth_global_path()

# ----------------------------
# MPC helper to assemble reference over horizon
# ----------------------------
def reference_window(idx_near, N, path_x, path_y, path_yaw):
    """
    Build state reference over the horizon:
      x_ref = [x1,y1,yaw1, x2,y2,yaw2, ..., xN,yN,yawN]^T
    Simply walks forward along the path indices (clamping at the end).
    """
    nx = 3
    Xref = np.zeros((N*nx,))
    n = len(path_x)
    for k in range(N):
        i = min(idx_near + 1 + k, n - 1)
        Xref[k*nx + 0] = path_x[i]
        Xref[k*nx + 1] = path_y[i]
        Xref[k*nx + 2] = path_yaw[i]
    return Xref

def build_AB_lists(idx_near, N, path_yaw):
    A_list, B_list = [], []
    n = len(path_yaw)
    for k in range(N):
        i = min(idx_near + k, n - 1)
        A,B = linearize_AB(path_yaw[i], L, V, DT)
        A_list.append(A); B_list.append(B)
    return A_list, B_list

def mpc_compute_delta(x_state, idx_near, delta_prev):
    """
    Solve unconstrained time-varying linear MPC (least-squares).
    States: [x, y, yaw], control: delta.
    """
    nx, nu = 3, 1

    # Build references and linearized model over horizon
    Xref = reference_window(idx_near, N, PATH_X, PATH_Y, PATH_YAW)
    A_list, B_list = build_AB_lists(idx_near, N, PATH_YAW)

    # Prediction matrices
    Phi_stack, Gamma = build_timevarying_prediction(PATH_YAW[idx_near:idx_near+N], A_list, B_list)

    # Desired stacked state (x1..xN)
    # Current state influence (Phi_stack @ x0)
    x0 = np.asarray(x_state).reshape(-1,1)  # (nx,1)
    Phi_x0 = (Phi_stack @ x0).reshape(-1,1)

    # Weights
    Q = np.diag([Q_xy, Q_xy, Q_yaw])
    Qf = np.diag([Qf_xy, Qf_xy, Qf_yaw])
    Qbar, Rbar = form_cost_matrices(N, nx, nu, Q, Qf, np.array([[R_steer]]), dR_steer)

    # Solve normal equations:
    #   (Gamma' Qbar Gamma + Rbar) U = Gamma' Qbar (Xref - Phi x0)
    Xref_vec = Xref.reshape(-1,1)
    rhs = Gamma.T.dot(Qbar.dot(Xref_vec - Phi_x0))
    H   = Gamma.T.dot(Qbar.dot(Gamma)) + Rbar

    # Regularize a tad for numerical stability
    H += 1e-8 * np.eye(H.shape[0])

    U = np.linalg.solve(H, rhs)  # (N*nu, 1)
    delta_cmd = float(U[0,0])    # apply first control only (receding horizon)
    return delta_cmd

# ----------------------------
# Simulation state
# ----------------------------
x = PATH_X[0] - 2.0
y = PATH_Y[0] - 3.0
yaw = PATH_YAW[0] + np.deg2rad(30.0)
delta_prev = 0.0

history_x = [x]
history_y = [y]
history_delta = []

# ----------------------------
# Plot / Animation setup
# ----------------------------
plt.close("all")
fig, ax = plt.subplots(figsize=(9, 6))
fig.canvas.manager.set_window_title("MPC (Kinematic) Path Tracking (2D)")

# Full global path (requirements #2 & #4)
global_path_line, = ax.plot(PATH_X, PATH_Y, linestyle='-', linewidth=2.0, alpha=0.6, label="Global Path")

# Vehicle body
veh_poly, = ax.plot([], [], linewidth=2.0, label="Vehicle Body")

# Vehicle heading
heading_line, = ax.plot([], [], linewidth=2.0, linestyle='--', alpha=0.8, label="Vehicle Heading")

# Traveled path
trace_line, = ax.plot([], [], linewidth=2.0, alpha=0.9, label="Traveled Path")

# Telemetry
telemetry = ax.text(0.02, 0.98, "", ha='left', va='top', transform=ax.transAxes, fontsize=10,
                    bbox=dict(boxstyle='round', alpha=0.2, pad=0.4))

# Legend
legend = ax.legend(loc="upper right", framealpha=0.9)

# Axes settings
pad = 10.0
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(np.min(PATH_X) - pad, np.max(PATH_X) + pad)
ax.set_ylim(np.min(PATH_Y) - pad, np.max(PATH_Y) + pad)
ax.set_title("2D Vehicle Path Tracking with MPC (Kinematic Bicycle)")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")

# ----------------------------
# Animation
# ----------------------------
sim_time = 0.0

def init_anim():
    trace_line.set_data([], [])
    veh_poly.set_data([], [])
    heading_line.set_data([], [])
    telemetry.set_text("")
    return global_path_line, trace_line, veh_poly, heading_line, telemetry

def animate(frame_idx):
    global x, y, yaw, delta_prev, sim_time

    # Nearest index for reference window (internal only; not visualized)
    idx = nearest_path_index(x, y, PATH_X, PATH_Y)

    # MPC solve (linearized around reference yaw sequence)
    delta_cmd = mpc_compute_delta([x, y, yaw], idx, delta_prev)

    # Apply steering rate smoothing toward delta_cmd if desired (implicit via dR)
    delta = delta_cmd
    x, y, yaw, delta = step_kinematic(x, y, yaw, V, delta, L, DT)
    delta_prev = delta
    sim_time += DT

    # Bookkeeping
    history_x.append(x)
    history_y.append(y)
    history_delta.append(delta)

    # Draw vehicle rectangle
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

    # Telemetry
    telemetry.set_text(
        "Time: {:.1f} s\nSpeed: {:.1f} m/s\nSteer: {: .1f}°\nHorizon N: {}\nQ=[{:.1f},{:.1f},{:.1f}] R={:.2f} dR={:.2f}".format(
            sim_time, V, np.rad2deg(delta), N, Q_xy, Q_xy, Q_yaw, R_steer, dR_steer
        )
    )

    return global_path_line, trace_line, veh_poly, heading_line, telemetry

# ----------------------------
# Run animation
# ----------------------------
frames = int(TOTAL_TIME / DT)
ani = animation.FuncAnimation(fig, animate, init_func=init_anim, frames=frames,
                              interval=DT * 1000.0, blit=True, repeat=False)

if __name__ == "__main__":
    # Requirement #5: show animation/plot window
    plt.show()
