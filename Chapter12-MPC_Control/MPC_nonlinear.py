# nonlinear_mpc_path_tracking.py
# Python 3.6–3.9 compatible
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation

# ----------------------------
# Global / Simulation settings
# ----------------------------
SEED = 7               # reproducible global path
DT = 0.05              # [s] simulation time step
TOTAL_TIME = 60.0      # [s]
U = 16.0               # [m/s] constant longitudinal speed

# Vehicle geometric parameters (bicycle)
m   = 1500.0           # [kg] (not used by kinematic model; kept for completeness)
Iz  = 2250.0           # [kg m^2]
a   = 1.2              # [m] CG -> front axle
b   = 1.6              # [m] CG -> rear axle
L   = a + b            # [m] wheelbase

# Drawing
VEH_LEN = 4.5
VEH_WID  = 1.9
MAX_STEER_DEG = 35.0   # saturation applied to the plant motion only

# NMPC (successive linearization; dense least-squares)
N = 20                 # horizon steps
GN_ITERS = 2           # Gauss-Newton iterations per control step
Q_pos = 8.0            # weight on x,y position error
Q_yaw = 6.0            # weight on heading error
Qf_pos = 12.0          # terminal weights
Qf_yaw = 10.0
R_delta  = 0.08        # steering effort
dR_delta = 0.20        # smoothness (penalize Δu)

# ----------------------------
# Helpers
# ----------------------------
def wrap_to_pi(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi

def make_smooth_global_path(num_pts=1200, span=180.0):
    """
    Generate a smooth, reproducible 'random' 2D path and its curvature + s.
    Returns: x, y, yaw, s, kappa
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
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)

    yaw = np.arctan2(dy, dx)
    ds = np.hypot(dx, dy)
    s = np.cumsum(ds); s -= s[0]

    # Curvature κ = (x' y'' - y' x'') / (x'^2 + y'^2)^(3/2)
    denom = (dx*dx + dy*dy)**1.5 + 1e-12
    kappa = (dx * ddy - dy * ddx) / denom

    return x, y, yaw, s, kappa

def vehicle_rectangle(x, y, yaw, length, width):
    cx = x + (length / 2.0) * math.cos(yaw)
    cy = y + (length / 2.0) * math.sin(yaw)
    hl = length / 2.0
    hw = width / 2.0
    pts = np.array([
        [ hl,  hw], [ hl, -hw], [-hl, -hw], [-hl,  hw], [ hl,  hw]
    ])
    R = np.array([[ math.cos(yaw), -math.sin(yaw)],
                  [ math.sin(yaw),  math.cos(yaw)]])
    world = (R @ pts.T).T
    world[:, 0] += cx; world[:, 1] += cy
    return world

def nearest_path_index(px, py, path_x, path_y):
    dx = path_x - px
    dy = path_y - py
    return int(np.argmin(dx*dx + dy*dy))

def search_by_s(s_query, PATH_S):
    # returns index i such that PATH_S[i] >= s_query (clamped to valid range)
    i = int(np.searchsorted(PATH_S, s_query, side='left'))
    if i >= len(PATH_S):
        i = len(PATH_S) - 1
    return i

# ----------------------------
# Global path (requirements #2 & #4)
# ----------------------------
PATH_X, PATH_Y, PATH_YAW, PATH_S, PATH_KAPPA = make_smooth_global_path()

# ----------------------------
# Nonlinear (kinematic bicycle) plant and linearization
# ----------------------------
def f_discrete(x, u):
    """
    x = [x, y, psi], u = [delta]
    Discrete-time kinematic bicycle with forward Euler and constant U.
    """
    X, Y, PSI = x
    delta = float(u)
    # dynamics
    Xn   = X   + DT * U * math.cos(PSI)
    Yn   = Y   + DT * U * math.sin(PSI)
    PSIn = wrap_to_pi(PSI + DT * (U / L) * math.tan(delta))
    return np.array([Xn, Yn, PSIn], dtype=float)

def linearize_discrete(x, u):
    """
    Returns A_k = ∂f/∂x, B_k = ∂f/∂u at (x,u) for the discretized system.
    """
    X, Y, PSI = x
    delta = float(u)

    # df/dx
    dX_dpsi = -DT * U * math.sin(PSI)
    dY_dpsi =  DT * U * math.cos(PSI)
    A = np.array([
        [1.0, 0.0, dX_dpsi],
        [0.0, 1.0, dY_dpsi],
        [0.0, 0.0, 1.0]
    ], dtype=float)

    # df/du
    sec2 = 1.0 / (math.cos(delta)**2 + 1e-12)  # sec^2(delta)
    dpsi_ddelta = DT * (U / L) * sec2
    B = np.array([[0.0], [0.0], [dpsi_ddelta]], dtype=float)
    return A, B

# ----------------------------
# Build references along s (no "closest point" visualization)
# ----------------------------
def build_ref_sequence(cur_idx, N):
    """
    Using path arclength, pick N future reference poses spaced by expected
    travel per step (U*DT). This avoids "closest-point" visualization.
    """
    ds_ahead = U * DT
    xref = np.zeros((N, 3))
    for k in range(N):
        s_target = PATH_S[cur_idx] + ds_ahead * (k + 1)
        i = search_by_s(s_target, PATH_S)
        xref[k, 0] = PATH_X[i]
        xref[k, 1] = PATH_Y[i]
        xref[k, 2] = PATH_YAW[i]
    return xref

# ----------------------------
# Cost helpers (stacked)
# ----------------------------
def form_Qbar_Rbar(N, qpos, qpsi, qfpos, qfyaw, r, dr):
    nx = 3
    Qbar = np.zeros((N*nx, N*nx))
    # stage
    Qstage = np.diag([qpos, qpos, qpsi])
    # terminal
    Qterm  = np.diag([qfpos, qfpos, qfyaw])
    for i in range(N-1):
        Qbar[i*nx:(i+1)*nx, i*nx:(i+1)*nx] = Qstage
    Qbar[(N-1)*nx:N*nx, (N-1)*nx:N*nx] = Qterm

    # input terms (Δu smoothing like a tridiagonal)
    Rbar = np.zeros((N, N))
    for i in range(N):
        Rbar[i, i] += r + dr
        if i > 0:
            Rbar[i, i-1] += -dr
            Rbar[i-1, i] += -dr
            Rbar[i-1, i-1] += dr
    return Qbar, Rbar

def build_Su(A_list, B_list):
    """
    For time-varying linear model:
      δx_{k+1} = A_k δx_k + B_k δu_k
    Build Su s.t. [δx1; δx2; ...; δxN] = Su * [δu0; ...; δu_{N-1}]
    """
    nx = A_list[0].shape[0]
    nu = B_list[0].shape[1]
    N = len(A_list)
    Su = np.zeros((N*nx, N*nu))
    # Precompute state transition products
    Phi = [np.eye(nx)]
    for k in range(N):
        Phi.append(A_list[k].dot(Phi[-1]))
    # Fill Su
    for i in range(N):          # for δx_{i+1}
        for j in range(i+1):    # u_j influences up to i
            Aprod = np.eye(nx)
            for t in range(j+1, i+1):
                Aprod = A_list[t-1].dot(Aprod)
            Su[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = Aprod.dot(B_list[j])
    return Su

# ----------------------------
# NMPC solve (one step): Gauss–Newton over horizon
# ----------------------------
def nmpc_solve_one_step(x0, u_prev, idx_near):
    """
    x0      : current state [x,y,psi]
    u_prev  : previous applied steering (for warm-start)
    idx_near: nearest index on path at current pose (internal only)
    Returns: (u0, pred_states_Nx3) first control and predicted states
    """
    nx, nu = 3, 1
    # warm start control sequence
    U_seq = np.full((N, 1), float(u_prev))
    # nominal rollout states
    X_seq = np.zeros((N, nx))
    # references
    Xref = build_ref_sequence(idx_near, N)

    for _ in range(GN_ITERS):
        # 1) Rollout with current U_seq
        x = np.array(x0, dtype=float)
        for k in range(N):
            x = f_discrete(x, U_seq[k, 0])
            X_seq[k, :] = x

        # 2) Linearize along nominal
        A_list, B_list = [], []
        x = np.array(x0, dtype=float)
        for k in range(N):
            A, B = linearize_discrete(x, U_seq[k, 0])
            A_list.append(A); B_list.append(B)
            x = f_discrete(x, U_seq[k, 0])

        # 3) Build stacked matrices
        Su = build_Su(A_list, B_list)               # (N*nx, N*nu)
        X_stack = X_seq.reshape(N*nx, 1)
        Xref_stack = Xref.reshape(N*nx, 1)

        Qbar, Rbar = form_Qbar_Rbar(
            N, Q_pos, Q_yaw, Qf_pos, Qf_yaw, R_delta, dR_delta
        )

        # 4) Solve normal equations for δU (Gauss–Newton)
        # minimize 0.5|| Q^(1/2) (Xnom + Su δU - Xref) ||^2 + 0.5|| R^(1/2) (Unom + δU) ||^2
        rhs = Su.T.dot(Qbar.dot(Xref_stack - X_stack)) - Rbar.dot(U_seq)
        H   = Su.T.dot(Qbar.dot(Su)) + Rbar
        H += 1e-8 * np.eye(H.shape[0])  # tiny regularization
        dU  = np.linalg.solve(H, rhs)
        U_seq = U_seq + dU  # update nominal

        # steer saturation (keep reasonable during iterations)
        max_steer = np.deg2rad(MAX_STEER_DEG)
        U_seq = np.clip(U_seq, -max_steer, max_steer)

    # Final rollout for visualization
    x = np.array(x0, dtype=float)
    for k in range(N):
        x = f_discrete(x, U_seq[k, 0])
        X_seq[k, :] = x

    return float(U_seq[0, 0]), X_seq

# ----------------------------
# Plant / state (global pose)
# ----------------------------
xg = PATH_X[0] - 2.0
yg = PATH_Y[0] - 3.0
yaw = PATH_YAW[0] + np.deg2rad(30.0)
delta_prev = 0.0

history_x, history_y = [xg], [yg]
history_delta = []

# ----------------------------
# Plot / Animation setup
# ----------------------------
plt.close("all")
fig, ax = plt.subplots(figsize=(9, 6))
fig.canvas.manager.set_window_title("Nonlinear MPC Path Tracking (Kinematic Bicycle)")

# Full global path (requirements #2 & #4)
global_path_line, = ax.plot(PATH_X, PATH_Y, linestyle='-', linewidth=2.0, alpha=0.6, label="Global Path")

# MPC predicted horizon (updated every frame)
pred_line, = ax.plot([], [], linestyle=':', linewidth=2.0, alpha=0.9, label="MPC Prediction")

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

# Axes
pad = 10.0
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(np.min(PATH_X) - pad, np.max(PATH_X) + pad)
ax.set_ylim(np.min(PATH_Y) - pad, np.max(PATH_Y) + pad)
ax.set_title("2D Path Tracking with Nonlinear MPC")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")

# ----------------------------
# Animation
# ----------------------------
sim_time = 0.0

def init_anim():
    trace_line.set_data([], [])
    pred_line.set_data([], [])
    veh_poly.set_data([], [])
    heading_line.set_data([], [])
    telemetry.set_text("")
    return global_path_line, pred_line, trace_line, veh_poly, heading_line, telemetry

def animate(frame_idx):
    global xg, yg, yaw, delta_prev, sim_time

    # --- Internal: nearest index (not visualized) ---
    idx = nearest_path_index(xg, yg, PATH_X, PATH_Y)

    # --- NMPC (nonlinear) solve for steering and get prediction ---
    x0 = np.array([xg, yg, yaw])
    delta_cmd, Xpred = nmpc_solve_one_step(x0, delta_prev, idx)

    # --- Apply to plant (saturate), then integrate kinematic motion ---
    max_steer = np.deg2rad(MAX_STEER_DEG)
    delta = float(np.clip(delta_cmd, -max_steer, max_steer))

    # Kinematic update (discrete)
    xg = xg + DT * U * math.cos(yaw)
    yg = yg + DT * U * math.sin(yaw)
    yaw = wrap_to_pi(yaw + DT * (U / L) * math.tan(delta))

    sim_time += DT
    delta_prev = delta

    # --- Bookkeeping for visualization ---
    history_x.append(xg); history_y.append(yg); history_delta.append(delta)

    # Vehicle rectangle
    poly = vehicle_rectangle(xg, yg, yaw, VEH_LEN, VEH_WID)
    veh_poly.set_data(poly[:, 0], poly[:, 1])

    # Heading line
    head_len = VEH_LEN * 0.8
    hx0, hy0 = xg, yg
    hx1 = xg + head_len * math.cos(yaw)
    hy1 = yg + head_len * math.sin(yaw)
    heading_line.set_data([hx0, hx1], [hy0, hy1])

    # Traveled path
    trace_line.set_data(history_x, history_y)

    # MPC predicted horizon (for legend & intuition)
    pred_line.set_data(Xpred[:, 0], Xpred[:, 1])

    # Telemetry
    epsi = wrap_to_pi(PATH_YAW[idx] - yaw)
    ex = PATH_X[idx] - xg
    ey = PATH_Y[idx] - yg
    telemetry.set_text(
        "Time: {:.1f} s\nSpeed U: {:.1f} m/s\nSteer: {: .1f}°\n"
        "ex: {: .2f} m  ey: {: .2f} m  eψ: {: .2f} rad\n"
        "N: {}  R: {:.2f}  dR: {:.2f}  GN iters: {}".format(
            sim_time, U, np.rad2deg(delta), ex, ey, epsi, N, R_delta, dR_delta, GN_ITERS
        )
    )

    return global_path_line, pred_line, trace_line, veh_poly, heading_line, telemetry

# ----------------------------
# Run animation
# ----------------------------
frames = int(TOTAL_TIME / DT)
ani = animation.FuncAnimation(fig, animate, init_func=init_anim, frames=frames,
                              interval=DT * 1000.0, blit=True, repeat=False)

if __name__ == "__main__":
    # Requirement #5: show animation/plot window
    plt.show()
