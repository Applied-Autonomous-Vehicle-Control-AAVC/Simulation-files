# linear_mpc_cornering_stiffness.py
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
DT = 0.05             # [s] simulation time step
TOTAL_TIME = 60.0      # [s]
U = 16.0                # [m/s] constant longitudinal speed

# Vehicle physical parameters (typical compact car-ish)
m   = 1500.0           # [kg] mass
Iz  = 2250.0           # [kg m^2] yaw inertia
a   = 1.2              # [m] CG -> front axle
b   = 1.6              # [m] CG -> rear axle
Cf  = 8.0e4            # [N/rad] front cornering stiffness
Cr  = 8.0e4            # [N/rad] rear  cornering stiffness

# Drawing
VEH_LEN = 4.5
VEH_WID  = 1.9
MAX_STEER_DEG = 35.0   # saturation applied to the plant motion only

# MPC (unconstrained, dense least-squares)
N = 20                # horizon steps
Q_ey   = 10.0           # weight on lateral error e_y
Q_epsi = 10.0           # weight on heading error e_psi
Q_vy   = 1.0           # weight on lateral velocity v_y
Q_r    = 1.0           # weight on yaw rate r
Qf_ey, Qf_epsi, Qf_vy, Qf_r = 5.0, 5.0, 1.0, 1.0  # stronger terminal pull

R_delta  = 0.08   # was 0.08 (lets it steer harder)
dR_delta = 0.2    # was 0.2  (still smooth, less sluggish)

# ----------------------------
# Helpers
# ----------------------------
def wrap_to_pi(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi

def make_smooth_global_path(num_pts=1200, span=180.0):
    """
    Generate a smooth, reproducible 'random' 2D path and its curvature.
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

def nearest_path_index(px, py, path_x, path_y):
    dx = path_x - px
    dy = path_y - py
    return int(np.argmin(dx*dx + dy*dy))

def lateral_error(px, py, yaw, path_x, path_y, path_yaw):
    """
    Signed lateral error e_y: positive if path is to the left of vehicle heading.
    """
    i = nearest_path_index(px, py, path_x, path_y)
    dx = path_x[i] - px
    dy = path_y[i] - py
    e_y = dx * (-math.sin(yaw)) + dy * ( math.cos(yaw))
    e_psi = wrap_to_pi(path_yaw[i] - yaw)
    return e_y, e_psi, i

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

# ----------------------------
# Linear bicycle model with cornering stiffness (continuous-time)
# States: x = [e_y, e_psi, v_y, r]^T, Input u = delta (steer)
# e_y_dot   = v_y + U * e_psi
# e_psi_dot = r - U * kappa_ref         (kappa enters as known disturbance)
# v_y_dot   = -(Cf+Cr)/(m U) v_y + (-(a Cf - b Cr)/(m U) - U) r + Cf/m * delta
# r_dot     = -(a Cf - b Cr)/(Iz U) v_y - (a^2 Cf + b^2 Cr)/(Iz U) r + a Cf/Iz * delta
# ----------------------------
A_c = np.array([
    [0.0,     U,   1.0,                         0.0],
    [0.0,   0.0,   0.0,                         1.0],  # (placeholder row; we'll override below)
    [0.0,   0.0, -(Cf+Cr)/(m*U),  (-(a*Cf - b*Cr)/(m*U) - U)],
    [0.0,   0.0, -(a*Cf - b*Cr)/(Iz*U), -(a*a*Cf + b*b*Cr)/(Iz*U)]
], dtype=float)
# Fix e_psi row: e_psi_dot = r + 0*others (kappa will be an affine term)
A_c[1,:] = [0.0, 0.0, 0.0, -1.0]

def g_d_from_kappa(kappa_ref):
    # was: return np.array([0.0, -U*kappa_ref, 0.0, 0.0]) * DT
    return np.array([0.0,  U * kappa_ref, 0.0, 0.0], dtype=float) * DT

B_c = np.array([
    [0.0],
    [0.0],
    [Cf/m],
    [a*Cf/Iz]
], dtype=float)

# Discretize (forward Euler is sufficient for this educational demo)
A_d = np.eye(4) + DT * A_c
B_d = DT * B_c

def g_d_from_kappa(kappa_ref):
    """
    Affine term due to path curvature at the reference:
      e_psi_dot = r - U * kappa_ref  -> g_c = [0, -U*kappa_ref, 0, 0]^T
    Discretized with Euler: g_d = DT * g_c
    """
    return np.array([0.0, -U * kappa_ref, 0.0, 0.0], dtype=float) * DT

# Prediction matrices for time-invariant (A_d, B_d)
def build_prediction_mats(A, B, N):
    nx, nu = A.shape[0], B.shape[1]
    # Phi_stack: [A; A^2; ...; A^N]
    Phi_rows = []
    Ak = np.eye(nx)
    for k in range(1, N+1):
        Ak = Ak @ A
        Phi_rows.append(Ak.copy())
    Phi_stack = np.vstack(Phi_rows)  # (N*nx, nx)

    # Gamma: block lower-triangular with A^(i-1-j) B
    Gamma = np.zeros((N*nx, N*nu))
    A_pows = [np.eye(nx)]
    Ak = np.eye(nx)
    for k in range(1, N+1):
        Ak = Ak @ A
        A_pows.append(Ak.copy())  # A^k
    for i in range(N):          # row block i (for x_{i+1})
        for j in range(i+1):    # u_j influences x_{i+1}
            Aij = A_pows[i-j]
            Gamma[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = Aij @ B
    return Phi_stack, Gamma, A_pows

def build_d_stack(A_pows, g_seq):
    """
    For affine system x_{k+1} = A x_k + B u_k + g_k,
    the stacked offset (for x1..xN) is:
      d_i = sum_{j=0..i} A^{i-j} g_j
    """
    nx = g_seq.shape[1]
    N  = g_seq.shape[0]
    d_list = []
    for i in range(N):
        acc = np.zeros((nx,))
        for j in range(i+1):
            Aij = A_pows[i-j]
            acc += (Aij @ g_seq[j])
        d_list.append(acc)
    return np.concatenate(d_list, axis=0).reshape(N*nx, 1)

def form_cost_matrices(N, nx, nu, Q, Qf, R, dR=None):
    # Qbar (x1..xN)
    Q_blocks = [Q] * (N-1) + [Qf]
    Qbar = np.zeros((N*nx, N*nx))
    for i,Qi in enumerate(Q_blocks):
        Qbar[i*nx:(i+1)*nx, i*nx:(i+1)*nx] = Qi
    # Rbar (u0..u_{N-1})
    Rbar = np.zeros((N*nu, N*nu))
    for i in range(N):
        Rbar[i*nu:(i+1)*nu, i*nu:(i+1)*nu] = R
    # Δu penalty as first-difference (tridiagonal)
    if dR is not None and dR > 0.0 and N > 1:
        for i in range(N):
            Rbar[i, i] += dR
            if i > 0:
                Rbar[i, i-1] += -dR
                Rbar[i-1, i] += -dR
                Rbar[i-1, i-1] += dR
    return Qbar, Rbar

# ----------------------------
# Build the global path
# ----------------------------
PATH_X, PATH_Y, PATH_YAW, PATH_S, PATH_KAPPA = make_smooth_global_path()

# Pre-build time-invariant prediction pieces
Phi_stack, Gamma, A_pows = build_prediction_mats(A_d, B_d, N)

# Weights
Q = np.diag([Q_ey, Q_epsi, Q_vy, Q_r])
Qf = np.diag([Qf_ey, Qf_epsi, Qf_vy, Qf_r])
R = np.array([[R_delta]])

# ----------------------------
# Plant (for visualization) state
# ----------------------------
# Global pose & dynamic states
xg = PATH_X[0] - 2.0
yg = PATH_Y[0] - 3.0
yaw = PATH_YAW[0] + np.deg2rad(30.0)
v_y = 0.0
r   = 0.0
delta_prev = 0.0

history_x, history_y = [xg], [yg]
history_delta = []

# ----------------------------
# Plot / Animation setup
# ----------------------------
plt.close("all")
fig, ax = plt.subplots(figsize=(9, 6))
fig.canvas.manager.set_window_title("Linear MPC (Cornering Stiffness) Path Tracking")

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

# Axes
pad = 10.0
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(np.min(PATH_X) - pad, np.max(PATH_X) + pad)
ax.set_ylim(np.min(PATH_Y) - pad, np.max(PATH_Y) + pad)
ax.set_title("2D Path Tracking with Linear MPC (Linear Tire Model)")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")

# ----------------------------
# MPC solver (unconstrained)
# ----------------------------
def solve_lti_mpc(x_state, kappa_seq):
    """
    x_state: current error state [e_y, e_psi, v_y, r]
    kappa_seq: array length N with future curvature (reference) values
    Returns first steering command delta (float).
    """
    nx, nu = 4, 1

    # Build affine offsets sequence g_k from κ_ref
    gseq = np.zeros((N, nx))
    for i in range(N):
        kappa = kappa_seq[min(i, len(kappa_seq)-1)]
        gseq[i, :] = g_d_from_kappa(kappa)

    d_stack = build_d_stack(A_pows, gseq)

    x0 = np.asarray(x_state).reshape(nx, 1)
    Phi_x0 = (Phi_stack @ x0)

    # Cost matrices
    Qbar, Rbar = form_cost_matrices(N, nx, nu, Q, Qf, R, dR_delta)

    # Solve: (Gamma' Qbar Gamma + Rbar) U = Gamma' Qbar (Xref - Phi x0 - d)
    Xref = np.zeros((N*nx, 1))  # track zero errors
    rhs = Gamma.T.dot(Qbar.dot(Xref - Phi_x0 - d_stack))
    H   = Gamma.T.dot(Qbar.dot(Gamma)) + Rbar

    # Small regularization for numeric stability
    H += 1e-8 * np.eye(H.shape[0])

    U = np.linalg.solve(H, rhs)  # shape (N,1)
    return float(U[0,0])

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
    global xg, yg, yaw, v_y, r, delta_prev, sim_time

    # --- Build current error state relative to path (internal; not visualized) ---
    e_y, e_psi, idx = lateral_error(xg, yg, yaw, PATH_X, PATH_Y, PATH_YAW)

    # Curvature sequence for the horizon (feedforward)
    kappa_seq = PATH_KAPPA[idx : idx + N]
    if kappa_seq.size < N:
        # pad with last value
        if kappa_seq.size == 0:
            kappa_seq = np.zeros((N,))
        else:
            kappa_seq = np.pad(kappa_seq, (0, N - kappa_seq.size), mode='edge')

    x_err = np.array([e_y, e_psi, v_y, r])

    # --- MPC solve for steering ---
    delta_cmd = solve_lti_mpc(x_err, kappa_seq)

    # --- Apply to plant (saturate physically), then integrate vehicle motion ---
    max_steer = np.deg2rad(MAX_STEER_DEG)
    delta = float(np.clip(delta_cmd, -max_steer, max_steer))

    # Linear tire lateral dynamics (continuous) to update v_y and r
    vy_dot = -(Cf+Cr)/(m*U) * v_y + (-(a*Cf - b*Cr)/(m*U) - U) * r + (Cf/m) * delta
    r_dot  = -(a*Cf - b*Cr)/(Iz*U) * v_y - (a*a*Cf + b*b*Cr)/(Iz*U) * r + (a*Cf/Iz) * delta

    v_y += vy_dot * DT
    r   += r_dot  * DT
    yaw = wrap_to_pi(yaw + r * DT)

    # Global pose integration using body velocities
    xg += (U * math.cos(yaw) - v_y * math.sin(yaw)) * DT
    yg += (U * math.sin(yaw) + v_y * math.cos(yaw)) * DT

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

    # Telemetry
    telemetry.set_text(
        "Time: {:.1f} s\nSpeed U: {:.1f} m/s\nSteer: {: .1f}°\n"
        "e_y: {: .2f} m  e_ψ: {: .2f} rad\nv_y: {: .2f} m/s  r: {: .2f} rad/s\nN: {}  R: {:.2f}  dR: {:.2f}".format(
            sim_time, U, np.rad2deg(delta), e_y, e_psi, v_y, r, N, R_delta, dR_delta
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
