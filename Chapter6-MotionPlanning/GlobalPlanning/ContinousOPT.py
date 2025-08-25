#!/usr/bin/env python3
"""
Minimal 2D continuous path planner on a ribbon track using the geometric-QP idea
r_i = p_i + alpha_i * n_i, where p_i is the centerline, n_i its left-hand normal.

Implements three optimization objectives under inner/outer bounds:
  1) Minimum path length (linearized arc length + small smoothness)
  2) Minimum curvature (approx. integral of kappa^2 along the path)
  3) Minimum time: IRLS on curvature + small length bias; then computes a
     speed profile for a simple "normal car" bicycle-like model with
     lateral accel limit and long. accel/brake caps.

This is a *minimal* educational demo, not a racing-grade optimizer.
Dependencies: numpy, scipy, matplotlib
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# -------------------------------
# Utilities
# -------------------------------

def resample_closed_curve(xy, n_out=420):
    """Resample a closed polyline (Nx2) to ~equal arc-length spacing with n_out samples."""
    pts = np.asarray(xy)
    if np.linalg.norm(pts[0] - pts[-1]) > 1e-9:
        pts = np.vstack([pts, pts[0]])
    seg = pts[1:] - pts[:-1]
    seglen = np.linalg.norm(seg, axis=1)
    s = np.concatenate([[0.0], np.cumsum(seglen)])
    total = s[-1]
    s_out = np.linspace(0.0, total, n_out+1)[:-1]
    x = np.interp(s_out, s, pts[:,0])
    y = np.interp(s_out, s, pts[:,1])
    return np.column_stack([x,y])

def circ_diff_mat(n):
    """Circulant first-difference matrix (wrap): (D1 x)[i] = x[i+1]-x[i]."""
    D = np.zeros((n,n))
    for i in range(n):
        D[i, i] = -1.0
        D[i, (i+1) % n] = 1.0
    return D

def circ_second_diff_mat(n):
    """Circulant second-difference matrix (wrap): x[i+1]-2x[i]+x[i-1]."""
    D2 = np.zeros((n,n))
    for i in range(n):
        D2[i, i] = -2.0
        D2[i, (i+1) % n] = 1.0
        D2[i, (i-1) % n] = 1.0
    return D2

def signed_curvature(poly):
    """Discrete signed curvature kappa and segment lengths ds for a closed curve."""
    p = np.asarray(poly)
    n = len(p)
    fwd = p[(np.arange(n)+1)%n] - p
    bwd = p - p[(np.arange(n)-1)%n]
    ds = 0.5*(np.linalg.norm(fwd,axis=1)+np.linalg.norm(bwd,axis=1)) + 1e-12
    tf = fwd / (np.linalg.norm(fwd,axis=1)[:,None] + 1e-12)
    tb = bwd / (np.linalg.norm(bwd,axis=1)[:,None] + 1e-12)
    cross = tb[:,0]*tf[:,1] - tb[:,1]*tf[:,0]
    dot = (tb*tf).sum(1)
    angle = np.arctan2(cross, dot)
    kappa = angle / ds
    return kappa, ds

def normals(poly):
    """Left-hand unit normals from forward tangents."""
    p = np.asarray(poly)
    fwd = p[(np.arange(len(p))+1)%len(p)] - p
    t = fwd / (np.linalg.norm(fwd,axis=1)[:,None] + 1e-12)
    return np.column_stack([-t[:,1], t[:,0]])

def solve_qp_quadratic_box(H, f, lb, ub, x0=None, maxiter=500):
    """Minimize 0.5 x^T H x + f^T x with simple box constraints using L-BFGS-B."""
    n = len(f)
    if x0 is None:
        x0 = np.clip(np.zeros(n), lb, ub)
    def obj(x):  return 0.5*np.dot(x, H.dot(x)) + np.dot(f, x)
    def grad(x): return H.dot(x) + f
    bounds = [(float(a), float(b)) for a,b in zip(lb,ub)]
    res = minimize(obj, x0, jac=grad, method="L-BFGS-B", bounds=bounds,
                   options=dict(maxiter=maxiter, ftol=1e-9))
    return res.x

def polyline_length(pts):
    d = np.linalg.norm(pts[(np.arange(len(pts))+1)%len(pts)] - pts, axis=1)
    return d.sum()

# -------------------------------
# Simple "normal car" speed model (bicycle-like limits)
# -------------------------------

def min_time_speed_profile(kappa, ds, v_max=60/3.6, ay_max=8.0, ax_max=3.0, ax_min=-6.0):
    """Forward/backward pass respecting lateral and longitudinal accel caps."""
    eps = 1e-6
    v_lat = np.sqrt(np.maximum(eps, ay_max / (np.abs(kappa)+eps)))
    v = np.minimum(v_lat, v_max*np.ones_like(v_lat))
    # forward (accel)
    for i in range(len(v)-1):
        v[i+1] = min(v[i+1], np.sqrt(v[i]**2 + 2*ax_max*ds[i]))
    # backward (brake)
    for i in range(len(v)-2, -1, -1):
        v[i] = min(v[i], np.sqrt(v[i+1]**2 + 2*abs(ax_min)*ds[i]))
    return v

def lap_time(v, ds):
    return np.sum(ds/np.maximum(v, 1e-6))

# -------------------------------
# Track setup with proper inner/outer bounds (variable width)
# -------------------------------

def make_track(Nc, N):
    th = np.linspace(0, 2*np.pi, Nc, endpoint=False)
    Rx, Ry = 14.0, 15.0
    base = np.column_stack([Rx*np.cos(th), Ry*np.sin(th)])
    # add features
    base[:,0] += 2.0*np.sin(3*th)
    base[:,1] += 1.6*np.sin(2*th + 0.7)
    center = resample_closed_curve(base, n_out=N)
    nrm = normals(center)
    kappa_c, ds_seg = signed_curvature(center)
    # variable widths: narrow chicane, wider straights
    phi = np.linspace(0, 2*np.pi, N, endpoint=False)
    wL = 5.0 + 1.2*np.sin(phi + 0.5)
    wR = 4.5 + 1.0*np.sin(2*phi + 1.1)
    mask = (phi>1.4) & (phi<1.9)
    wL[mask] *= 0.5
    wR[mask] *= 0.5
    left_edge  = center + (wL[:,None]*nrm)
    right_edge = center - (wR[:,None]*nrm)
    lb = -wR; ub = wL
    return center, nrm, kappa_c, ds_seg, left_edge, right_edge, lb, ub

# -------------------------------
# Three optimization problems on alpha (offset along normal)
# -------------------------------

def optimize_paths(center, nrm, kappa_c, ds_seg, lb, ub, iters_irls=3):
    N = len(center)
    ds_avg = float(np.mean(ds_seg))
    D1 = circ_diff_mat(N)/ds_avg
    D2 = circ_second_diff_mat(N)/(ds_avg**2)
    K2 = D2  # linear curvature map: kappa_r ≈ kappa_c + K2 @ alpha

    # 1) Min path length (linearized) + smoothness
    H_len = 1.0*(D1.T @ D1)
    c_len = - (ds_seg * kappa_c)
    a_len = solve_qp_quadratic_box(H_len, c_len, lb, ub)

    # 2) Min curvature (∫ kappa_r^2 ds)
    Wds = np.diag(ds_seg)
    H_curv = 25.0 * (K2.T @ Wds @ K2)
    a_curv = solve_qp_quadratic_box(H_curv, np.zeros(N), lb, ub)

    # 3) Min time (IRLS on |kappa| with steering & mild length bias)
    L_wb = 2.7
    delta_max = np.deg2rad(35.0)
    kappa_max_geom = np.tan(delta_max)/L_wb

    a_mt = np.zeros(N)
    for _ in range(iters_irls):
        kappa_lin = kappa_c + K2 @ a_mt
        # IRLS weights: \/ sqrt(|kappa|)
        w_irls = 1.0 / (np.sqrt(np.abs(kappa_lin)) + 1e-3)
        # penalize exceeding steering curvature
        over = np.maximum(0.0, np.abs(kappa_lin) - kappa_max_geom)
        W = np.diag(ds_seg * (w_irls + 10.0*over))
        H_time = (K2.T @ W @ K2) + 0.6*(D1.T @ D1) + 0.1*(D1.T @ D1)
        c_time = -0.2*(ds_seg * kappa_c)
        a_mt = solve_qp_quadratic_box(H_time, c_time, lb, ub, x0=a_mt)

    return a_len, a_curv, a_mt

# -------------------------------
# Demo / Plot
# -------------------------------
if __name__ == "__main__":
    center, nrm, kappa_c, ds_seg, left_edge, right_edge, lb, ub = make_track(Nc=360, N=120)
    a_len, a_curv, a_mt = optimize_paths(center, nrm, kappa_c, ds_seg, lb, ub)

    path_minlen   = center + (a_len [:,None]*nrm)
    path_mincurv  = center + (a_curv[:,None]*nrm)
    path_mintime  = center + (a_mt  [:,None]*nrm)

    # Car/speed model for lap-time metrics
    def metrics(path):
        k, ds = signed_curvature(path)
        v = min_time_speed_profile(k, ds, v_max=60/3.6, ay_max=8.0, ax_max=3.0, ax_min=-6.0)
        return polyline_length(path), lap_time(v, ds), np.max(np.abs(k))

    Lc, Tc, Kc = metrics(center)
    Ll, Tl, Kl = metrics(path_minlen)
    Lk, Tk, Kk = metrics(path_mincurv)
    Lm, Tm, Km = metrics(path_mintime)

    # Plot track & paths
    plt.figure(figsize=(7.8,7.2))
    plt.plot(left_edge[:,0],  left_edge[:,1],  label="Outer bound (left)")
    plt.plot(right_edge[:,0], right_edge[:,1], label="Inner bound (right)")
    plt.plot(center[:,0],     center[:,1],     label="Centerline")
    plt.plot(path_minlen[:,0],  path_minlen[:,1],  linewidth=2.0, label="Min Path")
    plt.plot(path_mincurv[:,0], path_mincurv[:,1], linewidth=2.0, linestyle='--', label="Min Curvature")
    plt.plot(path_mintime[:,0], path_mintime[:,1], linewidth=2.0, linestyle='-.', label="Min Time (IRLS)")
    plt.axis('equal'); plt.xlabel('x [m]'); plt.ylabel('y [m]')
    plt.title('Continuous Planning with Inner/Outer Bounds')
    plt.legend(loc='best'); plt.tight_layout(); plt.show()

    print("== Car params ==")
    print("wheelbase 2.7 m | delta_max 35° -> kappa_max ≈ {:.3f} 1/m".format(np.tan(np.deg2rad(35.0))/2.7))
    print("ay_max=8.0 m/s^2, ax_max=3.0 m/s^2, ax_min=-6.0 m/s^2, v_max=60 km/h")
    print("\n== Path metrics: length [m], lap time [s], max|kappa| [1/m] ==")
    print("Centerline   : {:.1f}, {:.1f}, {:.3f}".format(Lc, Tc, Kc))
    print("Min Path     : {:.1f}, {:.1f}, {:.3f}".format(Ll, Tl, Kl))
    print("Min Curvature: {:.1f}, {:.1f}, {:.3f}".format(Lk, Tk, Kk))
    print("Min Time     : {:.1f}, {:.1f}, {:.3f}".format(Lm, Tm, Km))
