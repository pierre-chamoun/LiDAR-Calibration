#!/usr/bin/env python3
import argparse
from pathlib import Path

import matplotlib.lines as mlines
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

DEG = 180.0 / np.pi


def skew(w):
    return np.array([[0.0, -w[2], w[1]],
                      [w[2], 0.0, -w[0]],
                      [-w[1], w[0], 0.0]])


def so3_left_jacobian(omega):
    theta = np.linalg.norm(omega)
    K = skew(omega)
    if theta < 1e-8:
        return np.eye(3) + 0.5 * K + (K @ K) / 6.0
    return (np.eye(3)
            + (1 - np.cos(theta)) / theta**2 * K
            + (theta - np.sin(theta)) / theta**3 * (K @ K))


def expmap_se3(xi):
    omega, v = xi[:3], xi[3:]
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(omega).as_matrix()
    T[:3, 3] = so3_left_jacobian(omega) @ v
    return T


def logmap_se3(T):
    R, t = T[:3, :3], T[:3, 3]
    omega = Rotation.from_matrix(R).as_rotvec()
    v = np.linalg.solve(so3_left_jacobian(omega), t)
    return np.concatenate([omega, v])


def pose_matrix(roll, pitch, yaw, tx, ty, tz):
    # Matches GTSAM's Rot3::roll()/pitch()/yaw(): R = Rz(yaw) * Ry(pitch) * Rx(roll).
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    T[:3, 3] = [tx, ty, tz]
    return T


def transport_covariance(C, G, eps=1e-6):
    """Sigma_xi = J C J^T, for xi = Log(Exp(d) @ G) with d ~ N(0, C).

    J = d(xi)/d(d) at d=0, obtained by central finite differences rather than
    GTSAM's closed-form Adjoint/LogmapDerivative, so this stays a pure
    numpy/scipy implementation.
    """
    J = np.zeros((6, 6))
    d = np.zeros(6)
    for i in range(6):
        d[i] = eps
        xi_p = logmap_se3(expmap_se3(d) @ G)
        d[i] = -eps
        xi_m = logmap_se3(expmap_se3(d) @ G)
        d[i] = 0.0
        J[:, i] = (xi_p - xi_m) / (2 * eps)
    return J @ C @ J.T


def upper_tri_to_sym(row):
    C = np.zeros((6, 6))
    for i in range(6):
        for j in range(i, 6):
            C[i, j] = C[j, i] = row[f"c{i}{j}"]
    return C


def info_weighted_mean(estimates, T_init, max_iter=100, tol=1e-8):
    """Info-weighted Frechet mean on SE(3) (left perturbation), matching
    montecarlo_utils.cpp's infoWeightedMean:
        delta = (sum H_i)^{-1} sum H_i log(T_i * T^{-1})
        T <- Exp(delta) * T
    `estimates` is a list of (T_i, info_i) 4x4/6x6 numpy arrays.
    """
    T = T_init.copy()
    for _ in range(max_iter):
        H_sum = np.zeros((6, 6))
        Hb = np.zeros(6)
        for T_i, info_i in estimates:
            xi = logmap_se3(T_i @ np.linalg.inv(T))
            H_sum += info_i
            Hb += info_i @ xi
        delta = np.linalg.solve(H_sum, Hb)
        T = expmap_se3(delta) @ T
        if np.linalg.norm(delta) < tol:
            break
    return T


parser = argparse.ArgumentParser()
parser.add_argument("output_dir", nargs="?", default="output")
parser.add_argument("--out", default="output/recursive_relinearized.pdf")
parser.add_argument("--lin", nargs=6, type=float,
                     metavar=("ROLL_DEG", "PITCH_DEG", "YAW_DEG", "TX", "TY", "TZ"),
                     default=None,
                     help="Linearization point to report the estimate/uncertainty against. "
                          "Defaults to the info-weighted Frechet mean of each dataset's final "
                          "(last-batch) fused estimate.")
args = parser.parse_args()

csv_files = sorted(Path(args.output_dir).rglob("recursive_estimate_*.csv"))
if not csv_files:
    raise SystemExit(f"No recursive_estimate_*.csv found under {args.output_dir}")

dfs = [pd.read_csv(p) for p in csv_files]
labels = [p.stem[len("recursive_estimate_"):] for p in csv_files]

if args.lin is not None:
    roll0, pitch0, yaw0, tx0, ty0, tz0 = args.lin
    T_lin = pose_matrix(np.deg2rad(roll0), np.deg2rad(pitch0), np.deg2rad(yaw0), tx0, ty0, tz0)
else:
    last_estimates = []
    for df in dfs:
        last = df.iloc[-1]
        T_last = pose_matrix(np.deg2rad(last["roll_deg"]), np.deg2rad(last["pitch_deg"]),
                              np.deg2rad(last["yaw_deg"]), last["tx"], last["ty"], last["tz"])
        info_last = np.linalg.inv(upper_tri_to_sym(last))
        last_estimates.append((T_last, info_last))
    T_lin = info_weighted_mean(last_estimates, last_estimates[0][0])
    roll_lin, pitch_lin, yaw_lin = Rotation.from_matrix(T_lin[:3, :3]).as_euler("xyz")
    print("Linearization point (info-weighted Frechet mean of final estimates): "
          f"roll={roll_lin*DEG:.4f} pitch={pitch_lin*DEG:.4f} yaw={yaw_lin*DEG:.4f} deg, "
          f"tx={T_lin[0,3]:.4f} ty={T_lin[1,3]:.4f} tz={T_lin[2,3]:.4f} m")
T_lin_inv = np.linalg.inv(T_lin)

# GTSAM Pose3 tangent ordering: [omega_x, omega_y, omega_z, v_x, v_y, v_z].
# Plot order: rotation first (idx 0,1,2), then translation (idx 3,4,5).
plot_idx = [0, 1, 2, 3, 4, 5]
scale = [DEG, DEG, DEG, 100, 100, 100]
ylabels = [r"$\delta\xi^{*}_{\phi_1}$ (deg)", r"$\delta\xi^{*}_{\phi_2}$ (deg)", r"$\delta\xi^{*}_{\phi_3}$ (deg)",
           r"$\delta\xi^{*}_x$ (cm)", r"$\delta\xi^{*}_y$ (cm)", r"$\delta\xi^{*}_z$ (cm)"]

series = []  # per file: (n, e[N,6], sig[N,6]) relative to the linearization point
for df in dfs:
    e_all = np.zeros((len(df), 6))
    sig_all = np.zeros((len(df), 6))
    for row_idx, row in df.iterrows():
        T_fused = pose_matrix(np.deg2rad(row["roll_deg"]), np.deg2rad(row["pitch_deg"]),
                               np.deg2rad(row["yaw_deg"]), row["tx"], row["ty"], row["tz"])
        G = T_fused @ T_lin_inv
        e_all[row_idx] = logmap_se3(G)
        Sigma_e = transport_covariance(upper_tri_to_sym(row), G)
        sig_all[row_idx] = 3.0 * np.sqrt(np.clip(np.diag(Sigma_e), 0, None))
    series.append((df["n"].to_numpy(), e_all, sig_all))

# Plots

plt.rc('text', usetex=True)
plt.rc('font', family='serif', size=8)
plt.rc('lines', linewidth=0.5)
plt.rc('axes', linewidth=0.5)
plt.rc('xtick.major', width=0.5)
plt.rc('ytick.major', width=0.5)
plt.rc('grid', linewidth=0.5, linestyle=':')
fig, axes = plt.subplots(2, 3, figsize=(7.16, 4.0))  


OKABE_ITO = ["#E69F00", "#0072B2", "#009E73", "#D55E00",
             "#CC79A7", "#56B4E9", "#F0E442", "#000000"]

colors = [OKABE_ITO[i % len(OKABE_ITO)] for i in range(len(labels))]

for ax, idx, ylabel, sc in zip(axes.flat, plot_idx, ylabels, scale):
    for (n, e_all, sig_all), label, color in zip(series, labels, colors):
        est = e_all[:, idx] * sc
        sig = sig_all[:, idx] * sc

        ax.plot(n, est, color=color, marker="o", markersize=1, label=label)
        ax.fill_between(n, est - sig, est + sig, alpha=0.2, color=color)
        ax.plot(n, est + sig, color=color, alpha=0.4)
        ax.plot(n, est - sig, color=color, alpha=0.4)
        
    ax.set_xlabel("$N$")
    ax.set_ylabel(ylabel)
    ax.grid(True)


handles, legend_labels = axes.flat[0].get_legend_handles_labels()
leg = axes.flat[2].legend(handles, legend_labels, loc="upper right", fontsize=7,
                           markerscale=0.6, handlelength=1, labelspacing=0.2, borderpad=0.5)
leg.get_frame().set_linewidth(0.5)

plt.tight_layout()
Path(args.out).parent.mkdir(parents=True, exist_ok=True)
plt.savefig(args.out, dpi=1200)
print(f"Saved {args.out}")
