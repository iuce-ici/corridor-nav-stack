#!/usr/bin/env python3
"""
Phase 2, step 1: consistency check of the EKF predict step. No ROS, no Gazebo.
 
State [x, y, theta, b], b being the gyro bias in rad/s. The predict step uses
the same midpoint heading integration as dead_reckoning_node.cpp, with the
bias estimate subtracted from the gyro reading.
 
What it checks: the filter covariance P is its claim about how large its own
error is. The script propagates P once along the nominal path, then runs
N_RUNS simulated trials, each with its own random bias and gyro noise, records
the actual error (truth minus estimate) at T_END, and compares the spread of
those errors with what P claims.
 
Cases:
  A  heading 0 deg,  bias prior 0.05 deg/s   x excluded: its error is second
                                             order in heading, P says zero
  B  heading 60 deg, bias prior 0.05 deg/s   every term exercised
  C  heading 60 deg, no bias, jitter only    tests Q and the noise Jacobian,
                                             which A and B cannot see
 
Pass: every standard deviation within 5 percent of Monte Carlo, every
correlation within 0.05 and of the same sign.
 
Usage:
  python3 ekf_predict_consistency.py
  python3 ekf_predict_consistency.py --bug theta_b    sign error, bias column
  python3 ekf_predict_consistency.py --bug x_theta    sign error, x on heading
  python3 ekf_predict_consistency.py --bug q          Q four times too large
A correct script passes clean and fails with every bug.
"""
 
import argparse
import sys
import time
 
import numpy as np
 
# Parameters, matched to the Gazebo configuration in the handoff
DT = 0.01                      # s, IMU period at 100 Hz
T_END = 50.0                   # s, 100 m at 2 m/s
V = 2.0                        # m/s, exact: no wheel radius error in this test
SIGMA_W = 1.45e-3              # rad/s per sample, gyro white noise
SIGMA_B = np.deg2rad(0.05)     # rad/s, prior on the gyro bias
N_RUNS = 5000
SEED = 42
 
STD_TOL = 0.05                 # relative, on standard deviations
CORR_TOL = 0.05                # absolute, on correlation coefficients
SIGN_MIN = 0.1                 # sign only compared where |rho| exceeds this
 
NAMES = ["x", "y", "theta", "b"]
 
 
def f(state, w_meas):
    """Discrete process model. Works on one state (4,) or many (N, 4)."""
    x, y, th, b = state[..., 0], state[..., 1], state[..., 2], state[..., 3]
    w = w_meas - b
    th_mid = th + 0.5 * w * DT
    out = np.empty_like(state)
    out[..., 0] = x + V * np.cos(th_mid) * DT
    out[..., 1] = y + V * np.sin(th_mid) * DT
    out[..., 2] = th + w * DT
    out[..., 3] = b
    return out
 
 
def jacobians(state, w_meas, bug):
    """F: derivative of f with respect to the state.
    G: derivative of f with respect to the gyro reading, which carries
    the white noise. Both derived from the discrete update above."""
    th, b = state[2], state[3]
    w = w_meas - b
    th_mid = th + 0.5 * w * DT
    s, c = np.sin(th_mid), np.cos(th_mid)
 
    F = np.eye(4)
    F[0, 2] = -V * s * DT
    F[0, 3] = 0.5 * V * s * DT * DT
    F[1, 2] = V * c * DT
    F[1, 3] = -0.5 * V * c * DT * DT
    F[2, 3] = -DT
 
    G = np.array([-0.5 * V * s * DT * DT,
                  0.5 * V * c * DT * DT,
                  DT,
                  0.0])
 
    if bug == "theta_b":
        F[2, 3] = -F[2, 3]
    if bug == "x_theta":
        F[0, 2] = -F[0, 2]
    return F, G
 
 
def propagate_P(theta0, sigma_b, n_steps, bug):
    """Filter covariance, propagated once along the nominal path:
    bias estimate 0, gyro reading 0, heading constant."""
    q = SIGMA_W ** 2 * (4.0 if bug == "q" else 1.0)
    nominal = np.array([0.0, 0.0, theta0, 0.0])
    P = np.diag([0.0, 0.0, 0.0, sigma_b ** 2])
    for _ in range(n_steps):
        F, G = jacobians(nominal, 0.0, bug)
        P = F @ P @ F.T + q * np.outer(G, G)
        nominal = f(nominal, 0.0)
    return P
 
 
def monte_carlo(theta0, sigma_b, n_steps, rng):
    """N_RUNS trials. Truth drives straight at theta0 with zero yaw rate.
    The estimate integrates the corrupted gyro with bias estimate 0.
    The true bias is used only to corrupt the gyro and to form the error."""
    b_true = rng.normal(0.0, sigma_b, N_RUNS) if sigma_b > 0 else np.zeros(N_RUNS)
 
    truth = np.zeros((N_RUNS, 4))
    truth[:, 2] = theta0
    truth[:, 3] = b_true
 
    est = np.zeros((N_RUNS, 4))
    est[:, 2] = theta0          # initial pose known exactly; bias estimate 0
 
    for _ in range(n_steps):
        noise = rng.normal(0.0, SIGMA_W, N_RUNS)
        w_meas = 0.0 + b_true + noise          # true yaw rate is zero
        truth[:, 0] += V * np.cos(truth[:, 2]) * DT
        truth[:, 1] += V * np.sin(truth[:, 2]) * DT
        est = f(est, w_meas)
 
    return truth - est
 
 
def compare(P, err, excluded):
    """Rows of (label, P value, MC value, difference string, verdict)."""
    C = np.cov(err, rowvar=False)
    sd_P = np.sqrt(np.clip(np.diag(P), 0.0, None))
    sd_M = np.sqrt(np.diag(C))
 
    # display units: m, m, deg, deg/s
    scale = [1.0, 1.0, np.rad2deg(1.0), np.rad2deg(1.0)]
    unit = ["m", "m", "deg", "deg/s"]
 
    rows, ok = [], True
    for i, n in enumerate(NAMES):
        label = f"sigma_{n} [{unit[i]}]"
        if n in excluded:
            rows.append((label, "", "", "", "excluded"))
            continue
        rel = (sd_P[i] - sd_M[i]) / sd_M[i]
        good = abs(rel) <= STD_TOL
        ok &= good
        rows.append((label, f"{sd_P[i] * scale[i]:.5g}", f"{sd_M[i] * scale[i]:.5g}",
                     f"{100 * rel:+.2f} %", "pass" if good else "FAIL"))
 
    for i in range(4):
        for j in range(i + 1, 4):
            label = f"corr({NAMES[i]}, {NAMES[j]})"
            if NAMES[i] in excluded or NAMES[j] in excluded:
                rows.append((label, "", "", "", "excluded"))
                continue
            rp = P[i, j] / (sd_P[i] * sd_P[j])
            rm = C[i, j] / (sd_M[i] * sd_M[j])
            diff = rp - rm
            sign_ok = abs(rm) < SIGN_MIN or np.sign(rp) == np.sign(rm)
            good = abs(diff) <= CORR_TOL and sign_ok
            ok &= good
            rows.append((label, f"{rp:+.4f}", f"{rm:+.4f}", f"{diff:+.4f}",
                         "pass" if good else "FAIL"))
    return rows, ok
 
 
def print_table(title, rows):
    head = ("Quantity", "P claims", "Monte Carlo", "Difference", "Result")
    w = [max(len(str(r[k])) for r in rows + [head]) for k in range(5)]
    line = "  ".join("-" * wk for wk in w)
    print(f"\n{title}")
    print("  ".join(h.ljust(wk) for h, wk in zip(head, w)))
    print(line)
    for r in rows:
        print("  ".join(str(c).ljust(wk) for c, wk in zip(r, w)))
 
 
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bug", choices=["none", "theta_b", "x_theta", "q"], default="none")
    args = ap.parse_args()
 
    # perf_counter is monotonic, so WSL2 wall clock jumps cannot corrupt it
    t0 = time.perf_counter()
    n_steps = int(round(T_END / DT))
 
    print("EKF predict step: P against Monte Carlo")
    print(f"dt {DT} s, T {T_END} s, v {V} m/s, gyro noise {SIGMA_W} rad/s per sample, "
          f"runs {N_RUNS}, seed {SEED}, bug {args.bug}")
 
    cases = [
        ("A", "heading 0 deg, bias prior 0.05 deg/s", 0.0, SIGMA_B, {"x"}),
        ("B", "heading 60 deg, bias prior 0.05 deg/s", 60.0, SIGMA_B, set()),
        ("C", "heading 60 deg, no bias, gyro noise only", 60.0, 0.0, {"b"}),
    ]
 
    all_ok = True
    for k, (tag, desc, heading_deg, sigma_b, excluded) in enumerate(cases):
        theta0 = np.deg2rad(heading_deg)
        rng = np.random.default_rng(SEED + k)
        P = propagate_P(theta0, sigma_b, n_steps, args.bug)
        err = monte_carlo(theta0, sigma_b, n_steps, rng)
        rows, ok = compare(P, err, excluded)
        all_ok &= ok
        print_table(f"Case {tag}: {desc}   ->   {'PASS' if ok else 'FAIL'}", rows)
 
    elapsed = time.perf_counter() - t0
    print(f"\nOverall: {'PASS' if all_ok else 'FAIL'}   ({elapsed:.1f} s)")
    return 0 if all_ok else 1
 
 
if __name__ == "__main__":
    sys.exit(main())