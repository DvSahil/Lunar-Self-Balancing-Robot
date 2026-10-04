"""
Wheeled inverted pendulum (cart-pole) simulation.

Compares three controllers on the same robot, with the same push at t = 1 s:
  1. PD on tilt only            (what the firmware does with 'outer 0')
  2. PD + outer speed/position  (what the firmware does with 'outer 1')
  3. LQR full-state feedback    (the "next level" upgrade)

Everything here is SIMULATION. The parameters below are example values for a small
robot - replace them with YOUR measured values (mass, centre-of-mass height, wheel
radius, motor torque) before drawing any conclusion about your hardware.

Run:  python pendulum_sim.py        (saves output/sim_comparison.png)
"""
import os
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ----------------------------------------------------------------- robot parameters (EDIT)
M = 0.20          # kg   wheels + motors (the part that translates)
m = 0.50          # kg   body: chassis, battery, electronics
l = 0.08          # m    height of body centre of mass above the wheel axle
h = 0.16          # m    body height (only used for its inertia)
I = m * h**2 / 12 # kg m^2 body inertia about its own centre of mass (uniform bar)
b = 0.05          # N s/m viscous friction of the drivetrain
g = 9.81
F_MAX = 15.0      # N    max total push at the ground (2 x motor torque / wheel radius)


def accelerations(theta, theta_d, x_d, F, F_dist=0.0):
    """Nonlinear cart-pole. theta = tilt from upright, + = leaning forward (+x)."""
    mass = np.array([[M + m, m * l * np.cos(theta)],
                     [m * l * np.cos(theta), I + m * l**2]])
    rhs = np.array([F - b * x_d + m * l * theta_d**2 * np.sin(theta) + F_dist,
                    m * g * l * np.sin(theta) + F_dist * l * np.cos(theta)])
    return np.linalg.solve(mass, rhs)          # [x_dd, theta_dd]


def linear_model():
    """Linearise around upright: state [x, x_d, theta, theta_d], input F."""
    Mm = np.array([[M + m, m * l], [m * l, I + m * l**2]])
    Minv = np.linalg.inv(Mm)
    A = np.zeros((4, 4)); B = np.zeros((4, 1))
    A[0, 1] = 1.0; A[2, 3] = 1.0
    A[1:4:2, 1] = Minv @ np.array([-b, 0.0])
    A[1:4:2, 2] = Minv @ np.array([0.0, m * g * l])
    B[1:4:2, 0] = Minv @ np.array([1.0, 0.0])
    return A, B


A, B = linear_model()
Q = np.diag([5.0, 1.0, 200.0, 1.0])
R = np.array([[0.05]])
P_are = solve_continuous_are(A, B, Q, R)
K_LQR = (np.linalg.inv(R) @ B.T @ P_are).flatten()      # F = -K s

# PD gains (N per rad, N s per rad). Same structure as kp / kd in the firmware.
KP, KD = 60.0, 3.0
# Outer loop: angle offset (rad) = -(VP * speed_error + VI * position_error)
VP, VI = 0.10, 0.05


def controller(kind):
    def f(t, s):
        x, xd, th, thd = s
        if kind == "pd":
            F = KP * th + KD * thd
        elif kind == "cascade":
            th_ref = -(VP * xd + VI * x)
            F = KP * (th - th_ref) + KD * thd
        else:
            F = -K_LQR @ s
        return float(np.clip(F, -F_MAX, F_MAX))
    return f


def push(t):
    return 6.0 if 1.0 <= t < 1.05 else 0.0            # 6 N shove on the body for 50 ms


def simulate(kind, t_end=6.0):
    ctrl = controller(kind)

    def rhs(t, s):
        F = ctrl(t, s)
        xdd, thdd = accelerations(s[2], s[3], s[1], F, push(t))
        return [s[1], xdd, s[3], thdd]

    sol = solve_ivp(rhs, (0, t_end), [0, 0, np.deg2rad(2.0), 0], max_step=0.002, rtol=1e-7)
    F = np.array([ctrl(t, s) for t, s in zip(sol.t, sol.y.T)])
    return sol.t, sol.y, F


if __name__ == "__main__":
    print("Open-loop poles (one is positive = unstable):", np.round(np.linalg.eigvals(A), 2))
    names = {"pd": "PD (tilt only)", "cascade": "PD + speed/position loop", "lqr": "LQR"}
    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for kind in ("pd", "cascade", "lqr"):
        t, y, F = simulate(kind)
        ax[0].plot(t, np.rad2deg(y[2]), label=names[kind])
        ax[1].plot(t, y[0], label=names[kind])
        ax[2].plot(t, F, label=names[kind])
        print(f"{names[kind]:28s} max tilt {np.max(np.abs(np.rad2deg(y[2]))):5.1f} deg | "
              f"final tilt {np.rad2deg(y[2][-1]):6.2f} deg | final position {y[0][-1]:6.2f} m")
    ax[0].set_ylabel("tilt (deg)"); ax[1].set_ylabel("position (m)"); ax[2].set_ylabel("wheel force (N)")
    ax[2].set_xlabel("time (s)")
    for a in ax:
        a.grid(alpha=0.3); a.axvline(1.0, color="k", lw=0.6, ls=":")
    ax[0].legend(); ax[0].set_title("Simulated 6 N push at t = 1 s (SIMULATION, not hardware data)")
    os.makedirs("output", exist_ok=True)
    fig.tight_layout(); fig.savefig("output/sim_comparison.png", dpi=130)
    print("saved output/sim_comparison.png")
