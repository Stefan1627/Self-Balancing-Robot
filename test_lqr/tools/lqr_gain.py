# tools/lqr_gain.py

import numpy as np
from scipy.signal import cont2discrete
from scipy.linalg import solve_discrete_are

# ============================================================
# MEASURE THESE ON YOUR REAL ROBOT
# ============================================================

g = 9.81

# Total robot mass in kg.
m = 1.20

# Distance from wheel axle to center of gravity in meters.
# Measure with battery and all electronics mounted.
l = 0.080

# Firmware control loop period.
dt = 0.005  # 200 Hz

# ============================================================
# STATE MODEL
#
# State vector:
#   x = [theta, theta_dot, position, velocity]
#
# theta:
#   pitch angle from vertical, radians
#
# input:
#   approximate horizontal force / effort
# ============================================================

A = np.array([
    [0.0,   1.0, 0.0, 0.0],
    [g/l,   0.0, 0.0, 0.0],
    [0.0,   0.0, 0.0, 1.0],
    [-g,    0.0, 0.0, 0.0],
])

B = np.array([
    [0.0],
    [-1.0 / (m * l * l)],
    [0.0],
    [1.0 / m],
])

# Penalize states.
# Increase Q[0,0] for stronger pitch correction.
# Increase Q[1,1] for more pitch-rate damping.
# Keep Q[2,2] near zero until you have wheel encoders.
# Increase Q[3,3] for tighter velocity control.
Q = np.diag([
    2.0,    # theta
    0.05,   # theta_dot
    1e-4,    # position
    1.0,    # velocity
])

# Penalize control effort.
# Higher R = softer, less aggressive.
# Lower R = more aggressive.
R = np.array([[2.0]])

C = np.eye(4)
D = np.zeros((4, 1))

Ad, Bd, _, _, _ = cont2discrete((A, B, C, D), dt)

P = solve_discrete_are(Ad, Bd, Q, R)
K = np.linalg.inv(Bd.T @ P @ Bd + R) @ (Bd.T @ P @ Ad)

print("Paste this into src/config.rs:")
print("pub const LQR_K: [f32; 4] = [")
for v in K.ravel():
    print(f"    {float(v):.8}f32,")
print("];")