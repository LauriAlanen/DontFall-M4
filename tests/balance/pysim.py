"""The balance simulator in Python, for the PC tools (sandbox.py,
flywheel.py): a copy of tests/balance/src/plant.c plus the motor's speed
limit. The C simulator and its tests stay the reference.
"""

import math

import lqr


def motor_limit(torque, omega, t_max, w_top):
    """Torque the motor can give at wheel speed omega: the back-EMF leaves
    less of t_max the faster it spins, none at w_top, when the torque
    would speed the wheel up further. Braking is never short.
    """
    if torque * omega <= 0:
        return torque
    room = t_max * max(0.0, 1 - abs(omega) / w_top)
    return math.copysign(min(abs(torque), room), torque)


class Robot:
    """The plant (plant.c) with the motor in front of it."""

    def __init__(self):
        self.reset()

    def reset(self, theta=math.radians(5)):
        self.theta, self.theta_dot, self.omega = theta, 0.0, 0.0
        self.wheel_angle = 0.0  # absolute, for drawing the spoke
        self.torque = 0.0       # what the motor actually gives
        self.fallen = False
        self.t = 0.0

    def deriv(self, p, x, torque):
        theta, theta_dot, omega = x
        j = p["i_b"] + p["m"] * p["l"] ** 2
        theta_ddot = (p["m"] * lqr.G * p["l"] * math.sin(theta) - torque) / j
        # lying on the floor: it holds the body unless pushed back up
        if abs(theta) >= math.pi / 2 and theta_ddot * theta > 0:
            theta_dot = theta_ddot = 0.0
        return (theta_dot, theta_ddot, torque / p["i_w"] - theta_ddot)

    def step(self, p, torque, h):
        x = (self.theta, self.theta_dot, self.omega)
        k1 = self.deriv(p, x, torque)
        k2 = self.deriv(p, [a + h / 2 * b for a, b in zip(x, k1)], torque)
        k3 = self.deriv(p, [a + h / 2 * b for a, b in zip(x, k2)], torque)
        k4 = self.deriv(p, [a + h * b for a, b in zip(x, k3)], torque)
        self.theta, self.theta_dot, self.omega = (
            a + h / 6 * (b + 2 * c + 2 * d + e) for a, b, c, d, e in zip(x, k1, k2, k3, k4))
        self.wheel_angle += h * (self.theta_dot + self.omega)
        # the floor
        if abs(self.theta) >= math.pi / 2:
            self.theta = math.copysign(math.pi / 2, self.theta)
            self.theta_dot = 0.0


def catch(p, k, t_max, w_top, theta0=0.0, push=0.0, secs=4.0, dt=lqr.DT, substeps=4):
    """The controller (balance.c, true state) from theta0 with the body
    pushed to push rad/s. Returns (caught, peak wheel speed, longest time
    the torque stayed at the limit). Caught: back under 1 deg at the end
    without passing 60 deg.
    """
    r = Robot()
    r.reset(theta0)
    r.theta_dot, r.omega = push, -push  # the wheel keeps its own speed
    peak_w = sat = longest = 0.0
    for _ in range(int(secs / dt)):
        cmd = -(k[0] * r.theta + k[1] * r.theta_dot + k[2] * r.omega)
        cmd = max(-t_max, min(t_max, cmd))
        sat = sat + dt if abs(cmd) >= t_max else 0.0
        longest = max(longest, sat)
        torque = motor_limit(cmd, r.omega, t_max, w_top)
        for _ in range(substeps):
            r.step(p, torque, dt / substeps)
        peak_w = max(peak_w, abs(r.omega))
        if abs(r.theta) > math.radians(60):
            return False, peak_w, longest
    return abs(r.theta) < math.radians(1), peak_w, longest


def largest(ok, hi, tol=1e-3):
    """Largest x in [0, hi] with ok(x), by bisection (ok is monotonic)."""
    lo = 0.0
    if ok(hi):
        return hi
    while hi - lo > tol:
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return lo
