#!/usr/bin/env python3
"""Balance sandbox: the reaction wheel balancer live, with sliders.

Runs on the PC (numpy, scipy, matplotlib with a display):

    tests/balance/sandbox.py

A toy to get a feel for the controller (docs/balance-plan.md), not a
test: the model is a Python copy of tests/balance/src/plant.c and the
controller of app/src/balance.c. The gains come from lqr.py, recomputed
when a weight slider moves.

- Weights: the Bryson limits LQR designs for (lqr.py).
- Robot: how far the real robot is from what the gains were designed
  for (x1 = the guesses).
- Trouble: IMU offset, gyro noise, delay, motor lag, torque limit and the
  motor's top speed.
- Push it with the buttons; past 25 deg the supervisor disarms and it
  falls over. Reset starts again from 5 deg.
"""

import collections
import math
import random
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, CheckButtons, Slider

import lqr

DT = lqr.DT
SUBSTEPS = 4
DISARM_TILT = math.radians(25)  # SUPERVISOR_DISARM_TILT
WINDOW = 6                      # s of history in the plots
PLOT_EVERY = 4                  # store every 4th sample for the plots
PUSH = 0.5                      # rad/s on the body, like the push test


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


class Sandbox:
    def __init__(self):
        self.robot = Robot()
        self.pending = collections.deque()  # torques on their way (delay)
        self.hist = collections.deque(maxlen=int(WINDOW / DT / PLOT_EVERY))
        self.n = 0
        self.build_ui()
        self.redesign()
        self.last = time.monotonic()

    # ---- UI ----------------------------------------------------------

    def build_ui(self):
        self.fig = fig = plt.figure(figsize=(13, 8.5))
        fig.canvas.manager.set_window_title("DontFall balance sandbox")

        self.ax_bot = fig.add_axes([0.02, 0.36, 0.36, 0.5])
        ax = self.ax_bot
        ax.set_xlim(-0.2, 0.2)
        ax.set_ylim(-0.02, 0.33)
        ax.set_aspect("equal")
        ax.axhline(0, c="k", lw=2)
        ax.set_xticks([])
        ax.set_yticks([])
        (self.body,) = ax.plot([], [], lw=10, c="#4a6fa5", solid_capstyle="round")
        self.wheel = plt.Circle((0, 0), 0.06, fill=False, lw=3, ec="#c0504d")
        ax.add_patch(self.wheel)
        (self.spoke,) = ax.plot([], [], lw=3, c="#c0504d")
        self.status = fig.text(0.2, 0.93, "", ha="center", fontsize=14, weight="bold")
        self.info = fig.text(0.03, 0.905, "", fontsize=9, family="monospace", va="top")

        self.ax_th = fig.add_axes([0.43, 0.77, 0.54, 0.2])
        self.ax_w = fig.add_axes([0.43, 0.56, 0.54, 0.2], sharex=self.ax_th)
        self.ax_tq = fig.add_axes([0.43, 0.35, 0.54, 0.2], sharex=self.ax_th)
        (self.l_th,) = self.ax_th.plot([], [])
        (self.l_w,) = self.ax_w.plot([], [], c="#c0504d")
        (self.l_tq,) = self.ax_tq.plot([], [], c="#9467bd")
        self.ax_th.set_ylabel("tilt θ (deg)")
        self.ax_w.set_ylabel("wheel ω (rad/s)")
        self.ax_tq.set_ylabel("torque (mN m)")
        self.ax_tq.set_xlabel("t (s)")
        self.lim_w = [self.ax_w.axhline(s * 1, ls=":", c="grey") for s in (1, -1)]
        self.lim_tq = [self.ax_tq.axhline(s * 1, ls=":", c="grey") for s in (1, -1)]
        for a in (self.ax_th, self.ax_w, self.ax_tq):
            a.grid(True)
        plt.setp(self.ax_th.get_xticklabels(), visible=False)
        plt.setp(self.ax_w.get_xticklabels(), visible=False)

        def slider(col, row, label, lo, hi, init, fmt, cb):
            x = [0.1, 0.43, 0.76][col]
            ax = fig.add_axes([x, 0.24 - row * 0.035, 0.17, 0.022])
            s = Slider(ax, label, lo, hi, valinit=init, valfmt=fmt)
            s.label.set_fontsize(8.5)
            s.on_changed(cb)
            return s

        fig.text(0.1, 0.27, "LQR weights (Bryson)", weight="bold")
        fig.text(0.43, 0.27, "Robot vs design", weight="bold")
        fig.text(0.76, 0.27, "Trouble", weight="bold")
        re = lambda _: self.redesign()
        nop = lambda _: None
        self.s_thmax = slider(0, 0, "θ max (°)", 1, 15, 5, "%.1f", re)
        self.s_thdmax = slider(0, 1, "rate max (rad/s)", 0.5, 5, 2, "%.1f", re)
        self.s_wmax = slider(0, 2, "ω max (rad/s)", 5, 100, 30, "%.0f", re)
        self.s_tqw = slider(0, 3, "torque / limit", 0.1, 1, 0.25, "%.2f", re)
        self.s_m = slider(1, 0, "mass ×", 0.5, 1.5, 1, "%.2f", nop)
        self.s_l = slider(1, 1, "CoM height ×", 0.5, 1.5, 1, "%.2f", nop)
        self.s_iw = slider(1, 2, "wheel inertia ×", 0.3, 3, 1, "%.2f", nop)
        self.s_tmax = slider(1, 3, "torque limit (mN m)", 50, 500, 1000 * lqr.T_MAX, "%.0f", re)
        self.s_wtop = slider(1, 4, "motor top speed", 10, 150, 60, "%.0f", nop)
        self.s_off = slider(2, 0, "IMU offset (°)", -3, 3, 0, "%.1f", nop)
        self.s_noise = slider(2, 1, "gyro noise (rad/s)", 0, 0.1, 0, "%.3f", nop)
        self.s_delay = slider(2, 2, "delay (samples)", 0, 40, 0, "%.0f", nop)
        self.s_lag = slider(2, 3, "motor lag (ms)", 0, 30, 0, "%.0f", nop)
        self.s_slow = slider(2, 4, "speed ×", 0.05, 1, 1, "%.2f", nop)

        def button(x, label, cb):
            b = Button(fig.add_axes([x, 0.02, 0.09, 0.04]), label)
            b.on_clicked(cb)
            return b

        self.b_left = button(0.1, "◀ push", lambda _: self.push(-1))
        self.b_right = button(0.2, "push ▶", lambda _: self.push(1))
        self.b_reset = button(0.3, "reset 5°", lambda _: self.reset())
        self.b_upright = button(0.4, "reset 0°", lambda _: self.reset(0.0))
        self.chk = CheckButtons(fig.add_axes([0.52, 0.01, 0.16, 0.06]),
                                ["ω term (k3)", "pause"], [True, False])

    # ---- control -----------------------------------------------------

    def redesign(self):
        lqr.THETA_MAX = math.radians(self.s_thmax.val)
        lqr.THETA_DOT_MAX = self.s_thdmax.val
        lqr.OMEGA_MAX = self.s_wmax.val
        lqr.T_MAX = self.s_tmax.val / 1000
        lqr.TORQUE_MAX = lqr.T_MAX * self.s_tqw.val
        (_, _, ad, bd), k, _ = lqr.gains()
        self.k = k
        poles = np.log(np.linalg.eigvals(ad - bd @ k[None, :])) / DT
        self.design = (f"k = {k[0]:+.3f} {k[1]:+.4f} {k[2]:+.5f}\n"
                       "poles " + " ".join(f"{p.real:.1f}{p.imag:+.1f}j" for p in poles))
        for line, s in zip(self.lim_tq, (1, -1)):
            line.set_ydata([s * lqr.T_MAX * 1000] * 2)

    def push(self, sign):
        self.robot.theta_dot += sign * PUSH
        self.robot.omega -= sign * PUSH  # the wheel keeps its own speed

    def reset(self, theta=math.radians(5)):
        self.robot.reset(theta)
        self.pending.clear()
        self.hist.clear()

    def params(self):
        return {
            "m": lqr.M * self.s_m.val,
            "l": lqr.L * self.s_l.val,
            "i_b": lqr.I_B * self.s_m.val,
            "i_w": lqr.I_W * self.s_iw.val,
        }

    def control(self):
        """One sample: balance.c with the IMU's and the motor's faults."""
        r = self.robot
        k = self.k.copy()
        if not self.chk.get_status()[0]:
            k[2] = 0
        theta = r.theta + math.radians(self.s_off.val)
        theta_dot = r.theta_dot + random.gauss(0, self.s_noise.val) if self.s_noise.val else r.theta_dot
        armed = not r.fallen
        t = -(k[0] * theta + k[1] * theta_dot + k[2] * r.omega) if armed else 0.0
        t = max(-lqr.T_MAX, min(lqr.T_MAX, t))
        self.pending.append(t)
        while len(self.pending) > int(self.s_delay.val) + 1:
            self.pending.popleft()
        return self.pending[0]

    def motor(self, cmd):
        r = self.robot
        lag = self.s_lag.val / 1000
        r.torque += (cmd - r.torque) * (1 - math.exp(-DT / lag)) if lag else cmd - r.torque
        # out of speed: no more torque the way that speeds it up
        if abs(r.omega) >= self.s_wtop.val and r.torque * r.omega > 0:
            return 0.0
        return r.torque

    def tick(self):
        r, p = self.robot, self.params()
        torque = self.motor(self.control())
        for _ in range(SUBSTEPS):
            r.step(p, torque, DT / SUBSTEPS)
        r.t += DT
        if abs(r.theta) > DISARM_TILT:
            r.fallen = True
        if self.n % PLOT_EVERY == 0:
            self.hist.append((r.t, math.degrees(r.theta), r.omega, 1000 * torque))
        self.n += 1

    # ---- drawing -----------------------------------------------------

    def frame(self):
        now = time.monotonic()
        elapsed, self.last = min(now - self.last, 0.1), now
        if not self.chk.get_status()[1]:
            for _ in range(int(elapsed * self.s_slow.val / DT)):
                self.tick()

        r, p = self.robot, self.params()
        h = 2 * p["l"] / lqr.L * 0.12  # draw the body a bit taller than the CoM
        top = (h * math.sin(r.theta), h * math.cos(r.theta))
        self.body.set_data([0, top[0]], [0, top[1]])
        self.wheel.center = top
        a = r.wheel_angle
        self.spoke.set_data([top[0], top[0] + 0.06 * math.sin(a)],
                            [top[1], top[1] + 0.06 * math.cos(a)])
        saturated = abs(r.torque) >= lqr.T_MAX * 0.999
        out_of_speed = abs(r.omega) >= self.s_wtop.val
        if r.fallen:
            self.status.set_text("FELL: supervisor disarmed at 25°")
            self.status.set_color("#c0504d")
        elif out_of_speed:
            self.status.set_text("wheel at the motor's top speed")
            self.status.set_color("#d68a00")
        elif saturated:
            self.status.set_text("torque at the limit")
            self.status.set_color("#d68a00")
        else:
            self.status.set_text("balancing")
            self.status.set_color("#2e7d32")
        self.info.set_text(f"{self.design}\n"
                           f"θ {math.degrees(r.theta):+6.2f}°  ω {r.omega:+6.1f} rad/s")
        for line, s in zip(self.lim_w, (1, -1)):
            line.set_ydata([s * self.s_wtop.val] * 2)

        if self.hist:
            t, th, w, tq = zip(*self.hist)
            self.l_th.set_data(t, th)
            self.l_w.set_data(t, w)
            self.l_tq.set_data(t, tq)
            self.ax_th.set_xlim(max(0, t[-1] - WINDOW), max(WINDOW, t[-1]))
            for ax, ys, floor in ((self.ax_th, th, 1), (self.ax_w, w, 5),
                                  (self.ax_tq, tq, 1000 * lqr.T_MAX * 1.1)):
                m = max(floor, max(abs(v) for v in ys) * 1.1)
                ax.set_ylim(-m, m)
        self.fig.canvas.draw_idle()

    def run(self):
        timer = self.fig.canvas.new_timer(interval=33)
        timer.add_callback(self.frame)
        timer.start()
        plt.show()


if __name__ == "__main__":
    Sandbox().run()
