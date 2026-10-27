#!/usr/bin/env python3
"""Flywheel size for the motor (docs/balance-plan.md step 4b).

Runs on the PC (numpy, scipy, matplotlib):

    tests/balance/flywheel.py                 # table and plot
    tests/balance/flywheel.py -o fly.png      # plot to a file
    tests/balance/flywheel.py --t-max 0.3 --w-top 80

For each flywheel (a ring of mass m_w and radius r_w) it builds the
robot, redesigns the LQR gains for it (lqr.py) and finds the largest
tilt it can catch from rest and the largest push from upright, with the
motor's torque limit and top speed (pysim.py). Ideal sensors; the C
tests check the chosen design with the simulated IMU.

The robot without the wheel is a guess like everything else: 0.45 kg,
centre of mass 10 cm up, a 20 cm plate; the wheel's centre 17 cm up.
At 150 g and 6 cm this is close to plant_guess.
"""

import argparse
import math

import numpy as np

import lqr
import pysim

M_BODY = 0.45                    # kg, frame, motor, board, battery
L_BODY = 0.10                    # m, its centre of mass above the pivot
I_BODY = M_BODY * 0.2 ** 2 / 12  # kg m^2, about its centre of mass
H_WHEEL = 0.17                   # m, wheel centre above the pivot

MASSES = [0.05, 0.1, 0.15, 0.2, 0.3, 0.4]  # kg
RADII = [0.03, 0.04, 0.05, 0.06, 0.07, 0.08]  # m


def robot(m_w, r_w):
    """Model parameters (plant.c's) for a ring flywheel."""
    m = M_BODY + m_w
    l = (M_BODY * L_BODY + m_w * H_WHEEL) / m
    j = I_BODY + M_BODY * L_BODY ** 2 + m_w * H_WHEEL ** 2  # about the pivot
    return dict(m=m, l=l, i_b=j - m * l ** 2, i_w=m_w * r_w ** 2)


def limits(p, t_max, w_top):
    """(largest catch from rest in deg, largest push in rad/s, peak wheel
    speed in a 5 deg catch), or None where no gains could be made."""
    lqr.M, lqr.L, lqr.I_B, lqr.I_W = p["m"], p["l"], p["i_b"], p["i_w"]
    lqr.T_MAX, lqr.TORQUE_MAX = t_max, t_max / 4
    _, k, _ = lqr.gains()
    ok_tilt = lambda x: pysim.catch(p, k, t_max, w_top, theta0=x)[0]
    ok_push = lambda x: pysim.catch(p, k, t_max, w_top, push=x)[0]
    tilt = math.degrees(pysim.largest(ok_tilt, math.radians(40)))
    push = pysim.largest(ok_push, 5.0)
    peak = pysim.catch(p, k, t_max, w_top, theta0=math.radians(5))[1]
    return tilt, push, peak


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--t-max", type=float, default=lqr.T_MAX, help="N m (guess)")
    ap.add_argument("--w-top", type=float, default=60, help="rad/s (guess)")
    ap.add_argument("-o", "--out", help="save the plot to a PNG instead of showing it")
    args = ap.parse_args()

    tilt = np.zeros((len(MASSES), len(RADII)))
    push = np.zeros_like(tilt)
    print(f"torque limit {args.t_max * 1000:.0f} mN m, top speed {args.w_top:.0f} rad/s")
    print(" m_w   r_w   I_w (kg m2)  total  static  catch   push  peak w @5deg")
    for i, m_w in enumerate(MASSES):
        for j, r_w in enumerate(RADII):
            p = robot(m_w, r_w)
            static = math.degrees(math.asin(min(1, args.t_max / (p["m"] * lqr.G * p["l"]))))
            tilt[i, j], push[i, j], peak = limits(p, args.t_max, args.w_top)
            print(f"{m_w * 1000:4.0f}g {r_w * 100:3.0f}cm  {p['i_w']:.2e}  {p['m']:.2f}kg"
                  f"  {static:5.1f}°  {tilt[i, j]:5.1f}°  {push[i, j]:4.2f}  {peak:5.1f}")

    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, data, title, fmt in ((axes[0], tilt, "largest catch from rest (deg)", "{:.1f}"),
                                 (axes[1], push, "largest push from upright (rad/s)", "{:.2f}")):
        im = ax.imshow(data, origin="lower", cmap="viridis", aspect="auto")
        ax.set_xticks(range(len(RADII)), [f"{r * 100:.0f}" for r in RADII])
        ax.set_yticks(range(len(MASSES)), [f"{m * 1000:.0f}" for m in MASSES])
        ax.set_xlabel("flywheel radius (cm)")
        ax.set_ylabel("flywheel mass (g)")
        ax.set_title(title)
        for (i, j), v in np.ndenumerate(data):
            ax.text(j, i, fmt.format(v), ha="center", va="center",
                    color="w" if v < data.max() * 0.6 else "k", fontsize=9)
        fig.colorbar(im, ax=ax)
    fig.suptitle(f"ring flywheel, motor {args.t_max * 1000:.0f} mN m, "
                 f"top speed {args.w_top:.0f} rad/s (guesses)")
    fig.tight_layout()
    if args.out:
        fig.savefig(args.out, dpi=100)
    else:
        plt.show()


if __name__ == "__main__":
    main()
