"""
Plot a balance-mode CSV recorded with log_serial.py and print summary numbers
you can quote in your README (only quote numbers from YOUR runs).

    python plot_log.py ../data/run_001.csv
"""
import sys

import matplotlib.pyplot as plt
import numpy as np


def main(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    if d.size < 5 or "angle" not in d.dtype.names:
        sys.exit("Need a balance-mode log with columns t_ms,angle,rate,u,speed_rps,angle_offset,armed")
    t = (d["t_ms"] - d["t_ms"][0]) / 1000.0
    armed = d["armed"] > 0.5
    if armed.sum() < 5:
        sys.exit("The robot was never armed in this log.")

    a = d["angle"][armed]
    u = d["u"][armed]
    t_armed = t[armed]
    print(f"armed time          : {t_armed[-1] - t_armed[0]:.1f} s")
    print(f"RMS tilt            : {np.sqrt(np.mean(a**2)):.2f} deg")
    print(f"peak |tilt|         : {np.max(np.abs(a)):.2f} deg")
    print(f"mean |u|            : {np.mean(np.abs(u)):.0f} / 255 PWM")
    print(f"time saturated      : {100 * np.mean(np.abs(u) >= 254):.1f} %   (high = gains or motors too weak)")

    fig, ax = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    ax[0].plot(t, d["angle"]); ax[0].set_ylabel("tilt (deg)")
    ax[1].plot(t, d["rate"]); ax[1].set_ylabel("rate (deg/s)")
    ax[2].plot(t, d["u"]); ax[2].set_ylabel("u (PWM)")
    ax[3].plot(t, d["speed_rps"], label="speed (rev/s)")
    ax[3].plot(t, d["angle_offset"], label="angle offset (deg)")
    ax[3].legend(); ax[3].set_xlabel("time (s)")
    for x in ax:
        x.grid(alpha=0.3)
    fig.tight_layout()
    out = path.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=130)
    print(f"saved {out}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: python plot_log.py <log.csv>")
    main(sys.argv[1])
