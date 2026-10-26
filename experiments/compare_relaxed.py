# -*- coding: utf-8 -*-
"""Compare the baseline (10 deg / 250 s caps) and relaxed (15 deg / 400 s) runs.

Reads
    results/data/rocket_two_burn_result.mat   (baseline, key 'result')
    results/data/rocket_two_burn_relaxed.mat  (relaxed,  key 'r')
Writes
    experiments/relaxed_vs_baseline.png

Run from the project root:
    python experiments/compare_relaxed.py
"""
import json
import os
import warnings

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio

warnings.filterwarnings("ignore")

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DATA = os.path.join(ROOT, "results", "data")
OUT = os.path.dirname(os.path.abspath(__file__))

BLUE, RED, GREY = "#1f6fb4", "#c0392b", "#6b7a86"

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["SimSun", "serif"],
    "mathtext.fontset": "stix",
    "axes.unicode_minus": False,
    "font.size": 9,
    "axes.titlesize": 9,
    "legend.fontsize": 7.6,
    "axes.linewidth": 0.8,
    "lines.linewidth": 1.3,
    "savefig.bbox": "tight",
    "savefig.dpi": 200,
})


def load(name):
    return sio.loadmat(os.path.join(DATA, name), squeeze_me=True,
                       struct_as_record=False)


base = load("rocket_two_burn_result.mat")["result"]
relaxed = load("rocket_two_burn_relaxed.mat")["r"]


def summarise(r):
    o, p = r.out, r.p
    t = np.atleast_1d(o.t)
    y = np.atleast_2d(o.y)
    thrust = np.atleast_1d(o.thrust)
    return dict(
        success=int(r.success),
        residual=np.atleast_1d(r.residual).tolist(),
        fitness=float(r.fitness),
        pso_fitness=float(r.psoFitness),
        pso_residual=np.atleast_1d(r.psoResidual).tolist(),
        z=np.atleast_1d(r.z).tolist(),
        t_end=float(t[-1]),
        prop=float(o.propellantUsed),
        mfin=float(y[-1, 4]),
        impulse=float(o.totalImpulse),
        peak_g=float((thrust / y[:, 4]).max() / 9.80665),
        peak_qdyn=float(np.atleast_1d(o.dynamicPressure).max() / 1e3),
        ecc=float(r.osculatingEccentricity),
        peri=float(r.osculatingPerigee) / 1e3,
        apo=float(r.osculatingApogee) / 1e3,
        elapsed=float(r.elapsedSeconds),
        t=t, h=np.atleast_1d(o.h) / 1e3, q=np.atleast_1d(o.q),
        delta=np.degrees(np.atleast_1d(o.delta)),
        qdyn=np.atleast_1d(o.dynamicPressure) / 1e3,
    )


B, N = summarise(base), summarise(relaxed)

print(json.dumps({
    k: v for k, v in B.items() if not isinstance(v, np.ndarray)
}, indent=1, ensure_ascii=False))
print(json.dumps({
    k: v for k, v in N.items() if not isinstance(v, np.ndarray)
}, indent=1, ensure_ascii=False))

print("\n%-14s %16s | %16s" % ("", "baseline", "relaxed"))
for key, fmt in [("residual_h", "%.4f m"), ("residual_u", "%.5f m/s"),
                 ("residual_w", "%.5f m/s"), ("fitness", "%.3e"),
                 ("t_end", "%.3f s"), ("prop", "%.1f kg"),
                 ("mfin", "%.1f kg"), ("impulse", "%.1f MN s"),
                 ("peak_g", "%.2f g"), ("peak_qdyn", "%.2f kPa"),
                 ("ecc", "%.3e")]:
    if key.startswith("residual"):
        i = {"residual_h": 0, "residual_u": 1, "residual_w": 2}[key]
        a, b = B["residual"][i], N["residual"][i]
    elif key == "impulse":
        a, b = B[key] / 1e6, N[key] / 1e6
    else:
        a, b = B[key], N[key]
    print("%-14s %16s | %16s" % (key, fmt % a, fmt % b))

# ------------------------------------------------------------------ figure
fig, axs = plt.subplots(2, 2, figsize=(6.6, 4.3))
pairs = [(B, BLUE, "基线（±10°，滑行 ≤250 s）"),
         (N, RED, "放宽（±15°，滑行 ≤400 s）")]

ax = axs[0, 0]
for S, c, lab in pairs:
    ax.plot(S["t"], S["h"], color=c, label=lab)
ax.set_ylabel("高度 / km")
ax.set_xlabel("时间 / s")
ax.legend(frameon=False, loc="upper left")

ax = axs[0, 1]
for S, c, lab in pairs:
    ax.plot(S["t"], S["q"], color=c)
ax.set_ylabel("流率 / (kg/s)")
ax.set_xlabel("时间 / s")

ax = axs[1, 0]
for S, c, lab in pairs:
    ax.plot(S["t"], S["delta"], color=c)
ax.set_ylabel("推力偏角 / (°)")
ax.set_xlabel("时间 / s")

ax = axs[1, 1]
for S, c, lab in pairs:
    ax.plot(S["t"], S["qdyn"], color=c)
ax.set_ylabel("动压 / kPa")
ax.set_xlabel("时间 / s")

for ax in axs.ravel():
    ax.grid(alpha=0.25, lw=0.5)
    ax.set_title("", loc="left")
axs[0, 0].text(0.02, 1.03, "(a) 高度", transform=axs[0, 0].transAxes, fontsize=8.6)
axs[0, 1].text(0.02, 1.03, "(b) 流率", transform=axs[0, 1].transAxes, fontsize=8.6)
axs[1, 0].text(0.02, 1.03, "(c) 推力偏角", transform=axs[1, 0].transAxes, fontsize=8.6)
axs[1, 1].text(0.02, 1.03, "(d) 动压", transform=axs[1, 1].transAxes, fontsize=8.6)

fig.tight_layout()
fig.savefig(os.path.join(OUT, "relaxed_vs_baseline.png"))
print("\nfigure ->", os.path.join(OUT, "relaxed_vs_baseline.png"))
