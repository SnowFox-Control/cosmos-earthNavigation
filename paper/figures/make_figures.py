# -*- coding: utf-8 -*-
"""Regenerate every figure of the paper from the saved MAT results.

Run from the project root:
    python paper/figures/make_figures.py
Outputs vector PDFs into the same folder and prints the numbers quoted in text.
"""
import json
import os
import warnings

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
from matplotlib.patches import Arc, Circle, FancyArrowPatch, Rectangle

warnings.filterwarnings("ignore")

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DATA = os.path.join(ROOT, "results", "data")
OUT = os.path.dirname(os.path.abspath(__file__))

BLUE = "#1f6fb4"
RED = "#c0392b"
GREY = "#6b7a86"
GREEN = "#2e7d4f"
ORANGE = "#d98032"

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["SimSun", "Songti SC", "serif"],
    "mathtext.fontset": "stix",
    "axes.unicode_minus": False,
    "font.size": 9,
    "axes.titlesize": 9,
    "axes.labelsize": 9,
    "legend.fontsize": 7.6,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "axes.linewidth": 0.8,
    "lines.linewidth": 1.3,
    "figure.dpi": 140,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
})


def load(name):
    return sio.loadmat(os.path.join(DATA, name), squeeze_me=True,
                       struct_as_record=False)


plan = load("rocket_two_burn_result.mat")["result"]
mpc = load("rocket_mpc_result.mat")["result"]
coast = load("rocket_orbit_coast.mat")["coastResult"]
bench = load("rocket_acceleration_benchmark.mat")["report"]

p = plan.p
o = plan.out
t = np.atleast_1d(o.t)
y = np.atleast_2d(o.y)
h = np.atleast_1d(o.h)
down = np.atleast_1d(o.downrange)
u = y[:, 2]
w = y[:, 3]
speed = np.atleast_1d(o.speed)
mass = y[:, 4]
q = np.atleast_1d(o.q)
delta = np.degrees(np.atleast_1d(o.delta))
thrust = np.atleast_1d(o.thrust)
qdyn = np.atleast_1d(o.dynamicPressure)
accel = thrust / mass

z = np.atleast_1d(plan.z)
t1, coast_dur, t2 = z[7], z[8], z[9]
t_cut1 = t1
t_relight = t1 + coast_dur
t_final = t_relight + t2
RE = p.Re
G0 = 9.80665

summary = {
    "target_alt_km": float(plan.opt.targetAltitude) / 1e3,
    "target_speed": float(plan.targetSpeed),
    "z": z.tolist(),
    "residual": np.atleast_1d(plan.residual).tolist(),
    "pso_residual": np.atleast_1d(plan.psoResidual).tolist(),
    "pso_fitness": float(plan.psoFitness),
    "fitness": float(plan.fitness),
    "t_final": float(t[-1]),
    "t_cut1": float(t_cut1),
    "t_relight": float(t_relight),
    "propellant": float(o.propellantUsed),
    "impulse": float(o.totalImpulse),
    "mass_final": float(mass[-1]),
    "mass_ratio": float(p.m0 / mass[-1]),
    "peak_qdyn": float(qdyn.max()),
    "peak_qdyn_t": float(t[qdyn.argmax()]),
    "peak_accel": float(accel.max()),
    "peak_accel_g": float(accel.max() / G0),
    "peak_accel_t": float(t[accel.argmax()]),
    "peak_thrust": float(thrust.max()),
    "h_cut1": float(h[np.searchsorted(t, t_cut1)]) / 1e3,
    "h_relight": float(h[np.searchsorted(t, t_relight)]) / 1e3,
    "h_max_coast": float(h[(t >= t_cut1) & (t <= t_relight)].max()) / 1e3,
    "coast_period": float(coast.period),
    "coast_alt_min_km": float(np.atleast_1d(coast.altitude).min()) / 1e3,
    "coast_alt_max_km": float(np.atleast_1d(coast.altitude).max()) / 1e3,
    "bench_cpu_s": float(bench.serialCPUSeconds),
    "bench_gpu_s": float(bench.gpuSeconds),
    "bench_speedup": float(bench.speedup),
    "bench_diff": float(bench.relativeObjectiveDifference),
    "ecc": float(plan.osculatingEccentricity),
    "perigee_km": float(plan.osculatingPerigee) / 1e3,
    "apogee_km": float(plan.osculatingApogee) / 1e3,
    "refine_diff": np.atleast_1d(plan.refinementDifference).tolist(),
    "elapsed": float(plan.elapsedSeconds),
}


def polar(deg, r):
    a = np.deg2rad(deg)
    return r * np.cos(a), r * np.sin(a)


# ---------------------------------------------------------------- figure 1
def fig_model():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(6.4, 3.0),
                                   gridspec_kw={"width_ratios": [1, 1.02]})
    # (a) geocentric polar frame
    ax1.add_patch(Circle((0, 0), RE / 1e3, facecolor="#dce9f5",
                         edgecolor="#4a6d8c", lw=1.0, zorder=1))
    ang = np.linspace(0, 2 * np.pi, 400)
    r_t = (RE + plan.opt.targetAltitude) / 1e3
    ax1.plot(r_t * np.cos(ang), r_t * np.sin(ang), ls="-.", lw=0.9,
             color=GREEN, zorder=2)
    th0 = np.deg2rad(45)
    ax1.add_patch(FancyArrowPatch((0, 0), (r_t * np.cos(th0), r_t * np.sin(th0)),
                                  arrowstyle="-|>", mutation_scale=9,
                                  lw=0.9, color="k", zorder=3))
    ax1.plot([r_t * np.cos(th0)], [r_t * np.sin(th0)], "o", ms=4, color=BLUE,
             zorder=4)
    ax1.add_patch(Arc((0, 0), 0.62 * r_t, 0.62 * r_t, theta1=0,
                      theta2=np.rad2deg(th0), color="k", lw=0.8))
    ax1.text(0.30 * r_t * np.cos(th0 / 2), 0.30 * r_t * np.sin(th0 / 2),
             r"$\theta$", fontsize=9)
    ax1.text(-0.60 * r_t, 0.06 * r_t, r"$R_E$", color="#33526e")
    ax1.annotate(r"$r$", xy=(0.52 * r_t * np.cos(th0), 0.52 * r_t * np.sin(th0)),
                 xytext=(5, 5), textcoords="offset points", fontsize=9)
    ax1.annotate("目标圆轨道", xy=polar(118, r_t), xytext=(0.02 * r_t, 0.80 * r_t),
                 fontsize=7.5, color=GREEN,
                 arrowprops=dict(arrowstyle="-", lw=0.6, color=GREEN))
    ax1.plot([0], [0], "k+", ms=5, mew=0.8)
    ax1.text(0.03 * r_t, -0.09 * r_t, "地心", fontsize=7.2, color="#33526e")
    ax1.set_xlim(-1.12 * r_t, 1.12 * r_t)
    ax1.set_ylim(-1.12 * r_t, 1.12 * r_t)
    ax1.set_aspect("equal")
    ax1.axis("off")
    ax1.set_title("(a) 地心惯性极坐标", fontsize=9)

    # (b) local frame attached to the vehicle
    ax2.set_xlim(-0.30, 1.20)
    ax2.set_ylim(-0.08, 1.16)
    ax2.axis("off")
    ax2.plot([0], [0], "o", ms=3, color="k")
    ax2.annotate("", xy=(0, 1.06), xytext=(0, 0),
                 arrowprops=dict(arrowstyle="-|>", color=GREY, lw=1.0))
    ax2.annotate("", xy=(1.08, 0), xytext=(0, 0),
                 arrowprops=dict(arrowstyle="-|>", color=GREY, lw=1.0))
    ax2.text(1.10, 0.02, r"$\hat e_\theta$", color="#3b4a55")
    ax2.text(0.05, 1.00, r"$\hat e_r$", color="#3b4a55")

    beta = 32.0
    alpha = 58.0
    V, T = 0.92, 0.62
    ax2.annotate("", xy=polar(90 - beta, V), xytext=(0, 0),
                 arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=1.4))
    ax2.annotate("", xy=polar(90 - alpha, T), xytext=(0, 0),
                 arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=1.4))
    ax2.add_patch(Arc((0, 0), 0.50, 0.50, theta1=90 - beta, theta2=90,
                      color=BLUE, lw=0.8))
    ax2.add_patch(Arc((0, 0), 0.74, 0.74, theta1=90 - alpha, theta2=90,
                      color=ORANGE, lw=0.8))
    ax2.add_patch(Arc((0, 0), 0.30, 0.30, theta1=90 - alpha, theta2=90 - beta,
                      color=RED, lw=1.0))
    ax2.text(*polar(90 - beta / 2, 0.60), r"$\beta$", color=BLUE)
    ax2.text(*polar(85, 0.82), r"$\alpha$", color=ORANGE)
    ax2.text(*polar(90 - (alpha + beta) / 2, 0.40), r"$\delta$", color=RED)
    ax2.text(*polar(90 - beta, V + 0.09), r"$\vec v$", color=BLUE)
    ax2.text(*polar(90 - alpha, T + 0.11), r"$\vec T=cq$", color=ORANGE)
    ax2.set_title("(b) 箭体当地系（角度示意）", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_model.pdf"))
    plt.close(fig)


# ---------------------------------------------------------------- figure 2
def fig_trajectory():
    th = y[:, 1]
    x = y[:, 0] * np.cos(th) / 1e3
    yy = y[:, 0] * np.sin(th) / 1e3
    m1 = t <= t_cut1
    m2 = (t > t_cut1) & (t <= t_relight)
    m3 = t > t_relight

    fig, (ax1, ax2) = plt.subplots(
        1, 2, figsize=(6.7, 3.05),
        gridspec_kw={"width_ratios": [1.0, 1.28]})
    ax1.add_patch(Circle((0, 0), RE / 1e3, facecolor="#dce9f5",
                         edgecolor="#4a6d8c", lw=1.0, zorder=0))
    ang = np.linspace(0, 2 * np.pi, 600)
    ax1.plot((RE + plan.opt.targetAltitude) / 1e3 * np.cos(ang),
             (RE + plan.opt.targetAltitude) / 1e3 * np.sin(ang),
             ls="-.", lw=0.9, color=GREEN, zorder=1)
    ax1.plot(x[m1], yy[m1], color=RED, lw=2.0, label="第一段燃烧", zorder=3)
    ax1.plot(x[m2], yy[m2], color=GREY, lw=1.0, label="滑行段", zorder=2)
    ax1.plot(x[m3], yy[m3], color=BLUE, lw=2.0, label="圆轨化燃烧", zorder=3)
    ax1.plot(x[-1], yy[-1], "o", ms=4.0, mfc="w", mec=BLUE, mew=1.0, zorder=4)
    ax1.set_aspect("equal", adjustable="box")
    lim = 1.06 * (RE + plan.opt.targetAltitude) / 1e3
    ax1.set_xlim(-lim, lim)
    ax1.set_ylim(-lim, lim)
    ax1.set_xlabel("地心 $x$ / km")
    ax1.set_ylabel("地心 $y$ / km")
    ax1.legend(loc="lower left", frameon=False, handlelength=1.2,
               borderaxespad=0.15, fontsize=7.0)
    ax1.set_title("(a) 惯性平面内的轨迹", fontsize=8.6)

    ax2.axhspan(0, 95, color="#f4f4f4", zorder=0)
    ax2.plot(down[m1] / 1e3, h[m1] / 1e3, color=RED, lw=1.7)
    ax2.plot(down[m2] / 1e3, h[m2] / 1e3, color=GREY, lw=1.0)
    ax2.plot(down[m3] / 1e3, h[m3] / 1e3, color=BLUE, lw=1.7)
    ax2.axhline(plan.opt.targetAltitude / 1e3, ls="-.", lw=0.9, color=GREEN,
                label="目标高度 600 km")
    for tt, lab, c, off in ((t_cut1, "第一次关机", RED, (14, -22)),
                            (t_relight, "第二次点火", ORANGE, (16, -30))):
        k = np.searchsorted(t, tt)
        ax2.plot(down[k] / 1e3, h[k] / 1e3, "s", ms=3.4, color=c, zorder=5)
        ax2.annotate(lab, xy=(down[k] / 1e3, h[k] / 1e3),
                     xytext=off, textcoords="offset points",
                     fontsize=7.0, color=c,
                     arrowprops=dict(arrowstyle="-", lw=0.6, color=c))
    ax2.set_xlabel("地表航程 / km")
    ax2.set_ylabel("高度 / km")
    ax2.set_xlim(-30, 1090)
    ax2.grid(alpha=0.25, lw=0.5)
    ax2.legend(loc="lower right", frameon=False, handlelength=1.4, fontsize=7.0)
    ax2.set_title("(b) 高度—航程剖面", fontsize=8.6)
    fig.tight_layout(w_pad=1.0)
    fig.savefig(os.path.join(OUT, "fig_trajectory.pdf"))
    plt.close(fig)


# ---------------------------------------------------------------- figure 3
def fig_states():
    fig, axs = plt.subplots(2, 2, figsize=(6.4, 4.1), sharex=True)
    ax = axs[0, 0]
    ax.plot(t, h / 1e3, color=BLUE)
    ax.axhline(plan.opt.targetAltitude / 1e3, ls="-.", lw=0.8, color=GREEN)
    ax.set_ylabel("高度 $h$ / km")
    ax = axs[0, 1]
    ax.plot(t, u, color=RED, label="径向 $u$")
    ax.plot(t, w, color=BLUE, label="切向 $w$")
    ax.axhline(plan.targetSpeed, ls="-.", lw=0.8, color=GREEN)
    ax.set_ylabel("速度分量 / (m/s)")
    ax.legend(frameon=False, loc="center", ncol=2)
    ax = axs[1, 0]
    ax.plot(t, mass / 1e3, color=GREY)
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("质量 $m$ / t")
    ax = axs[1, 1]
    ax.plot(t, speed, color=GREEN)
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("速率 $V$ / (m/s)")
    for ax in axs.ravel():
        for tt, c in ((t_cut1, RED), (t_relight, ORANGE)):
            ax.axvline(tt, color=c, lw=0.7, ls=":", alpha=0.8)
        ax.grid(alpha=0.25, lw=0.5)
    axs[0, 0].set_title("(a) 高度", loc="left", fontsize=8.8)
    axs[0, 1].set_title("(b) 径向与切向速度", loc="left", fontsize=8.8)
    axs[1, 0].set_title("(c) 质量", loc="left", fontsize=8.8)
    axs[1, 1].set_title("(d) 速率", loc="left", fontsize=8.8)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_states.pdf"))
    plt.close(fig)


# ---------------------------------------------------------------- figure 4
def fig_control():
    fig, axs = plt.subplots(2, 2, figsize=(6.4, 4.0), sharex=True)
    ax = axs[0, 0]
    ax.plot(t, delta, color=BLUE)
    ax.plot(np.atleast_1d(plan.control.t),
            np.degrees(np.atleast_1d(plan.control.delta)), "o", ms=2.6,
            mfc="w", mec=BLUE, mew=0.9)
    ax.set_ylabel(r"推力偏角 $\delta$ / (°)")
    ax = axs[0, 1]
    ax.plot(t, q, color=RED)
    ax.set_ylabel(r"流率 $q$ / (kg/s)")
    ax = axs[1, 0]
    ax.plot(t, qdyn / 1e3, color=GREEN)
    ax.plot([t[qdyn.argmax()]], [qdyn.max() / 1e3], "o", ms=3, mfc="w",
            mec=GREEN, mew=0.9)
    ax.annotate("峰值 %.1f kPa" % (qdyn.max() / 1e3),
                xy=(t[qdyn.argmax()], qdyn.max() / 1e3),
                xytext=(24, -18), textcoords="offset points", fontsize=7.2,
                color=GREEN,
                arrowprops=dict(arrowstyle="->", lw=0.6, color=GREEN))
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("动压 $q_d$ / kPa")
    ax = axs[1, 1]
    ax.plot(t, accel / G0, color=ORANGE)
    ax.annotate("峰值 %.1f g" % (accel.max() / G0),
                xy=(t[accel.argmax()], accel.max() / G0),
                xytext=(-72, -14), textcoords="offset points", fontsize=7.2,
                color=ORANGE,
                arrowprops=dict(arrowstyle="->", lw=0.6, color=ORANGE))
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("过载 $a/g_0$")
    for ax in axs.ravel():
        for tt, c in ((t_cut1, RED), (t_relight, ORANGE)):
            ax.axvline(tt, color=c, lw=0.7, ls=":", alpha=0.8)
        ax.grid(alpha=0.25, lw=0.5)
    titles = ["(a) 推力偏角", "(b) 流率", "(c) 动压", "(d) 过载"]
    for ax, lab in zip(axs.ravel(), titles):
        ax.set_title(lab, loc="left", fontsize=8.6)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_control.pdf"))
    plt.close(fig)


# ---------------------------------------------------------------- figure 5
def fig_pso():
    hist = np.atleast_2d(plan.history)
    fig, ax = plt.subplots(figsize=(4.4, 2.9))
    ax.semilogy(hist[:, 0], hist[:, 1], color=BLUE, label="种群最优 $J$")
    ax.semilogy(hist[:, 0], hist[:, 2], color=GREY, lw=0.9, alpha=0.85,
                label="种群均值 $J$")
    ax.axhline(plan.fitness, color=RED, lw=0.6, ls=":", zorder=1)
    ax.plot(hist[-1, 0], plan.fitness, "o", ms=4.5, color=RED, zorder=5,
            label="SQP 精修后")
    ax.set_xlabel("迭代代数")
    ax.set_ylabel("归一化目标函数 $J$")
    ax.set_ylim(1e-9, 3e3)
    ax.grid(alpha=0.25, lw=0.5, which="both")
    ax.legend(frameon=False, loc="lower left")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_pso.pdf"))
    plt.close(fig)


# ---------------------------------------------------------------- figure 6
def fig_benchmark():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(5.6, 2.5),
                                   gridspec_kw={"width_ratios": [1.35, 1]})
    labels = ["串行 CPU\n(ode45)", "GPU 批量\n(RK4)"]
    vals = [bench.serialCPUSeconds, bench.gpuSeconds]
    bars = ax1.bar(labels, vals, color=[GREY, BLUE], width=0.55)
    ax1.set_yscale("log")
    ax1.set_ylabel("96 条轨迹墙钟时间 / s")
    for b, v in zip(bars, vals):
        ax1.text(b.get_x() + b.get_width() / 2, v * 1.25, "%.3f s" % v,
                 ha="center", fontsize=8)
    ax1.set_ylim(0.1, 40)
    ax1.grid(alpha=0.25, lw=0.5, axis="y", which="both")
    ax1.set_title("(a) 批量耗时（对数轴）", loc="left", fontsize=8.8)
    ax2.bar(["CPU", "GPU"], [bench.cpuObjective, bench.gpuObjective],
            color=[GREY, BLUE], width=0.5)
    for k, v in enumerate([bench.cpuObjective, bench.gpuObjective]):
        ax2.text(k, v + 0.006, "%.4f" % v, ha="center", fontsize=8)
    ax2.set_ylim(0, 0.55)
    ax2.set_ylabel("归一化目标值 $J$")
    ax2.set_title("(b) 同一控制的目标值", loc="left", fontsize=8.8)
    ax2.grid(alpha=0.25, lw=0.5, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_benchmark.pdf"))
    plt.close(fig)


# ---------------------------------------------------------------- figure 7, 8
def mpc_errors():
    tt = np.atleast_1d(mpc.time)
    ref = np.atleast_2d(mpc.referenceState)
    cl = np.atleast_2d(mpc.closedState)
    op = np.atleast_2d(mpc.openState)
    pos_cl = np.hypot(cl[:, 0] - ref[:, 0], RE * (cl[:, 1] - ref[:, 1]))
    pos_op = np.hypot(op[:, 0] - ref[:, 0], RE * (op[:, 1] - ref[:, 1]))
    vel_cl = np.hypot(cl[:, 2] - ref[:, 2], cl[:, 3] - ref[:, 3])
    vel_op = np.hypot(op[:, 2] - ref[:, 2], op[:, 3] - ref[:, 3])
    return tt, ref, cl, op, pos_cl, pos_op, vel_cl, vel_op


def fig_mpc_tracking():
    tt, ref, cl, op, *_ = mpc_errors()
    fig, axs = plt.subplots(3, 1, figsize=(5.4, 4.6), sharex=True)
    ref_style = dict(color="#b3c0c9", lw=3.2, zorder=1)
    open_style = dict(color=RED, lw=1.0, ls="--", zorder=2)
    closed_style = dict(color=BLUE, lw=1.2, zorder=3)
    ax = axs[0]
    ax.plot(tt, ref[:, 0] / 1e3 - RE / 1e3, label="参考（PSO）", **ref_style)
    ax.plot(tt, op[:, 0] / 1e3 - RE / 1e3, label="开环", **open_style)
    ax.plot(tt, cl[:, 0] / 1e3 - RE / 1e3, label="MPC 闭环", **closed_style)
    ax.set_ylabel("高度 / km")
    ax.legend(frameon=False, ncol=3, loc="upper left")
    ax = axs[1]
    ax.plot(tt, ref[:, 2], **ref_style)
    ax.plot(tt, op[:, 2], **open_style)
    ax.plot(tt, cl[:, 2], **closed_style)
    ax.set_ylabel("径向速度 / (m/s)")
    ax = axs[2]
    ax.plot(tt, ref[:, 3], **ref_style)
    ax.plot(tt, op[:, 3], **open_style)
    ax.plot(tt, cl[:, 3], **closed_style)
    ax.axhline(plan.targetSpeed, ls="-.", lw=0.8, color=GREEN)
    ax.set_ylabel("切向速度 / (m/s)")
    ax.set_xlabel("时间 / s")
    for ax in axs:
        ax.grid(alpha=0.25, lw=0.5)
        for tt0, c in ((t_cut1, RED), (t_relight, ORANGE)):
            ax.axvline(tt0, color=c, lw=0.6, ls=":", alpha=0.6)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_mpc_tracking.pdf"))
    plt.close(fig)


def fig_mpc_error():
    tt, ref, cl, op, pos_cl, pos_op, vel_cl, vel_op = mpc_errors()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(5.4, 3.6), sharex=True)
    ax1.set_yscale("symlog", linthresh=1.0, linscale=0.5)
    ax1.plot(tt, pos_cl, color=BLUE, label="MPC 闭环")
    ax1.plot(tt, pos_op, color=RED, ls="--", label="开环")
    ax1.set_ylabel("位置误差 / m")
    ax1.legend(frameon=False, ncol=2, loc="lower right")
    ax2.set_yscale("symlog", linthresh=1e-2, linscale=0.5)
    ax2.plot(tt, vel_cl, color=BLUE)
    ax2.plot(tt, vel_op, color=RED, ls="--")
    ax2.set_ylabel("速度误差 / (m/s)")
    ax2.set_xlabel("时间 / s")
    for ax in (ax1, ax2):
        ax.grid(alpha=0.25, lw=0.5, which="both")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_mpc_error.pdf"))
    plt.close(fig)


# ---------------------------------------------------------------- figure 9
def fig_coast():
    tc = np.atleast_1d(coast.t)
    alt = np.atleast_1d(coast.altitude)
    fig, ax = plt.subplots(figsize=(5.0, 2.5))
    ax.plot(tc / 60, alt - alt.mean(), color=BLUE, lw=0.7)
    ax.axhline(0, color=GREY, lw=0.6)
    ax.set_xlabel("关机后时间 / min")
    ax.set_ylabel("高度偏差（相对均值）/ m")
    ax.grid(alpha=0.25, lw=0.5)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_coast.pdf"))
    plt.close(fig)


# ---------------------------------------------------------------- figure 10
def fig_qp():
    sec = np.atleast_1d(mpc.QPSolveSeconds)
    sec = sec[np.isfinite(sec)]
    fig, ax = plt.subplots(figsize=(4.4, 2.5))
    ax.hist(sec * 1e3, bins=np.logspace(-0.2, 2.0, 26), color=BLUE,
            edgecolor="w", lw=0.4)
    ax.set_xscale("log")
    ax.axvline(sec.mean() * 1e3, color=RED, lw=0.9)
    ax.text(sec.mean() * 1e3 * 1.15, ax.get_ylim()[1] * 0.85,
            "均值 %.2f ms" % (sec.mean() * 1e3), color=RED, fontsize=7.4)
    ax.set_xlabel("单次 QP 求解时间 / ms")
    ax.set_ylabel("调用次数")
    ax.grid(alpha=0.25, lw=0.5, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig_qp.pdf"))
    plt.close(fig)


if __name__ == "__main__":
    fig_model()
    fig_trajectory()
    fig_states()
    fig_control()
    fig_pso()
    fig_benchmark()
    fig_mpc_tracking()
    fig_mpc_error()
    fig_coast()
    fig_qp()
    tt, ref, cl, op, pos_cl, pos_op, vel_cl, vel_op = mpc_errors()
    summary.update({
        "mpc_time": float(tt[-1]),
        "closed_pos_rms": float(np.sqrt(np.mean(pos_cl ** 2))),
        "open_pos_rms": float(np.sqrt(np.mean(pos_op ** 2))),
        "closed_vel_rms": float(np.sqrt(np.mean(vel_cl ** 2))),
        "open_vel_rms": float(np.sqrt(np.mean(vel_op ** 2))),
        "closed_final_alt_err": float(cl[-1, 0] - RE - plan.opt.targetAltitude),
        "open_final_alt_err": float(op[-1, 0] - RE - plan.opt.targetAltitude),
        "closed_final_u": float(cl[-1, 2]),
        "open_final_u": float(op[-1, 2]),
        "closed_final_dw": float(cl[-1, 3] - plan.targetSpeed),
        "open_final_dw": float(op[-1, 3] - plan.targetSpeed),
        "qp_calls": int(np.sum(np.isfinite(np.atleast_1d(mpc.QPExitflags)))),
        "qp_fail": int(np.sum(np.atleast_1d(mpc.QPExitflags) <= 0)),
        "qp_mean_ms": float(np.atleast_1d(mpc.QPSolveSeconds).mean() * 1e3),
        "qp_max_ms": float(np.atleast_1d(mpc.QPSolveSeconds).max() * 1e3),
        "coast_samples": int(len(np.atleast_1d(coast.t))),
        "corr_max_delta_deg": float(np.degrees(np.abs(
            np.atleast_2d(mpc.corrections)[:, 0]).max())),
        "corr_max_dq": float(np.abs(np.atleast_2d(mpc.corrections)[:, 1]).max()),
        "slack_max": float(np.atleast_1d(mpc.altitudeSlack).max()),
    })
    with open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=1, ensure_ascii=False)
    print(json.dumps(summary, indent=1, ensure_ascii=False))
