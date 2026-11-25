#!/usr/bin/env python3
"""
Trajectory-tracking analysis for the PCB sorting runs (joint space vs task space).

Reads the per-trajectory CSV logs written by scripts/trajectory_node.py
(Milestone 04/Data/joint_space and .../task_space) and the matching launch.log,
and produces the figures and numbers used in the report:

    figures/quintic_profile.pdf      quintic time scaling vs cubic
    figures/flc_membership.pdf       the five fuzzy sets, with physical scales
    figures/flc_surface.pdf          fuzzy control surfaces u(e, e_dot)
    figures/joint_tracking.pdf       one joint-space leg: q_des vs q_act, errors
    figures/task_tracking.pdf        one task-space leg: X_des vs X_act, errors
    figures/path_comparison.pdf      tool path nest -> bin in both modes
    figures/path_deviation.pdf       off-line deviation of every long move
    generated/metrics.tex            LaTeX macros + summary table

Usage (from the package root, with the venv active):
    python3 "Milestone 04/Code/tracking_analysis.py" [output_folder]
(default output folder: /tmp/milestone04_analysis)
"""

import csv
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                  # noqa: E402

M4 = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(M4.parent / "scripts"))
from kinematics import ee_position                               # noqa: E402
from trajectory_node import (FuzzyLogicController,               # noqa: E402
                             plan_joint_space_trajectory)
from gui_node import PICK_PRESETS, TASK_PRESETS                   # noqa: E402

DATA = M4 / "Data"
# Output folder: first argument, or a temporary folder (nothing is written
# into the repository).
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/milestone04_analysis")
FIG = OUT / "figures"
GEN = OUT / "generated"

# ---- chart style: categorical slots in fixed order, recessive chrome ----
C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"]   # series 1..5
INK, INK2, MUTED, GRID = "#0b0b0b", "#52514e", "#898781", "#e1e0d9"
LS = ["-", "--", "-.", ":", (0, (5, 1, 1, 1))]                  # secondary encoding
plt.rcParams.update({
    "font.size": 9, "axes.titlesize": 9.5, "axes.labelsize": 9,
    "axes.edgecolor": "#c3c2b7", "axes.labelcolor": INK2, "axes.linewidth": 0.8,
    "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelcolor": INK2,
    "ytick.labelcolor": INK2, "axes.grid": True, "grid.color": GRID,
    "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
    "legend.frameon": False, "legend.fontsize": 8, "lines.linewidth": 1.6,
    "figure.dpi": 150, "savefig.bbox": "tight", "pdf.fonttype": 42,
})
JOINT_NAMES = ["$q_1$ base", "$q_2$ shoulder", "$q_3$ elbow", "$q_4$ wrist pitch", "$q_5$ wrist roll"]

JOINT_WP = {"ready": PICK_PRESETS[0][1], "grasp": PICK_PRESETS[1][1], "pass": PICK_PRESETS[2][1],
            "rework": PICK_PRESETS[3][1], "scrap": PICK_PRESETS[4][1], "home": (0, 0, 0, 0, 0)}
TASK_WP = {"ready": TASK_PRESETS[0][1], "grasp": TASK_PRESETS[1][1], "pass": TASK_PRESETS[2][1],
           "rework": TASK_PRESETS[3][1], "scrap": TASK_PRESETS[4][1]}


# ==============================================================================
# Loading and classifying the logs
# ==============================================================================

def load_csv(path):
    rows = list(csv.DictReader(open(path)))
    col = lambda k: np.array([float(r[k]) for r in rows])
    d = {"t": col("t"), "file": path.name, "mode": path.name.split("_")[1]}
    d["q_ref"] = np.column_stack([col(f"q_ref{i}") for i in range(5)])
    d["q_meas"] = np.column_stack([col(f"q_meas{i}") for i in range(5)])
    for k in ("x_ref", "x_des", "x_meas"):
        d[k] = np.column_stack([col(f"{k}_{a}") for a in "xyz"])
    return d


def nearest(point, table, tol):
    name, dist = min(((n, np.linalg.norm(np.asarray(point) - np.asarray(p)))
                      for n, p in table.items()), key=lambda x: x[1])
    return name if dist < tol else "?"


def label_leg(d):
    """'ready->pass' etc., from the start and end of the reference."""
    if d["mode"] == "joint":
        a = nearest(d["q_ref"][0], JOINT_WP, 0.06)
        b = nearest(d["q_ref"][-1], JOINT_WP, 0.02)
    else:
        a = nearest(d["x_ref"][0], TASK_WP, 0.006)
        b = nearest(d["x_ref"][-1], TASK_WP, 0.002)
    return f"{a}->{b}"


def motion_end(d):
    """Index where the planned reference stops moving (t = T)."""
    ref = d["q_ref"] if d["mode"] == "joint" else d["x_ref"]
    moving = np.where(np.abs(np.diff(ref, axis=0)).max(axis=1) > 1e-9)[0]
    return int(moving[-1]) + 1 if len(moving) else len(ref) - 1


def settle_info(log_path):
    """(settle time, final error) of every completed trajectory, in order."""
    txt = re.sub(r"\x1b\[[0-9;]*m", "", log_path.read_text(errors="ignore"))
    out = []
    for m in re.finditer(r"Trajectory complete \((settled in ([\d.]+)s|NOT settled[^)]*)\)\. "
                         r"Final \|error\| = ([\d.]+) (rad|m)", txt):
        out.append((float(m.group(2)) if m.group(2) else np.nan, float(m.group(3)), m.group(4)))
    return out


def load_run(name):
    folder = DATA / name
    legs = [load_csv(p) for p in sorted(folder.glob("traj_*.csv"))]
    done = settle_info(folder / "launch.log")
    for i, leg in enumerate(legs):
        # a trajectory cut off by shutdown has no "Trajectory complete" line
        leg["complete"] = i < len(done)
        if leg["complete"]:
            leg["settle"], leg["final_err"], leg["final_unit"] = done[i]
    legs = [l for l in legs if l["complete"]]
    for leg in legs:
        leg["label"] = label_leg(leg)
        leg["k_end"] = motion_end(leg)
    return legs


def off_line(X, a, b):
    """Distance of every point of X from the straight line a -> b."""
    u = (b - a) / np.linalg.norm(b - a)
    rel = X - a
    return np.linalg.norm(rel - np.outer(rel @ u, u), axis=1)


# ==============================================================================
# Metrics
# ==============================================================================

def metrics(legs, mode):
    own = [l for l in legs if l["mode"] == mode]
    rows = {"n": len(own)}
    if mode == "joint":
        e = [np.abs(l["q_ref"][:l["k_end"]] - l["q_meas"][:l["k_end"]]) for l in own]
        rows["max_e_joint"] = max(x.max() for x in e)                     # rad
        rows["rms_e_joint"] = np.sqrt(np.mean(np.concatenate([x.ravel() ** 2 for x in e])))
        ec = [np.linalg.norm(l["x_des"][:l["k_end"]] - l["x_meas"][:l["k_end"]], axis=1) for l in own]
    else:
        ec = [np.linalg.norm(l["x_ref"][:l["k_end"]] - l["x_meas"][:l["k_end"]], axis=1) for l in own]
    rows["max_e_tool"] = max(x.max() for x in ec) * 1000                  # mm
    rows["rms_e_tool"] = np.sqrt(np.mean(np.concatenate(ec) ** 2)) * 1000
    rows["settle_mean"] = np.nanmean([l["settle"] for l in own])
    rows["settle_max"] = np.nanmax([l["settle"] for l in own])
    rows["final_max"] = max(l["final_err"] for l in own)                  # rad or m
    long_legs = [l for l in own if any(b in l["label"] for b in ("pass", "rework", "scrap"))]
    dev = []
    for l in long_legs:
        X = l["x_meas"]
        ref = l["x_des"] if mode == "joint" else l["x_ref"]
        dev.append(off_line(X, ref[0], ref[-1]).max() * 1000)
    rows["dev_mean"], rows["dev_max"], rows["n_long"] = np.mean(dev), np.max(dev), len(dev)
    return rows


# ==============================================================================
# Figures
# ==============================================================================

def fig_quintic():
    tau = np.linspace(0, 1, 400)
    quint = [10 * tau**3 - 15 * tau**4 + 6 * tau**5,
             30 * tau**2 - 60 * tau**3 + 30 * tau**4,
             60 * tau - 180 * tau**2 + 120 * tau**3]
    cubic = [3 * tau**2 - 2 * tau**3, 6 * tau - 6 * tau**2, 6 - 12 * tau]
    fig, ax = plt.subplots(1, 3, figsize=(7.0, 2.2))
    titles = ["position $s(\\tau)$", "velocity $\\dot s\\,T$", "acceleration $\\ddot s\\,T^2$"]
    for i in range(3):
        ax[i].plot(tau, quint[i], color=C[0], label="quintic (used)")
        ax[i].plot(tau, cubic[i], color=C[1], ls="--", label="cubic")
        ax[i].set_title(titles[i], color=INK)
        ax[i].set_xlabel("$\\tau = t/T$")
    ax[2].annotate("cubic: acceleration\nstep at start and end", xy=(0.0, 6), xytext=(0.56, 4.0),
                   fontsize=7.5, color=INK2, arrowprops=dict(arrowstyle="-", color=MUTED))
    ax[0].legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(FIG / "quintic_profile.pdf")
    plt.close(fig)


def fig_flc_membership():
    x = np.linspace(-1.25, 1.25, 600)
    flc = FuzzyLogicController(1, 1, 1)
    mu = np.array([flc.fuzzify(np.clip(v, -1, 1)) for v in x])
    fig, ax = plt.subplots(figsize=(4.6, 1.9))
    for k, lab in enumerate(flc.LABELS):
        ax.plot(x, mu[:, k], color=C[k], ls=LS[k], label=lab)
        ax.text(flc.CENTERS[k], 1.06, lab, ha="center", color=INK2, fontsize=8)
    ax.set_ylim(0, 1.18)
    ax.set_xlabel("normalised input $e/K_e$, $\\dot e/K_{\\dot e}$  (saturated beyond $\\pm 1$)")
    ax.set_ylabel("membership $\\mu$")
    fig.tight_layout()
    fig.savefig(FIG / "flc_membership.pdf")
    plt.close(fig)


def fig_flc_surface():
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.8))
    cases = [(FuzzyLogicController(0.05, 2.0, 0.5), 0.08, 3.0, "rad", "rad/s", "rad/s",
              "joint-space FLC (each joint)"),
             (FuzzyLogicController(0.01, 0.30, 0.08), 0.016, 0.45, "m", "m/s", "m/s",
              "task-space FLC (each axis)")]
    for a, (f, emax, demax, ue, ude, uu, title) in zip(ax, cases):
        E, DE = np.meshgrid(np.linspace(-emax, emax, 161), np.linspace(-demax, demax, 161))
        U = np.vectorize(f)(E, DE)
        cs = a.contourf(E, DE, U, levels=17, cmap="RdBu_r", vmin=-f.Ku, vmax=f.Ku)
        a.contour(E, DE, U, levels=[0], colors=INK, linewidths=0.8)
        a.set_title(title, color=INK)
        a.set_xlabel(f"error $e$ [{ue}]")
        a.set_ylabel(f"error rate $\\dot e$ [{ude}]")
        a.grid(False)
        cb = fig.colorbar(cs, ax=a, fraction=0.05, pad=0.02)
        cb.set_label(f"output $u$ [{uu}]", color=INK2)
        cb.ax.tick_params(labelsize=7)
    fig.tight_layout()
    fig.savefig(FIG / "flc_surface.pdf")
    plt.close(fig)


def pick(legs, mode, label):
    for l in legs:
        if l["mode"] == mode and l["label"] == label:
            return l
    raise LookupError(f"no {mode} leg {label}")


def fig_joint_tracking(leg):
    t = leg["t"]
    fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.4), sharex=True)
    for i in range(5):
        ax[0].plot(t, leg["q_ref"][:, i], color=C[i], ls="--", lw=1.1)
        ax[0].plot(t, leg["q_meas"][:, i], color=C[i], ls="-", label=JOINT_NAMES[i])
        ax[1].plot(t, (leg["q_ref"][:, i] - leg["q_meas"][:, i]) * 1000, color=C[i], ls=LS[i],
                   label=JOINT_NAMES[i])
    T = t[leg["k_end"]]
    for a in ax:
        a.axvline(T, color=MUTED, lw=0.8, ls=":")
    ax[0].text(T, ax[0].get_ylim()[1], "  t = T", color=INK2, fontsize=7.5, va="top")
    ax[0].set_ylabel("joint angle [rad]")
    ax[0].set_title("planned $q_{des}$ (dashed) vs measured $q_{act}$ (solid)", color=INK)
    ax[0].legend(ncol=5, loc="upper center", bbox_to_anchor=(0.5, -0.02), fontsize=7.5)
    ax[1].set_ylabel("$q_e = q_{des}-q_{act}$ [mrad]")
    ax[1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(FIG / "joint_tracking.pdf")
    plt.close(fig)


def fig_task_tracking(leg):
    t = leg["t"]
    fig, ax = plt.subplots(2, 1, figsize=(7.0, 4.2), sharex=True)
    for k, a in enumerate("xyz"):
        ax[0].plot(t, leg["x_ref"][:, k] * 1000, color=C[k], ls="--", lw=1.1)
        ax[0].plot(t, leg["x_meas"][:, k] * 1000, color=C[k], label=f"${a}$")
        ax[1].plot(t, (leg["x_ref"][:, k] - leg["x_meas"][:, k]) * 1000, color=C[k], ls=LS[k],
                   label=f"${a}_e$")
    ax[1].plot(t, np.linalg.norm(leg["x_ref"] - leg["x_meas"], axis=1) * 1000, color=INK,
               lw=1.0, label="$\\|X_e\\|$")
    T = t[leg["k_end"]]
    for a in ax:
        a.axvline(T, color=MUTED, lw=0.8, ls=":")
    ax[0].set_ylabel("tool position [mm]")
    ax[0].set_title("planned $X_{des}$ (dashed) vs measured $f(q_{act})$ (solid)", color=INK)
    ax[0].legend(loc="center left", bbox_to_anchor=(1.01, 0.5))
    ax[1].set_ylabel("$X_e$ [mm]")
    ax[1].set_xlabel("time [s]")
    ax[1].legend(loc="center left", bbox_to_anchor=(1.01, 0.5))
    fig.tight_layout()
    fig.savefig(FIG / "task_tracking.pdf")
    plt.close(fig)


def fig_path_comparison(jleg, tleg):
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 3.0))
    for leg, col, ls, name in ((jleg, C[0], "-", "joint space"), (tleg, C[1], "--", "task space")):
        X = leg["x_meas"] * 1000
        ax[0].plot(X[:, 0], X[:, 1], color=col, ls=ls, label=name)
        a, b = X[0], X[-1]
        horiz = np.linalg.norm(X[:, :2] - a[:2], axis=1)
        ax[1].plot(horiz, X[:, 2], color=col, ls=ls, label=name)
    for leg in (jleg, tleg):
        ref = (leg["x_des"] if leg["mode"] == "joint" else leg["x_ref"]) * 1000
        a, b = ref[0], ref[-1]
        ax[0].plot([a[0], b[0]], [a[1], b[1]], color=MUTED, lw=0.9, ls=":")
        ax[1].plot([0, np.linalg.norm(b[:2] - a[:2])], [a[2], b[2]], color=MUTED, lw=0.9, ls=":")
    ax[0].plot([], [], color=MUTED, lw=0.9, ls=":", label="straight line")
    ax[0].scatter([-180], [0], s=30, color=INK2, zorder=3)
    ax[0].annotate("nest", (-180, 0), xytext=(4, 4), textcoords="offset points", color=INK2, fontsize=8)
    ax[0].scatter([-100], [-130], s=30, color=INK2, marker="s", zorder=3)
    ax[0].annotate("PASS bin", (-100, -130), xytext=(4, -10), textcoords="offset points",
                   color=INK2, fontsize=8)
    ax[0].set_xlabel("x [mm]")
    ax[0].set_ylabel("y [mm]")
    ax[0].set_title("top view", color=INK)
    ax[0].set_aspect("equal", adjustable="datalim")
    ax[0].legend(loc="lower left", fontsize=7.5)
    ax[1].set_xlabel("horizontal distance from start [mm]")
    ax[1].set_ylabel("z [mm]")
    ax[1].set_title("side view (along the move)", color=INK)
    fig.tight_layout()
    fig.savefig(FIG / "path_comparison.pdf")
    plt.close(fig)


def fig_path_deviation(legs):
    order = ["ready->pass", "pass->ready", "ready->rework", "rework->ready",
             "ready->scrap", "scrap->ready"]
    fig, ax = plt.subplots(figsize=(7.0, 2.6))
    w = 0.38
    for m, (mode, col) in enumerate((("joint", C[0]), ("task", C[1]))):
        vals = []
        for lab in order:
            dev = [off_line(l["x_meas"], *(lambda r: (r[0], r[-1]))(
                       l["x_des"] if mode == "joint" else l["x_ref"])).max() * 1000
                   for l in legs if l["mode"] == mode and l["label"] == lab]
            vals.append(max(dev) if dev else np.nan)
        x = np.arange(len(order)) + (m - 0.5) * w
        bars = ax.bar(x, vals, width=w - 0.04, color=col, label=f"{mode} space",
                      hatch=None if mode == "joint" else "////", edgecolor="white", linewidth=0)
        for xi, v in zip(x, vals):
            if np.isfinite(v):
                ax.text(xi, v + 0.3, f"{v:.1f}", ha="center", fontsize=7, color=INK2)
    ax.set_xticks(np.arange(len(order)))
    ax.set_xticklabels([o.replace("->", " → ") for o in order], fontsize=8)
    ax.set_ylabel("max distance from\nstraight line [mm]")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(FIG / "path_deviation.pdf")
    plt.close(fig)


# ==============================================================================

def main():
    FIG.mkdir(parents=True, exist_ok=True)
    GEN.mkdir(parents=True, exist_ok=True)
    jlegs = load_run("joint_space")
    tlegs = load_run("task_space")
    mj, mt = metrics(jlegs, "joint"), metrics(tlegs, "task")

    fig_quintic()
    fig_flc_membership()
    fig_flc_surface()
    jleg, tleg = pick(jlegs, "joint", "ready->pass"), pick(tlegs, "task", "ready->pass")
    fig_joint_tracking(jleg)
    fig_task_tracking(tleg)
    fig_path_comparison(jleg, tleg)
    fig_path_deviation(jlegs + tlegs)

    boards = lambda run: len(re.findall(r"Released pcb_\d \(\w+\) into the \w+ bin - correct",
                                        (DATA / run / "launch.log").read_text(errors="ignore")))
    macros = {
        "JNlegs": f"{mj['n']}", "TNlegs": f"{mt['n']}",
        "JmaxEq": f"{mj['max_e_joint'] * 1000:.1f}", "JrmsEq": f"{mj['rms_e_joint'] * 1000:.2f}",
        "JmaxEx": f"{mj['max_e_tool']:.2f}", "JrmsEx": f"{mj['rms_e_tool']:.2f}",
        "TmaxEx": f"{mt['max_e_tool']:.2f}", "TrmsEx": f"{mt['rms_e_tool']:.2f}",
        "Jsettle": f"{mj['settle_mean']:.2f}", "Tsettle": f"{mt['settle_mean']:.2f}",
        "JsettleMax": f"{mj['settle_max']:.2f}", "TsettleMax": f"{mt['settle_max']:.2f}",
        "Jfinal": f"{mj['final_max'] * 1000:.2f}", "Tfinal": f"{mt['final_max'] * 1000:.2f}",
        "Jdev": f"{mj['dev_mean']:.1f}", "JdevMax": f"{mj['dev_max']:.1f}",
        "Tdev": f"{mt['dev_mean']:.1f}", "TdevMax": f"{mt['dev_max']:.1f}",
        "Jlong": f"{mj['n_long']}", "Tlong": f"{mt['n_long']}",
        "Jboards": f"{boards('joint_space')}", "Tboards": f"{boards('task_space')}",
    }
    with open(GEN / "metrics.tex", "w") as fh:
        fh.write("% generated by Code/tracking_analysis.py - do not edit\n")
        for k, v in macros.items():
            fh.write(f"\\newcommand{{\\{k}}}{{{v}}}\n")
    for k, v in macros.items():
        print(f"{k:12s} {v}")


if __name__ == "__main__":
    main()
