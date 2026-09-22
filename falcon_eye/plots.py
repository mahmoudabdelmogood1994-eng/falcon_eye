"""
plots.py
Performance graphs (spec section 13): single-scenario traces and
Monte Carlo comparison charts across fog density / sensing mode.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .simulation import SimulationResult
from .multi_target_simulation import MultiTargetSimulationResult


def plot_scenario_trace(result: SimulationResult, title: str, savepath: str):
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)

    ax = axes[0]
    ax.plot(result.time, result.true_range, label="True range", color="black", lw=1.5)
    ax.plot(result.time, result.tracked_range, label="Tracked (Falcon Eye) range",
            color="tab:blue", lw=1.2, alpha=0.85)
    det_times = result.time[result.detected & ~result.false_detection]
    det_ranges = result.tracked_range[result.detected & ~result.false_detection]
    ax.scatter(det_times, det_ranges, s=8, color="tab:green", label="Valid detections", zorder=5)
    false_times = result.time[result.false_detection]
    false_ranges = result.tracked_range[result.false_detection]
    if len(false_times):
        ax.scatter(false_times, false_ranges, s=10, color="tab:red", marker="x",
                   label="False detections", zorder=5)
    if result.warning_time is not None:
        ax.axvline(result.warning_time, color="orange", ls="--", label="Warning issued")
    if result.collision_time is not None:
        ax.axvline(result.collision_time, color="darkred", ls=":", label="Collision point")
    ax.set_ylabel("Range (m)")
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title(title)

    ax = axes[1]
    risk_numeric = np.array([{"none": 0, "advisory": 1, "warning": 2}[r] for r in result.risk_level])
    ax.step(result.time, risk_numeric, where="post", color="tab:purple")
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["none", "advisory", "warning"])
    ax.set_ylabel("Risk level")

    ax = axes[2]
    ax.plot(result.time, result.true_ttc, color="gray", label="True TTC")
    ax.set_ylim(0, 120)
    ax.set_ylabel("TTC (s)")
    ax.set_xlabel("Time (s)")
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(savepath, dpi=140)
    plt.close(fig)


def plot_multi_target_trace(result: MultiTargetSimulationResult, title: str, savepath: str):
    """Per-target tracked range over time, colored by risk level, with
    reflex-trigger events marked."""
    risk_colors = {"CLEAR": "tab:gray", "ADVISORY": "gold", "WARNING": "tab:orange", "CRITICAL_REFLEX": "tab:red"}

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True,
                              gridspec_kw={"height_ratios": [3, 1]})

    ax = axes[0]
    for tid in result.target_ids:
        tr = result.tracked_range[tid]
        risks = result.risk_level[tid]
        t = result.time
        ax.plot(t, result.true_range[tid], color="black", lw=0.6, alpha=0.35)
        # color the tracked line by risk level using scatter segments
        for level, color in risk_colors.items():
            mask = np.array([r == level for r in risks]) & ~np.isnan(tr)
            if mask.any():
                ax.scatter(t[mask], tr[mask], s=6, color=color, label=f"{level}" if tid == result.target_ids[0] else None)
        ax.text(t[-1] * 1.01, np.nanmax(tr) if np.any(~np.isnan(tr)) else 0,
                f"T{tid}", fontsize=8, va="center")

    for (rt, rid) in result.reflex_events:
        ax.axvline(rt, color="darkred", ls="--", lw=1, alpha=0.7)
        ax.text(rt, ax.get_ylim()[1] * 0.95, f"reflex\nT{rid}", fontsize=7, color="darkred", ha="center")

    ax.set_ylabel("Range (m)\n(dots = tracked, colored by risk; thin line = true range)")
    ax.set_title(title)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(fontsize=8, loc="upper right")

    ax2 = axes[1]
    id_to_y = {tid: idx for idx, tid in enumerate(result.target_ids)}
    raw_y = [id_to_y.get(tid, np.nan) if tid is not None else np.nan for tid in result.top_priority_target]
    stable_y = [id_to_y.get(tid, np.nan) if tid is not None else np.nan for tid in result.stable_priority_target]
    ax2.step(result.time, raw_y, where="post", color="tab:purple", alpha=0.35, lw=1.2, label="raw slack-sort #1 (unfiltered)")
    ax2.step(result.time, stable_y, where="post", color="tab:purple", lw=2.2, label="hysteresis-stabilized")
    ax2.set_yticks(list(id_to_y.values()))
    ax2.set_yticklabels([f"T{tid}" for tid in id_to_y.keys()])
    ax2.set_ylabel("Top priority")
    ax2.set_xlabel("Time (s)")
    ax2.legend(fontsize=7, loc="upper right")

    fig.tight_layout()
    fig.savefig(savepath, dpi=140)
    plt.close(fig)


def plot_mode_fog_comparison(results: dict, metric_name: str, ylabel: str, title: str, savepath: str):
    """results: dict[mode] -> dict[fog_key] -> AggregateMetrics"""
    fog_order = ["clear", "light", "moderate", "dense"]
    modes = list(results.keys())
    x = np.arange(len(fog_order))
    width = 0.8 / len(modes)

    fig, ax = plt.subplots(figsize=(8, 5))
    colors = {"lidar": "tab:blue", "radar": "tab:orange", "fusion": "tab:green"}

    for i, mode in enumerate(modes):
        values = []
        for fog_key in fog_order:
            agg = results[mode][fog_key]
            v = getattr(agg, metric_name)
            values.append(v if v is not None and not (isinstance(v, float) and np.isnan(v)) else 0.0)
        ax.bar(x + i * width - width * (len(modes) - 1) / 2, values, width,
               label=mode.upper(), color=colors.get(mode, None))

    ax.set_xticks(x)
    ax.set_xticklabels([f.capitalize() for f in fog_order])
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(savepath, dpi=140)
    plt.close(fig)
