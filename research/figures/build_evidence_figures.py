"""Rebuild evidence figures from recorded JSON and raw evaluator observations.

The old incident and later Mac-connected trace are distinct sessions, not
randomized before/after trials.  No intermediate old-run queue values exist.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


PROJECT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
DOCS = PROJECT / "data" / "reports"
ROUTE_RUN = PROJECT / "data" / "route"

BLUE = "#1D5F8A"
ORANGE = "#D16D30"
GREEN = "#27806A"
INK = "#203144"
GRID = "#D5DEE6"

plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 9,
        "axes.titlesize": 10,
        "axes.labelsize": 9,
        "axes.edgecolor": INK,
        "axes.labelcolor": INK,
        "text.color": INK,
        "xtick.color": INK,
        "ytick.color": INK,
        "savefig.facecolor": "white",
        "figure.facecolor": "white",
        "svg.fonttype": "none",
    }
)


def read_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as fh:
        return json.load(fh)


def format_axes(ax: plt.Axes) -> None:
    ax.spines[["top", "right"]].set_visible(False)
    ax.grid(axis="y", color=GRID, linewidth=0.65)
    ax.set_axisbelow(True)


def save(fig: plt.Figure, stem: str) -> None:
    for suffix in ("png", "svg"):
        path = OUT / f"{stem}.{suffix}"
        fig.savefig(path, dpi=300, bbox_inches="tight")
        if suffix == "svg":
            # Matplotlib emits trailing spaces inside multiline path data.
            # SVG treats newlines as separators, so this is visually identical.
            path.write_text("\n".join(line.rstrip() for line in path.read_text().splitlines()) + "\n")
    plt.close(fig)


def incident_comparison(old: dict, later: dict) -> None:
    old_data = old["diagnostic"]
    assert old_data["queue_before"] == 547
    assert old_data["queue_after"] == 598
    assert old_data["ack_delta"] == 0
    assert len(old_data["oldest_packet_delivery"]) == 12
    assert all(p["mac_acks_seen"] == 0 for p in old_data["oldest_packet_delivery"])
    assert all(p["telemetry_copies"] in (13, 14) for p in old_data["oldest_packet_delivery"])

    samples = later["samples"]
    t = np.array([row["wall"] - samples[0]["wall"] for row in samples])
    queued = np.array([row["queued"] for row in samples])
    assert len(samples) == 15
    assert int(queued.max()) == later["max_queue"] == 5

    fig, axes = plt.subplots(
        1, 2, figsize=(8.1, 3.35), gridspec_kw={"width_ratios": [0.78, 1.22]}
    )
    fig.suptitle("Outbox occupancy in two separate sessions", fontweight="bold", y=1.055)

    ax = axes[0]
    bars = ax.bar([0, 1], [old_data["queue_before"], old_data["queue_after"]],
                  color=["#8295A4", ORANGE], width=0.48)
    for bar, number in zip(bars, (547, 598)):
        ax.text(bar.get_x() + bar.get_width() / 2, number + 10, str(number),
                ha="center", va="bottom", fontweight="bold")
    ax.set_xticks([0, 1], ["First", "+15 s"])
    ax.set_ylim(0, 660)
    ax.set_ylabel("Queued envelopes")
    ax.set_title("A. Incident: RViz closed\n12 oldest: 13–14 copies, 0 ACKs", fontsize=9)
    format_axes(ax)

    ax = axes[1]
    ax.scatter(t, queued, color=BLUE, s=29, zorder=3)
    ax.set_ylim(-0.3, 6.1)
    ax.set_xlim(-1, 29)
    ax.set_yticks(range(0, 7))
    ax.set_xlabel("Wall time since first sample (s)")
    ax.set_ylabel("Queued envelopes")
    ax.set_title("B. Later Mac-connected trace (15 samples)")
    ax.text(0.98, 0.95, "Peak 5 · 0 reported drops", transform=ax.transAxes,
            ha="right", va="top", fontsize=8,
            bbox=dict(facecolor="white", edgecolor=GRID, boxstyle="round,pad=0.35"))
    format_axes(ax)

    fig.text(0.5, -0.01,
             "Different runs and viewing conditions; panel B is not a matched intervention trial. "
             "Points/bars are recorded values only.",
             ha="center", va="top", fontsize=7.7)
    fig.tight_layout(w_pad=2.3)
    save(fig, "incident_vs_later_queue")


def later_trace(later: dict) -> None:
    samples = later["samples"]
    t = np.array([row["wall"] - samples[0]["wall"] for row in samples])
    ack_delta = np.array([row["mac_acks"] - samples[0]["mac_acks"] for row in samples])
    pose_age = np.array([row["rviz_pose_age_sim_s"] for row in samples])
    assert ack_delta[-1] == later["mac_ack_delta"] == 97
    assert math.isclose(float(pose_age.max()), later["max_rviz_pose_age_sim_s"])
    assert all(row["rviz_state"] == "LIVE" for row in samples)
    assert all(row["dropped"] == 0 for row in samples)

    fig, axes = plt.subplots(2, 1, figsize=(7.2, 3.65), sharex=True,
                             gridspec_kw={"hspace": 0.28})
    fig.suptitle("Later 28 s Mac-connected observation", fontweight="bold", y=1.02)

    ax = axes[0]
    ax.plot(t, ack_delta, color=GREEN, linewidth=1.65, marker="o", markersize=3.8)
    ax.set_ylabel("Mac ACK counter\nchange (messages)")
    ax.set_ylim(-4, 106)
    ax.annotate("+97", xy=(t[-1], ack_delta[-1]), xytext=(-34, -18),
                textcoords="offset points", color=GREEN, fontweight="bold")
    format_axes(ax)

    ax = axes[1]
    ax.plot(t, pose_age, color=BLUE, linewidth=1.5, marker="o", markersize=3.8)
    ax.set_ylabel("RViz pose age\n(simulation s)")
    ax.set_xlabel("Wall time since first sample (s)")
    ax.set_xlim(-1, 29)
    ax.set_ylim(0, 0.55)
    ax.annotate("max 0.432 s", xy=(t[int(np.argmax(pose_age))], pose_age.max()),
                xytext=(10, 13), textcoords="offset points", color=BLUE, fontsize=8,
                arrowprops=dict(arrowstyle="-", color=BLUE, linewidth=0.7))
    format_axes(ax)

    fig.text(0.5, -0.025,
             "15 read-only diagnostic samples; all local RViz states LIVE. "
             "ACK counter covers gateway receipts, not unique packets.",
             ha="center", va="top", fontsize=7.7)
    fig.subplots_adjust(left=0.15, right=0.98, top=0.88, bottom=0.2, hspace=0.32)
    save(fig, "mac_ack_and_rviz_age_trace")


def route_accuracy(baseline: dict) -> None:
    start = baseline["start_stamp"]
    stop = baseline["checks"]["complete_route"]["details"]["stamp"]
    samples = []
    with (ROUTE_RUN / "pose-errors.jsonl").open(encoding="utf-8") as fh:
        for line in fh:
            row = json.loads(line)
            if start <= row["stamp"] <= stop:
                samples.append(row)
    assert len(samples) == baseline["accuracy"]["samples"] == 1742
    t = np.array([row["stamp"] - start for row in samples])
    error = np.array([row["position_error_m"] for row in samples])
    assert math.isclose(float(np.sqrt(np.mean(error ** 2))),
                        baseline["accuracy"]["rmse_m"], rel_tol=1e-12)
    # The original report uses the lower empirical order statistic, not
    # NumPy's interpolated percentile default.
    empirical_p95 = float(np.sort(error)[int(0.95 * (len(error) - 1))])
    assert math.isclose(empirical_p95,
                        baseline["accuracy"]["p95_m"], rel_tol=1e-12)

    route_id = baseline["request_id"]
    reached = []
    with (ROUTE_RUN / "mission-events.jsonl").open(encoding="utf-8") as fh:
        for line in fh:
            row = json.loads(line)
            if row.get("request_id") == route_id and row.get("event") == "checkpoint_reached":
                reached.append(row["stamp"] - start)
    assert len(reached) == 3
    arrivals = baseline["accuracy"]["arrivals"]
    arrival_error = [row["physical_error_m"] for row in arrivals]
    names = ["Warehouse", "Shelter", "Depot"]

    fig = plt.figure(figsize=(8.0, 4.25))
    gs = fig.add_gridspec(2, 1, height_ratios=[1.5, 1], hspace=0.47)
    ax = fig.add_subplot(gs[0])
    ax.plot(t, error, color=BLUE, linewidth=1.25)
    for i, (stamp, name) in enumerate(zip(reached, names)):
        ax.axvline(stamp, color=[GREEN, ORANGE, GREEN][i], linestyle="--", linewidth=0.95)
        label_x = stamp - 2 if i == 2 else stamp + 2
        ax.text(label_x, 4.06, name, fontsize=7.4, rotation=0, va="top",
                ha="right" if i == 2 else "left")
    ax.set_xlim(0, stop - start)
    ax.set_ylim(0, 4.25)
    ax.set_ylabel("Estimated vs physical\nposition error (m)")
    ax.set_xlabel("Simulation time since route request (s)")
    ax.set_title("A. Localization error over one complete inspection\n"
                 "1,742 samples · RMS 1.975 m · p95 3.673 m · max 3.875 m",
                 loc="left", fontsize=9)
    format_axes(ax)

    ax = fig.add_subplot(gs[1])
    colors = [GREEN, ORANGE, GREEN]
    bars = ax.bar(names, arrival_error, color=colors, width=0.46)
    for bar, value in zip(bars, arrival_error):
        ax.text(bar.get_x() + bar.get_width() / 2, value + 0.1, f"{value:.3f}",
                ha="center", va="bottom", fontsize=8, fontweight="bold")
    ax.set_ylim(0, 4.15)
    ax.set_ylabel("Physical arrival error\nto checkpoint (m)")
    ax.set_title("B. Evaluator-measured arrival distance", loc="left")
    format_axes(ax)

    fig.suptitle("Navigation success and measured geometric error", fontweight="bold", y=1.02)
    fig.text(0.5, -0.035,
             "Independent simulator truth is used for offline scoring only; it is not a twin input. "
             "One route, no confidence interval.",
             ha="center", va="top", fontsize=7.7)
    save(fig, "full_route_accuracy")


if __name__ == "__main__":
    incident_comparison(read_json(DOCS / "stale-telemetry-diagnosis.json"),
                        read_json(DOCS / "telemetry-freshness-validation.json"))
    later_trace(read_json(DOCS / "telemetry-freshness-validation.json"))
    route_accuracy(read_json(DOCS / "command-center-baseline.json"))
