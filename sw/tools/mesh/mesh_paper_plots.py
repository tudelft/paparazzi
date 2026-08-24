#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2026 The Paparazzi Team
#
# This file is part of paparazzi.
#
# Paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
"""Generate reproducible figures for the broadcast-mesh design papers."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Sequence

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

import mesh_coded_sim as coded
import mesh_link_budget as link_budget


BLUE = "#0072B2"
VERMILLION = "#D55E00"
GREEN = "#009E73"
CHARCOAL = "#20262E"
GRID = "#D9DEE5"
PAPER = "#FFFFFF"

OUTPUT_DIR = Path(__file__).resolve().parents[3] / "doc" / "mesh" / "figures"


def configure_style() -> None:
    mpl.rcParams.update({
        "figure.facecolor": PAPER,
        "axes.facecolor": PAPER,
        "axes.edgecolor": CHARCOAL,
        "axes.labelcolor": CHARCOAL,
        "axes.titlecolor": CHARCOAL,
        "axes.titleweight": "bold",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.axisbelow": True,
        "grid.color": GRID,
        "grid.linewidth": 0.8,
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "xtick.color": CHARCOAL,
        "ytick.color": CHARCOAL,
        "legend.frameon": False,
        "lines.linewidth": 2.4,
        "savefig.facecolor": PAPER,
        "savefig.bbox": "tight",
        "svg.fonttype": "none",
        "svg.hashsalt": "paparazzi-mesh-paper-plots",
    })


def save_figure(figure: plt.Figure, stem: str) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    metadata = {"Creator": "Paparazzi mesh_paper_plots.py", "Date": None}
    figure.savefig(OUTPUT_DIR / f"{stem}.svg", metadata=metadata)
    figure.savefig(OUTPUT_DIR / f"{stem}.png", dpi=300,
                   metadata={"Software": "Paparazzi mesh_paper_plots.py"})
    plt.close(figure)


def plot_flighted_rate_scaling() -> None:
    aircraft = np.arange(1, 65)
    peers = aircraft + 1
    rates = np.minimum(8.0, 25.0 / peers) / 16.0

    figure, axis = plt.subplots(figsize=(8.2, 4.6), layout="constrained")
    axis.axvspan(1, 9, color=GREEN, alpha=0.08)
    axis.axvspan(9, 16, color=BLUE, alpha=0.07)
    axis.axvspan(16, 64, color=VERMILLION, alpha=0.055)
    axis.plot(aircraft, rates, color=BLUE)
    axis.fill_between(aircraft, rates, color=BLUE, alpha=0.10)

    for count in (9, 16, 64):
        rate = min(8.0, 25.0 / (count + 1)) / 16.0
        axis.scatter(count, rate, s=48, color=VERMILLION, edgecolor=PAPER,
                     linewidth=1.2, zorder=3)
        horizontal = "right" if count == 64 else "left"
        offset = -1.2 if count == 64 else 1.2
        axis.annotate(f"{count} aircraft\n{rate:.3f} Hz mean",
                      (count, rate), xytext=(count + offset, rate + 0.045),
                      ha=horizontal, va="bottom", fontsize=9,
                      arrowprops={"arrowstyle": "-", "color": CHARCOAL,
                                  "linewidth": 0.8})

    axis.text(4.8, 0.49, "common", color=GREEN, ha="center",
              fontsize=8.5, fontweight="bold")
    axis.text(12.5, 0.465, "normal\nmaximum", color=BLUE, ha="center",
              fontsize=8.5, fontweight="bold", linespacing=0.9)
    axis.text(40, 0.49, "graceful-degradation scope", color=VERMILLION,
              ha="center", fontsize=8.5, fontweight="bold")
    axis.set(xlim=(1, 64), ylim=(0, 0.54), xlabel="Aircraft in fleet",
             ylabel="Mean scheduled state-update rate per peer (Hz)")
    axis.set_xticks([1, 9, 16, 25, 40, 64])
    axis.grid(axis="y")
    axis.set_title("Fixed channel capacity forces lower per-peer update rates")
    axis.text(0.0, -0.22,
              "Model: 25 fair slots per 16 s superframe; GCS included as one peer; "
              "maximum 8 slots per peer.",
              transform=axis.transAxes, fontsize=8.5, color="#4E5965")
    save_figure(figure, "flighted_rate_scaling")


def plot_rf_margin() -> None:
    budget = link_budget.LinkBudget(
        prop=link_budget.Propagation(frequency_mhz=434.125, ground="average",
                                     surface="grass"),
        gcs_mast_m=4.0,
    )
    distances_km = np.linspace(0.5, 20.0, 280)
    air_air = np.array([
        budget.margin_at_m(distance * 1000.0, 100.0, 100.0)
        for distance in distances_km
    ])
    air_gcs = np.array([
        budget.margin_at_m(distance * 1000.0, 100.0, 4.0)
        for distance in distances_km
    ])
    air_air_range = budget.design_range_m(100.0, 100.0) / 1000.0
    air_gcs_range = budget.design_range_m(100.0, 4.0) / 1000.0
    assert math.isclose(air_air_range, 12.30, abs_tol=0.02)
    assert math.isclose(air_gcs_range, 9.17, abs_tol=0.02)

    figure, axis = plt.subplots(figsize=(8.2, 4.8), layout="constrained")
    axis.axhspan(-22, 0, color=VERMILLION, alpha=0.055)
    axis.axhline(0, color=CHARCOAL, linewidth=1.1)
    axis.plot(distances_km, air_air, color=BLUE,
              label="Air-to-air: 100 m / 100 m AGL")
    axis.plot(distances_km, air_gcs, color=VERMILLION,
              label="Air-to-GCS: 100 m / 4 m mast")

    for distance, color, label, y_offset in (
            (air_air_range, BLUE, "12.30 km", 3.8),
            (air_gcs_range, VERMILLION, "9.17 km", -6.3)):
        axis.scatter(distance, 0, s=50, color=color, edgecolor=PAPER,
                     linewidth=1.2, zorder=4)
        axis.annotate(label, (distance, 0), xytext=(distance + 0.5, y_offset),
                      color=color, fontsize=9, fontweight="bold",
                      arrowprops={"arrowstyle": "-", "color": color,
                                  "linewidth": 0.8})

    axis.text(19.6, 1.0, "link closes", ha="right", va="bottom",
              fontsize=8.5, color=GREEN, fontweight="bold")
    axis.text(19.6, -1.0, "relay or shorter edge required", ha="right",
              va="top", fontsize=8.5, color=VERMILLION, fontweight="bold")
    axis.set(xlim=(0.5, 20), ylim=(-22, 35), xlabel="Separation (km)",
             ylabel="Spare link margin above charged fade reserve (dB)")
    axis.set_xticks([1, 4, 8, 12, 16, 20])
    axis.grid(axis="both")
    axis.legend(loc="upper right")
    axis.set_title("The low GCS antenna makes the ground leg the range limit")
    axis.text(0.0, -0.22,
              "Analytical model: 434.125 MHz, 10 dBm EIRP, 45 deg bank, "
              "10 dB fade reserve, average grass-covered ground.",
              transform=axis.transAxes, fontsize=8.5, color="#4E5965")
    save_figure(figure, "flighted_rf_margin")


def coded_metrics(arguments: Sequence[str]) -> tuple[coded.Metrics, coded.Metrics,
                                                      coded.argparse.Namespace]:
    args = coded.parse_args(arguments)
    return coded.run_legacy(args), coded.run_coded(args), args


def metric_values(metrics: coded.Metrics, args: coded.argparse.Namespace,
                  member_count: int) -> tuple[float, float, float]:
    possible = metrics.originated * (member_count - 1)
    delivery = len(metrics.delivered) / max(1, possible)
    p95 = coded.percentile(metrics.ages_ms, 0.95)
    airtime = metrics.transmissions * args.air_time_ms / (args.duration * 10.0)
    return delivery, p95, airtime


def plot_coded_tradeoff() -> None:
    ids = "0,3,19,42,58,77,101,125,251"
    legacy, gossip, args = coded_metrics(
        ["--ac-ids", ids, "--duration", "120", "--seed", "7"])
    values = np.array([
        metric_values(legacy, args, 9),
        metric_values(gossip, args, 9),
    ])
    assert np.allclose(values, [[1.0, 5.44, 25.1], [0.95, 2.67, 30.8]],
                       atol=[0.0005, 0.005, 0.05])

    figure, axes = plt.subplots(1, 3, figsize=(10.2, 3.9),
                                layout="constrained")
    labels = ("TDMA flood", "Coded gossip")
    colors = (BLUE, VERMILLION)
    specs = (
        (values[:, 1], "p95 state age", "Seconds (lower is better)", 6.2, "{:.2f}"),
        (values[:, 0] * 100.0, "Unique delivery", "Percent (higher is better)",
         105.0, "{:.1f}%"),
        (values[:, 2], "Aggregate airtime", "Percent (lower is better)",
         35.0, "{:.1f}%"),
    )
    for index, (axis, spec) in enumerate(zip(axes, specs)):
        data, title, xlabel, maximum, value_format = spec
        positions = np.arange(2)
        axis.barh(positions, data, color=colors, height=0.52)
        axis.set(xlim=(0, maximum), yticks=positions, yticklabels=labels,
                 xlabel=xlabel)
        axis.invert_yaxis()
        axis.grid(axis="x")
        axis.set_title(f"{chr(ord('A') + index)}   {title}", loc="left")
        for position, value in zip(positions, data):
            axis.text(value + maximum * 0.025, position,
                      value_format.format(value), va="center", fontsize=9,
                      fontweight="bold", color=CHARCOAL)
    figure.suptitle("Coded gossip trades completeness and airtime for fresher state",
                    fontsize=13, fontweight="bold", color=CHARCOAL)
    save_figure(figure, "coded_tradeoff")


def plot_coded_fleet_scaling() -> None:
    fleet_sizes = (9, 16, 64)
    legacy_values = []
    gossip_values = []
    for fleet_size in fleet_sizes:
        legacy, gossip, args = coded_metrics(
            ["--aircraft", str(fleet_size), "--duration", "120", "--seed", "7"])
        legacy_values.append(metric_values(legacy, args, fleet_size + 1))
        gossip_values.append(metric_values(gossip, args, fleet_size + 1))
    legacy_array = np.array(legacy_values)
    gossip_array = np.array(gossip_values)

    figure, axes = plt.subplots(1, 2, figsize=(9.4, 4.1),
                                layout="constrained")
    panels = (
        (legacy_array[:, 1], gossip_array[:, 1], "p95 state age",
         "Seconds (lower is better)"),
        (legacy_array[:, 2], gossip_array[:, 2], "Aggregate airtime",
         "Percent across all radios"),
    )
    for index, (axis, panel) in enumerate(zip(axes, panels)):
        legacy_data, gossip_data, title, ylabel = panel
        axis.plot(fleet_sizes, legacy_data, marker="o", markersize=7,
                  color=BLUE, label="Idealized TDMA flood")
        axis.plot(fleet_sizes, gossip_data, marker="s", markersize=7,
                  color=VERMILLION, label="Coded gossip")
        axis.set(xlabel="Aircraft in fleet", ylabel=ylabel,
                 xticks=fleet_sizes, xlim=(7, 66), ylim=(0, None))
        axis.grid(axis="both")
        axis.set_title(f"{chr(ord('A') + index)}   {title}", loc="left")
    axes[1].axhline(100, color=CHARCOAL, linewidth=1.0, linestyle="--")
    axes[1].text(63.5, 103, "one full channel-equivalent", ha="right",
                 va="bottom", fontsize=8.5, color=CHARCOAL)
    axes[0].legend(loc="upper left")
    figure.suptitle("A single seed exposes the scale-dependent tradeoff",
                    fontsize=13, fontweight="bold", color=CHARCOAL)
    save_figure(figure, "coded_fleet_scaling")


def main() -> None:
    configure_style()
    plot_flighted_rate_scaling()
    plot_rf_margin()
    plot_coded_tradeoff()
    plot_coded_fleet_scaling()
    print(f"wrote publication figures to {OUTPUT_DIR}")


if __name__ == "__main__":
    main()