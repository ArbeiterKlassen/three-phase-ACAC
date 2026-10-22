#!/usr/bin/env python3
"""Plot the reproducible results emitted by run_acdc_halfbridge_sim.ps1."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def configure_style() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 140,
            "savefig.dpi": 160,
            "axes.grid": True,
            "grid.alpha": 0.28,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "font.size": 9,
            "legend.fontsize": 8,
        }
    )


def load_summary(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        return list(csv.DictReader(handle))


def number(row: dict[str, str], key: str) -> float:
    return float(row[key])


def plot_sweep(rows: list[dict[str, str]], out_path: Path) -> None:
    limit = sorted(
        (row for row in rows if row["case"].startswith("limit-")),
        key=lambda row: number(row, "ilimit"),
    )
    inputs = [
        row
        for row in rows
        if row["case"].startswith("input-")
        and (row["case"].endswith("limit3p8") or row["case"].endswith("limit5p5"))
    ]
    lc = [row for row in rows if row["case"].startswith("lc-")]

    fig, axes = plt.subplots(2, 2, figsize=(11.5, 7.2), constrained_layout=True)

    ax = axes[0, 0]
    ax.axhspan(
        62 * 0.98,
        62 * 1.02,
        color="#b7d7a8",
        alpha=0.45,
        label="62 V +/-2%",
    )
    for suffix, marker, color, label in (
        ("limit3p8", "o", "#d95f02", "limit 3.8 A"),
        ("limit5p5", "s", "#1b9e77", "limit 5.5 A"),
    ):
        part = sorted(
            (row for row in inputs if row["case"].endswith(suffix)),
            key=lambda row: number(row, "vac"),
        )
        ax.plot(
            [number(row, "vac") for row in part],
            [number(row, "vbus_mean") for row in part],
            marker=marker,
            color=color,
            label=label,
        )
    ax.set_xlabel("Input voltage (V RMS)")
    ax.set_ylabel("Bus voltage mean (V)")
    ax.set_title("Full-load bus regulation")
    ax.legend()

    ax = axes[0, 1]
    ax.axhspan(62 * 0.98, 62 * 1.02, color="#b7d7a8", alpha=0.45)
    ax.plot(
        [number(row, "ilimit") for row in limit],
        [number(row, "vbus_mean") for row in limit],
        marker="o",
        color="#2c7fb8",
        label="Vbus mean",
    )
    ax.plot(
        [number(row, "ilimit") for row in limit],
        [number(row, "vbus_ripple") for row in limit],
        marker="s",
        color="#e6ab02",
        label="Vbus peak-to-peak",
    )
    ax.set_xlabel("Outer current command limit (A RMS)")
    ax.set_ylabel("Voltage (V)")
    ax.set_title("36 V RMS / 1.8 A: limit sweep")
    ax.legend()

    ax = axes[1, 0]
    ax.plot(
        [number(row, "ilimit") for row in limit],
        [number(row, "pf") for row in limit],
        marker="o",
        color="#1b9e77",
        label="PF",
    )
    ax.set_xlabel("Outer current command limit (A RMS)")
    ax.set_ylabel("Power factor")
    ax.set_ylim(min(0.75, min(number(row, "pf") for row in limit) - 0.02), 1.01)
    ax2 = ax.twinx()
    ax2.plot(
        [number(row, "ilimit") for row in limit],
        [number(row, "thd") for row in limit],
        marker="s",
        color="#d95f02",
        label="Current THD",
    )
    ax2.set_ylabel("Current THD h2..h15 (%)")
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="lower right")
    ax.set_title("Power quality with current headroom")

    ax = axes[1, 1]
    scatter = ax.scatter(
        [number(row, "cf_uf") for row in lc],
        [number(row, "lf_mh") for row in lc],
        c=[number(row, "vbus_mean") for row in lc],
        s=90,
        cmap="viridis",
        vmin=58,
        vmax=63,
        edgecolors="black",
        linewidths=0.35,
    )
    for row in lc:
        ax.annotate(
            f"PF {number(row, 'pf'):.3f}\nTHD {number(row, 'thd'):.1f}%",
            (number(row, "cf_uf"), number(row, "lf_mh")),
            xytext=(3, 3),
            textcoords="offset points",
            fontsize=6,
        )
    ax.set_xscale("log")
    ax.set_xlabel("Cf (uF)")
    ax.set_ylabel("Lf (mH)")
    ax.set_title("LC sweep at 36 V RMS / 1.8 A / 5.5 A limit")
    fig.colorbar(scatter, ax=ax, label="Vbus mean (V)")

    fig.savefig(out_path)
    plt.close(fig)


def plot_bus_trace(path: Path, label: str, color: str, ax_bus: plt.Axes) -> None:
    data = np.genfromtxt(path, delimiter=",", names=True)
    t = data["time_s"] - data["time_s"][0]
    mask = t <= 0.10
    ax_bus.plot(
        t[mask] * 1e3,
        data["vbus_v"][mask],
        color=color,
        linewidth=1.0,
        label=label,
    )


def plot_waveforms(path_38: Path, path_55: Path, out_path: Path) -> None:
    data = np.genfromtxt(path_55, delimiter=",", names=True)
    t = data["time_s"] - data["time_s"][0]
    mask = t <= 0.10

    fig, axes = plt.subplots(3, 1, figsize=(11.5, 7.8), sharex=True, constrained_layout=True)
    axes[0].plot(t[mask] * 1e3, data["vac_v"][mask], color="#4c78a8", label="Vac")
    axes[0].set_ylabel("Vac (V)")
    axes[0].set_title("Closed-loop waveforms at 36 V RMS, 1.8 A load, 5.5 A command limit")
    current_axis = axes[0].twinx()
    current_axis.plot(t[mask] * 1e3, data["iac_a"][mask], color="#f58518", label="Iac")
    current_axis.set_ylabel("Iac (A)")
    handles1, labels1 = axes[0].get_legend_handles_labels()
    handles2, labels2 = current_axis.get_legend_handles_labels()
    axes[0].legend(handles1 + handles2, labels1 + labels2, ncol=2, loc="upper right")

    axes[1].plot(t[mask] * 1e3, data["vcf_v"][mask], color="#54a24b", label="vCf")
    axes[1].set_ylabel("vCf (V)")
    boost_axis = axes[1].twinx()
    boost_axis.plot(t[mask] * 1e3, data["ilb_a"][mask], color="#b279a2", label="iLb")
    boost_axis.plot(t[mask] * 1e3, data["iref_a"][mask], color="#e45756", label="iRef (Lb)")
    boost_axis.set_ylabel("Boost current (A)")
    handles1, labels1 = axes[1].get_legend_handles_labels()
    handles2, labels2 = boost_axis.get_legend_handles_labels()
    axes[1].legend(handles1 + handles2, labels1 + labels2, ncol=3, loc="upper right")

    plot_bus_trace(path_38, "3.8 A limit", "#9c755f", axes[2])
    plot_bus_trace(path_55, "5.5 A limit", "#1b9e77", axes[2])
    duty_axis = axes[2].twinx()
    duty_axis.plot(
        t[mask] * 1e3,
        data["d_low"][mask],
        color="#2c7fb8",
        alpha=0.65,
        label="Low-side d",
    )
    axes[2].set_xlabel("Time within 100 ms window (ms)")
    axes[2].set_ylabel("Vbus (V)")
    duty_axis.set_ylabel("Low-side duty")
    handles1, labels1 = axes[2].get_legend_handles_labels()
    handles2, labels2 = duty_axis.get_legend_handles_labels()
    axes[2].legend(handles1 + handles2, labels1 + labels2, ncol=3, loc="upper right")

    fig.savefig(out_path)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--wave-3p8", type=Path, required=True)
    parser.add_argument("--wave-5p5", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()

    configure_style()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows = load_summary(args.summary)
    plot_sweep(rows, args.out_dir / "acdc_halfbridge_sweep.png")
    plot_waveforms(
        args.wave_3p8,
        args.wave_5p5,
        args.out_dir / "acdc_halfbridge_waveforms.png",
    )


if __name__ == "__main__":
    main()
