#!/usr/bin/env python3
"""Generate evaluation plots for EVERY subfolder and save all to results_graphics/."""

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

RESULTS_DIR = Path(__file__).resolve().parent
OUT_DIR = RESULTS_DIR / "results_graphics"
OUT_DIR.mkdir(exist_ok=True)

EXP_TYPES = ["base", "sine", "gauss"]
EXP_COLORS = {"base": "#2196F3", "sine": "#FF9800", "gauss": "#4CAF50"}
EXP_LABELS = {"base": "Base", "sine": "Sine", "gauss": "Gaussian"}


def load_csv(path):
    df = pd.read_csv(path)
    d = {col: df[col].to_numpy().astype(float) for col in df.columns}
    d["time"] = d["time"] - d["time"][0]
    return d


# ── Per-run plots ─────────────────────────────────────────────────────────

def plot_desired_vs_actual(d, title, save_path):
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for i, a in enumerate(["x", "y", "z"]):
        axes[i].plot(d["time"], d[f"des_{a}"] * 1000, "b-", label="Desired", lw=1.5)
        axes[i].plot(d["time"], d[f"act_{a}"] * 1000, "r--", label="Actual", lw=1.2)
        axes[i].set_ylabel(f"{a.upper()} [mm]")
        axes[i].legend(loc="upper right", fontsize=8)
        axes[i].grid(True, alpha=0.3)
    axes[0].set_title(f"Desired vs Actual — {title}")
    axes[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_error(d, title, save_path):
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for i, a in enumerate(["x", "y", "z"]):
        err = d[f"err_{a}"] * 1000
        axes[i].plot(d["time"], err, "r-", lw=1)
        axes[i].axhline(0, color="k", lw=0.5, ls="--")
        axes[i].set_ylabel(f"Error {a.upper()} [mm]")
        axes[i].grid(True, alpha=0.3)
        rmse = np.sqrt(np.mean(err**2))
        axes[i].text(0.98, 0.92, f"RMSE = {rmse:.2f} mm",
                     transform=axes[i].transAxes, ha="right", fontsize=9,
                     bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.8))
    axes[0].set_title(f"Error Over Time — {title}")
    axes[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


def plot_velocity(d, title, save_path):
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(d["time"], d["vel_mag"] * 1000, "g-", lw=1)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Cmd Velocity Mag [mm/s]")
    ax.set_title(f"Commanded Velocity — {title}")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(save_path, dpi=150)
    plt.close(fig)


# ── Comparison plots ──────────────────────────────────────────────────────

def plot_comparison_desired_vs_actual(datasets):
    fig, axes = plt.subplots(3, 3, figsize=(18, 10), sharex="col")
    for col, exp in enumerate(EXP_TYPES):
        if exp not in datasets:
            continue
        d = datasets[exp]
        for row, a in enumerate(["x", "y", "z"]):
            ax = axes[row, col]
            ax.plot(d["time"], d[f"des_{a}"] * 1000, "b-", label="Desired", lw=1.2)
            ax.plot(d["time"], d[f"act_{a}"] * 1000, color=EXP_COLORS[exp],
                    ls="--", label="Actual", lw=1.2)
            ax.grid(True, alpha=0.3)
            if row == 0:
                ax.set_title(EXP_LABELS[exp], fontweight="bold")
            if col == 0:
                ax.set_ylabel(f"{a.upper()} [mm]")
            if row == 2:
                ax.set_xlabel("Time [s]")
            if row == 0 and col == 0:
                ax.legend(fontsize=8)
    fig.suptitle("Desired vs Actual — All Experiments", fontsize=14, y=1.01)
    plt.tight_layout()
    fig.savefig(OUT_DIR / "comparison_desired_vs_actual.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_comparison_error(datasets):
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True)
    for i, a in enumerate(["x", "y", "z"]):
        for exp, d in datasets.items():
            axes[i].plot(d["time"], d[f"err_{a}"] * 1000,
                         color=EXP_COLORS[exp], label=EXP_LABELS[exp], lw=1, alpha=0.85)
        axes[i].axhline(0, color="k", lw=0.5, ls="--")
        axes[i].set_ylabel(f"Error {a.upper()} [mm]")
        axes[i].grid(True, alpha=0.3)
        if i == 0:
            axes[i].legend(fontsize=9)
    axes[0].set_title("Error Comparison — All Experiments")
    axes[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    fig.savefig(OUT_DIR / "comparison_error.png", dpi=150)
    plt.close(fig)


def plot_comparison_velocity(datasets):
    fig, ax = plt.subplots(figsize=(14, 5))
    for exp, d in datasets.items():
        ax.plot(d["time"], d["vel_mag"] * 1000, color=EXP_COLORS[exp],
                label=EXP_LABELS[exp], lw=1, alpha=0.85)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Cmd Velocity Mag [mm/s]")
    ax.set_title("Velocity Comparison — All Experiments")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(OUT_DIR / "comparison_velocity.png", dpi=150)
    plt.close(fig)


def plot_comparison_rmse(datasets):
    x = np.arange(4)
    width = 0.25
    fig, ax = plt.subplots(figsize=(10, 5))
    for i, exp in enumerate(EXP_TYPES):
        if exp not in datasets:
            continue
        d = datasets[exp]
        rx = np.sqrt(np.mean(d["err_x"]**2)) * 1000
        ry = np.sqrt(np.mean(d["err_y"]**2)) * 1000
        rz = np.sqrt(np.mean(d["err_z"]**2)) * 1000
        rt = np.sqrt(rx**2 + ry**2 + rz**2)
        vals = [rx, ry, rz, rt]
        bars = ax.bar(x + i * width, vals, width, label=EXP_LABELS[exp],
                      color=EXP_COLORS[exp])
        for bar, v in zip(bars, vals):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                    f"{v:.1f}", ha="center", fontsize=8)
    ax.set_ylabel("RMSE [mm]")
    ax.set_title("RMSE Comparison — All Experiments")
    ax.set_xticks(x + width)
    ax.set_xticklabels(["X", "Y", "Z", "Total"])
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")
    plt.tight_layout()
    fig.savefig(OUT_DIR / "comparison_rmse.png", dpi=150)
    plt.close(fig)


# ── Main ──────────────────────────────────────────────────────────────────

def main():
    total = 0
    comparison_data = {}

    for exp in EXP_TYPES:
        exp_dir = RESULTS_DIR / exp
        if not exp_dir.exists():
            print(f"  Skipping {exp}/ (not found)")
            continue
        folders = sorted([f for f in exp_dir.iterdir()
                          if f.is_dir() and (f / "robot_evaluation.csv").exists()])
        for run_idx, folder in enumerate(folders, start=1):
            label = f"{EXP_LABELS[exp]} {run_idx}"
            prefix = f"{exp}_{run_idx}"
            print(f"  {exp}/{folder.name} → {label} ...", end=" ")
            d = load_csv(folder / "robot_evaluation.csv")

            plot_desired_vs_actual(d, label, OUT_DIR / f"{prefix}_desired_vs_actual.png")
            plot_error(d, label, OUT_DIR / f"{prefix}_error.png")
            plot_velocity(d, label, OUT_DIR / f"{prefix}_velocity.png")
            total += 3
            print("OK")

        if folders:
            comparison_data[exp] = load_csv(folders[-1] / "robot_evaluation.csv")

    if len(comparison_data) >= 2:
        print("\n  Generating comparison plots...")
        plot_comparison_desired_vs_actual(comparison_data)
        plot_comparison_error(comparison_data)
        plot_comparison_velocity(comparison_data)
        plot_comparison_rmse(comparison_data)
        total += 4

    # Metrics table
    print(f"\n{'='*75}")
    print(f"  {'Metric':<22} {'Base':>14} {'Sine':>14} {'Gaussian':>14}")
    print(f"{'='*75}")
    rows = []
    for exp in EXP_TYPES:
        if exp not in comparison_data:
            rows.append({})
            continue
        d = comparison_data[exp]
        rx = np.sqrt(np.mean(d["err_x"]**2)) * 1000
        ry = np.sqrt(np.mean(d["err_y"]**2)) * 1000
        rz = np.sqrt(np.mean(d["err_z"]**2)) * 1000
        mx = np.max(np.abs(d["err_x"])) * 1000
        my = np.max(np.abs(d["err_y"])) * 1000
        mz = np.max(np.abs(d["err_z"])) * 1000
        rows.append({"rx": rx, "ry": ry, "rz": rz,
                     "rt": np.sqrt(rx**2+ry**2+rz**2),
                     "mx": mx, "my": my, "mz": mz})
    for label, key in [("RMSE X [mm]","rx"), ("RMSE Y [mm]","ry"),
                        ("RMSE Z [mm]","rz"), ("RMSE Total [mm]","rt"),
                        ("Max|err| X [mm]","mx"), ("Max|err| Y [mm]","my"),
                        ("Max|err| Z [mm]","mz")]:
        vals = [f"{r.get(key,0):>14.2f}" if r else f"{'N/A':>14}" for r in rows]
        print(f"  {label:<22} {''.join(vals)}")
    print(f"{'='*75}")
    print(f"\n  {total} plots saved to {OUT_DIR}")


if __name__ == "__main__":
    main()
