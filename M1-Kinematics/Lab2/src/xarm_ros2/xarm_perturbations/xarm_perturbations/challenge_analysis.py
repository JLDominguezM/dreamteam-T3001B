#!/usr/bin/env python3
"""
challenge_analysis.py — Post-experiment analysis and plot generation
for the xArm Lite 6 CTC vs PD challenge.

Computes all required metrics and generates all required plots:
  - Joint tracking plots (6 subplots per trial)
  - Task-space component plots (X, Y, Z)
  - 3D path visualization
  - End-effector error norm over time
  - Phase portraits (e_j vs e_dot_j)
  - Controller comparison plots (CTC vs PD overlay)
  - Summary comparison table

Usage:
  python3 challenge_analysis.py --data_dir results/challenge/

Reference: compare_robust_ctc_vs_pid_lite6_IK_fixed_enhanced.py
"""

import argparse
import csv
import json
import pathlib
import sys
from typing import Dict, List, Tuple

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

N_JOINTS = 6

# ═══════════════════════════════════════════════════════════════════════
#  Data Loading
# ═══════════════════════════════════════════════════════════════════════

def load_trial_csv(csv_path: str) -> Dict[str, np.ndarray]:
    """
    Load a trial CSV file into a dictionary of numpy arrays.

    Returns dict with keys:
      time, q (N,6), qd (N,6), q_des (N,6), qd_des (N,6), qdd_des (N,6),
      p (N,3), p_des (N,3), cmd (N,3), saturation (N,6), pert_enabled (N,)
    """
    data = {
        "time": [], "q": [], "qd": [], "q_des": [], "qd_des": [],
        "qdd_des": [], "p": [], "p_des": [], "cmd": [],
        "saturation": [], "pert_enabled": [],
    }

    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            data["time"].append(float(row["time"]))
            data["q"].append([float(row[f"q{j+1}"]) for j in range(6)])
            data["qd"].append([float(row[f"qd{j+1}"]) for j in range(6)])
            data["q_des"].append([float(row[f"q_des{j+1}"]) for j in range(6)])
            data["qd_des"].append([float(row[f"qd_des{j+1}"]) for j in range(6)])
            data["qdd_des"].append([float(row[f"qdd_des{j+1}"]) for j in range(6)])
            data["p"].append([float(row["px"]), float(row["py"]), float(row["pz"])])
            data["p_des"].append([float(row["px_des"]), float(row["py_des"]), float(row["pz_des"])])
            data["cmd"].append([float(row["cmd_x"]), float(row["cmd_y"]), float(row["cmd_z"])])
            data["saturation"].append([int(row[f"sat{j+1}"]) for j in range(6)])
            data["pert_enabled"].append(int(row["pert_enabled"]))

    return {k: np.array(v) for k, v in data.items()}


def load_metadata(json_path: str) -> dict:
    """Load trial metadata from JSON file."""
    with open(json_path, "r") as f:
        return json.load(f)


# ═══════════════════════════════════════════════════════════════════════
#  Metrics Computation
# ═══════════════════════════════════════════════════════════════════════

def compute_metrics(
    trial_data: Dict[str, np.ndarray],
    dwell_windows: List[Tuple[float, float]] = None,
    success_threshold: float = 0.005,
) -> dict:
    """
    Compute all required metrics for one trial.

    Metrics computed:
      - Joint RMSE per joint
      - Joint max absolute error per joint
      - Dwell-window mean error per joint per waypoint
      - End-effector position error norm RMSE
      - End-effector max error
      - Waypoint success rate

    Args:
        trial_data: dict with arrays from load_trial_csv
        dwell_windows: list of (t_start, t_end) for waypoint dwells
        success_threshold: meters, for waypoint success check

    Returns:
        metrics: dict of computed values
    """
    metrics = {}
    t = trial_data["time"]
    q = trial_data["q"]
    q_des = trial_data["q_des"]
    qd = trial_data["qd"]
    qd_des = trial_data["qd_des"]
    p = trial_data["p"]
    p_des = trial_data["p_des"]

    # ── Joint-space metrics ──────────────────────────────────────
    e_joint = q - q_des
    metrics["joint_rmse"] = np.sqrt(np.mean(e_joint**2, axis=0))
    metrics["joint_max_error"] = np.max(np.abs(e_joint), axis=0)
    metrics["joint_rmse_avg"] = float(np.mean(metrics["joint_rmse"]))
    metrics["joint_max_error_avg"] = float(np.mean(metrics["joint_max_error"]))

    # ── Task-space metrics ───────────────────────────────────────
    e_ee = np.linalg.norm(p - p_des, axis=1)
    metrics["ee_rmse"] = float(np.sqrt(np.mean(e_ee**2)))
    metrics["ee_max_error"] = float(np.max(e_ee))
    metrics["ee_error_timeseries"] = e_ee

    # Per-axis EE errors
    for ax, label in enumerate(["x", "y", "z"]):
        e_ax = p[:, ax] - p_des[:, ax]
        metrics[f"ee_rmse_{label}"] = float(np.sqrt(np.mean(e_ax**2)))

    # ── Dwell-window metrics ─────────────────────────────────────
    if dwell_windows is not None:
        dwell_joint_errors = []
        dwell_ee_errors = []
        successes = 0

        for dw_start, dw_end in dwell_windows:
            mask = (t >= dw_start) & (t <= dw_end)
            if np.sum(mask) == 0:
                continue

            # Joint dwell mean error
            dwell_e_joint = np.mean(np.abs(e_joint[mask]), axis=0)
            dwell_joint_errors.append(dwell_e_joint)

            # EE dwell metrics
            dwell_ee = e_ee[mask]
            dwell_ee_errors.append({
                "mean": float(np.mean(dwell_ee)),
                "max": float(np.max(dwell_ee)),
            })

            # Waypoint success: check last 0.5s of dwell
            final_mask = (t >= dw_end - 0.5) & (t <= dw_end)
            if np.sum(final_mask) > 0:
                final_ee = e_ee[final_mask]
                if np.all(final_ee < success_threshold):
                    successes += 1

        n_wps = len(dwell_windows)
        metrics["dwell_joint_mean_errors"] = dwell_joint_errors
        metrics["dwell_ee_errors"] = dwell_ee_errors
        metrics["waypoint_success_count"] = successes
        metrics["waypoint_total"] = n_wps
        metrics["waypoint_success_rate"] = (
            100.0 * successes / n_wps if n_wps > 0 else 0.0
        )
    else:
        metrics["waypoint_success_rate"] = 0.0

    metrics["success_threshold"] = success_threshold

    return metrics


# ═══════════════════════════════════════════════════════════════════════
#  Plotting Functions
# ═══════════════════════════════════════════════════════════════════════

def plot_joint_tracking(
    trial_data: Dict[str, np.ndarray],
    title_prefix: str,
    dwell_windows: list = None,
    out_dir: pathlib.Path = None,
):
    """
    Generate joint position tracking plots: 6 subplots
    showing q_j(t) vs q_des_j(t).
    """
    t = trial_data["time"]
    q = trial_data["q"]
    q_des = trial_data["q_des"]

    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(f"{title_prefix}: Joint Position Tracking", fontsize=14)

    for j in range(N_JOINTS):
        ax = axes[j // 2, j % 2]
        ax.plot(t, np.degrees(q[:, j]), "b-", lw=0.8, label="Measured")
        ax.plot(t, np.degrees(q_des[:, j]), "r--", lw=1.0, label="Desired")

        # Highlight dwell windows
        if dwell_windows:
            for dw_s, dw_e in dwell_windows:
                ax.axvspan(dw_s, dw_e, alpha=0.1, color="green")

        ax.set_ylabel(f"$q_{j+1}$ [deg]")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[2, 0].set_xlabel("Time [s]")
    axes[2, 1].set_xlabel("Time [s]")
    fig.tight_layout()

    if out_dir:
        fig.savefig(out_dir / f"{title_prefix}_joint_tracking.png", dpi=150)
    plt.close(fig)


def plot_taskspace_tracking(
    trial_data: Dict[str, np.ndarray],
    title_prefix: str,
    dwell_windows: list = None,
    out_dir: pathlib.Path = None,
):
    """
    Generate task-space plots:
      - 3 subplots: X, Y, Z components
      - 1 subplot: 3D path visualization
    """
    t = trial_data["time"]
    p = trial_data["p"]
    p_des = trial_data["p_des"]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f"{title_prefix}: Task-Space Tracking", fontsize=14)

    labels = ["X", "Y", "Z"]
    for i in range(3):
        ax = axes[i // 2, i % 2]
        ax.plot(t, p[:, i] * 1000, "b-", lw=0.8, label="Measured")
        ax.plot(t, p_des[:, i] * 1000, "r--", lw=1.0, label="Desired")

        if dwell_windows:
            for dw_s, dw_e in dwell_windows:
                ax.axvspan(dw_s, dw_e, alpha=0.1, color="green")

        ax.set_ylabel(f"{labels[i]} [mm]")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[1, 0].set_xlabel("Time [s]")

    # 3D path
    ax3d = fig.add_subplot(2, 2, 4, projection="3d")
    ax3d.plot(
        p[:, 0] * 1000, p[:, 1] * 1000, p[:, 2] * 1000,
        "b-", lw=0.6, label="Measured"
    )
    ax3d.plot(
        p_des[:, 0] * 1000, p_des[:, 1] * 1000, p_des[:, 2] * 1000,
        "r--", lw=0.8, label="Desired"
    )
    # Mark waypoints
    ax3d.plot(
        [p_des[0, 0] * 1000], [p_des[0, 1] * 1000], [p_des[0, 2] * 1000],
        "go", ms=8, label="Home"
    )
    ax3d.set_xlabel("X [mm]")
    ax3d.set_ylabel("Y [mm]")
    ax3d.set_zlabel("Z [mm]")
    ax3d.legend(fontsize=8)

    fig.tight_layout()
    if out_dir:
        fig.savefig(out_dir / f"{title_prefix}_taskspace.png", dpi=150)
    plt.close(fig)


def plot_ee_error(
    trial_data: Dict[str, np.ndarray],
    metrics: dict,
    title_prefix: str,
    dwell_windows: list = None,
    out_dir: pathlib.Path = None,
):
    """
    Plot end-effector error norm over time with success threshold line.
    """
    t = trial_data["time"]
    e_ee = metrics["ee_error_timeseries"] * 1000  # convert to mm
    thresh = metrics["success_threshold"] * 1000   # mm

    fig, ax = plt.subplots(figsize=(14, 5))
    ax.plot(t, e_ee, "b-", lw=0.8, label="$||e_{EE}||$")
    ax.axhline(y=thresh, color="red", ls="--", lw=1.0,
               label=f"Threshold = {thresh:.1f} mm")

    if dwell_windows:
        for dw_s, dw_e in dwell_windows:
            ax.axvspan(dw_s, dw_e, alpha=0.1, color="green")

    # Highlight perturbation periods
    pert = trial_data["pert_enabled"]
    if np.any(pert > 0):
        ax.fill_between(
            t, 0, e_ee.max(),
            where=pert > 0, alpha=0.05, color="red",
            label="Perturbation active"
        )

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("EE Error [mm]")
    ax.set_title(
        f"{title_prefix}: End-Effector Error — "
        f"RMSE={metrics['ee_rmse']*1000:.2f} mm, "
        f"Max={metrics['ee_max_error']*1000:.2f} mm"
    )
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    if out_dir:
        fig.savefig(out_dir / f"{title_prefix}_ee_error.png", dpi=150)
    plt.close(fig)


def plot_phase_portraits(
    trial_data: Dict[str, np.ndarray],
    title_prefix: str,
    out_dir: pathlib.Path = None,
):
    """
    Generate phase portraits: e_j vs e_dot_j for all 6 joints.
    Annotate equilibrium at origin.
    """
    q = trial_data["q"]
    q_des = trial_data["q_des"]
    qd = trial_data["qd"]
    qd_des = trial_data["qd_des"]

    e = q - q_des
    e_dot = qd - qd_des

    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    fig.suptitle(f"{title_prefix}: Phase Portraits", fontsize=14)

    for j in range(N_JOINTS):
        ax = axes[j // 2, j % 2]
        ax.plot(
            np.degrees(e[:, j]), np.degrees(e_dot[:, j]),
            "b-", lw=0.3, alpha=0.7
        )
        ax.plot(0, 0, "r+", ms=12, mew=2, label="Equilibrium")
        ax.set_xlabel(f"$e_{j+1}$ [deg]")
        ax.set_ylabel(f"$\\dot{{e}}_{j+1}$ [deg/s]")
        ax.set_title(f"Joint {j+1}")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

    fig.tight_layout()
    if out_dir:
        fig.savefig(out_dir / f"{title_prefix}_phase_portraits.png", dpi=150)
    plt.close(fig)


def plot_comparison(
    data_ctc: Dict[str, np.ndarray],
    data_pd: Dict[str, np.ndarray],
    metrics_ctc: dict,
    metrics_pd: dict,
    pert_label: str,
    out_dir: pathlib.Path = None,
):
    """
    Create controller comparison plots: CTC vs PD overlaid.
    """
    t_ctc = data_ctc["time"]
    t_pd = data_pd["time"]

    # ── EE error comparison ──────────────────────────────────────
    fig, axes = plt.subplots(2, 1, figsize=(14, 8))
    fig.suptitle(f"Controller Comparison — {pert_label}", fontsize=14)

    e_ctc = metrics_ctc["ee_error_timeseries"] * 1000
    e_pd = metrics_pd["ee_error_timeseries"] * 1000

    axes[0].plot(t_ctc, e_ctc, "b-", lw=0.8, label="CTC")
    axes[0].plot(t_pd, e_pd, "r-", lw=0.8, label="PD")
    axes[0].set_ylabel("EE Error [mm]")
    axes[0].set_title("End-Effector Error Norm")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # ── Joint RMSE bar chart ─────────────────────────────────────
    x = np.arange(N_JOINTS)
    width = 0.35
    axes[1].bar(
        x - width / 2,
        np.degrees(metrics_ctc["joint_rmse"]),
        width, label="CTC", color="steelblue"
    )
    axes[1].bar(
        x + width / 2,
        np.degrees(metrics_pd["joint_rmse"]),
        width, label="PD", color="salmon"
    )
    axes[1].set_xlabel("Joint")
    axes[1].set_ylabel("RMSE [deg]")
    axes[1].set_title("Joint RMSE Comparison")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels([f"J{j+1}" for j in range(N_JOINTS)])
    axes[1].legend()
    axes[1].grid(True, alpha=0.3, axis="y")

    fig.tight_layout()
    if out_dir:
        tag = "nopert" if "No" in pert_label else "pert"
        fig.savefig(out_dir / f"comparison_{tag}.png", dpi=150)
    plt.close(fig)


def create_comparison_table(
    metrics_list: list,
    labels: list,
    out_dir: pathlib.Path = None,
) -> str:
    """
    Create the required summary comparison table.

    | Metric              | No Pert CTC | No Pert PD | Pert CTC | Pert PD |
    """
    header = (
        f"{'Metric':<25} | "
        + " | ".join(f"{l:>12}" for l in labels)
        + " |"
    )
    sep = "-" * len(header)

    rows = []

    # Joint RMSE (avg)
    vals = [f"{m['joint_rmse_avg']*1000:.3f}" for m in metrics_list]
    rows.append(f"{'Joint RMSE avg (mrad)':<25} | " + " | ".join(f"{v:>12}" for v in vals) + " |")

    # Joint Max Error (avg)
    vals = [f"{m['joint_max_error_avg']*1000:.3f}" for m in metrics_list]
    rows.append(f"{'Joint Max Err avg (mrad)':<25} | " + " | ".join(f"{v:>12}" for v in vals) + " |")

    # EE RMSE (mm)
    vals = [f"{m['ee_rmse']*1000:.3f}" for m in metrics_list]
    rows.append(f"{'EE RMSE (mm)':<25} | " + " | ".join(f"{v:>12}" for v in vals) + " |")

    # EE Max Error (mm)
    vals = [f"{m['ee_max_error']*1000:.3f}" for m in metrics_list]
    rows.append(f"{'EE Max Error (mm)':<25} | " + " | ".join(f"{v:>12}" for v in vals) + " |")

    # Waypoint Success (%)
    vals = [f"{m['waypoint_success_rate']:.1f}%" for m in metrics_list]
    rows.append(f"{'Waypoint Success (%)':<25} | " + " | ".join(f"{v:>12}" for v in vals) + " |")

    table = "\n".join([sep, header, sep] + rows + [sep])

    if out_dir:
        with open(out_dir / "comparison_table.txt", "w") as f:
            f.write(table)

        # Also save as CSV
        with open(out_dir / "metrics_summary.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Metric"] + labels)
            writer.writerow(["Joint RMSE avg (mrad)"] + [f"{m['joint_rmse_avg']*1000:.4f}" for m in metrics_list])
            writer.writerow(["Joint Max Err avg (mrad)"] + [f"{m['joint_max_error_avg']*1000:.4f}" for m in metrics_list])
            writer.writerow(["EE RMSE (mm)"] + [f"{m['ee_rmse']*1000:.4f}" for m in metrics_list])
            writer.writerow(["EE Max Error (mm)"] + [f"{m['ee_max_error']*1000:.4f}" for m in metrics_list])
            writer.writerow(["Waypoint Success (%)"] + [f"{m['waypoint_success_rate']:.1f}" for m in metrics_list])

    return table


# ═══════════════════════════════════════════════════════════════════════
#  Main Analysis Entry Point
# ═══════════════════════════════════════════════════════════════════════

def run_analysis(data_dir: str):
    """
    Run full analysis on all trial data in the given directory.

    Expects subdirectories:
      trial_ctc_nopert_*/
      trial_pd_nopert_*/
      trial_ctc_pert_*/
      trial_pd_pert_*/
    """
    base = pathlib.Path(data_dir)
    if not base.exists():
        print(f"Error: directory '{data_dir}' not found.")
        sys.exit(1)

    # Find trial directories
    trial_dirs = {}
    for d in sorted(base.iterdir()):
        if not d.is_dir():
            continue
        name = d.name.lower()
        if "ctc" in name and "nopert" in name:
            trial_dirs["ctc_nopert"] = d
        elif "pd" in name and "nopert" in name:
            trial_dirs["pd_nopert"] = d
        elif "ctc" in name and "pert" in name and "nopert" not in name:
            trial_dirs["ctc_pert"] = d
        elif "pd" in name and "pert" in name and "nopert" not in name:
            trial_dirs["pd_pert"] = d

    print(f"Found {len(trial_dirs)} trial directories:")
    for k, v in trial_dirs.items():
        print(f"  {k}: {v.name}")

    # Output directory for plots
    plots_dir = base / "plots"
    plots_dir.mkdir(exist_ok=True)

    # Load and analyze each trial
    all_data = {}
    all_metrics = {}

    for trial_key, trial_dir in trial_dirs.items():
        # Find CSV and metadata
        csvs = list(trial_dir.glob("*.csv"))
        jsons = list(trial_dir.glob("*metadata*.json"))

        if not csvs:
            print(f"  Warning: no CSV found in {trial_dir}")
            continue

        csv_path = csvs[0]
        print(f"\nLoading {trial_key}: {csv_path.name}")

        data = load_trial_csv(str(csv_path))
        all_data[trial_key] = data

        # Load dwell windows from metadata
        dwell_windows = None
        if jsons:
            meta = load_metadata(str(jsons[0]))
            dwell_windows = [
                (dw["start"], dw["end"])
                for dw in meta.get("dwell_windows", [])
            ]

        # Compute metrics
        metrics = compute_metrics(data, dwell_windows)
        all_metrics[trial_key] = metrics

        print(f"  Joint RMSE (avg): {metrics['joint_rmse_avg']*1000:.3f} mrad")
        print(f"  EE RMSE:          {metrics['ee_rmse']*1000:.3f} mm")
        print(f"  EE Max Error:     {metrics['ee_max_error']*1000:.3f} mm")
        print(f"  WP Success:       {metrics['waypoint_success_rate']:.1f}%")

        # Generate per-trial plots
        prefix = trial_key
        plot_joint_tracking(data, prefix, dwell_windows, plots_dir)
        plot_taskspace_tracking(data, prefix, dwell_windows, plots_dir)
        plot_ee_error(data, metrics, prefix, dwell_windows, plots_dir)
        plot_phase_portraits(data, prefix, plots_dir)

    # Generate comparison plots
    if "ctc_nopert" in all_data and "pd_nopert" in all_data:
        plot_comparison(
            all_data["ctc_nopert"], all_data["pd_nopert"],
            all_metrics["ctc_nopert"], all_metrics["pd_nopert"],
            "No Perturbations", plots_dir,
        )

    if "ctc_pert" in all_data and "pd_pert" in all_data:
        plot_comparison(
            all_data["ctc_pert"], all_data["pd_pert"],
            all_metrics["ctc_pert"], all_metrics["pd_pert"],
            "With Perturbations", plots_dir,
        )

    # Generate comparison table
    order = ["ctc_nopert", "pd_nopert", "ctc_pert", "pd_pert"]
    labels = ["CTC NoPert", "PD NoPert", "CTC Pert", "PD Pert"]
    available_metrics = []
    available_labels = []
    for key, label in zip(order, labels):
        if key in all_metrics:
            available_metrics.append(all_metrics[key])
            available_labels.append(label)

    if available_metrics:
        table = create_comparison_table(
            available_metrics, available_labels, plots_dir
        )
        print(f"\n{'='*60}")
        print("SUMMARY COMPARISON TABLE")
        print(f"{'='*60}")
        print(table)

    print(f"\nAll plots saved to: {plots_dir}")
    print("Analysis complete.")


def main():
    parser = argparse.ArgumentParser(
        description="Challenge analysis: compute metrics and generate plots"
    )
    parser.add_argument(
        "--data_dir",
        type=str,
        default=str(
            pathlib.Path(__file__).resolve().parent / "results" / "challenge"
        ),
        help="Directory containing trial subdirectories",
    )
    args = parser.parse_args()
    run_analysis(args.data_dir)


if __name__ == "__main__":
    main()
