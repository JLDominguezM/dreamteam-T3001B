"""
Standalone evaluator — can be run on any robot_evaluation.csv to regenerate
metrics and plots (e.g. to compare baseline / sine / gaussian experiments).

Usage:
  python3 evaluator.py path/to/robot_evaluation.csv
"""

import sys
import pathlib

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def evaluate_performance(csv_file: str):
    csv_path = pathlib.Path(csv_file)
    df = pd.read_csv(csv_path)

    t = df["time"].to_numpy()
    ex, ey, ez = df["err_x"].to_numpy(), df["err_y"].to_numpy(), df["err_z"].to_numpy()
    e_total = np.sqrt(ex**2 + ey**2 + ez**2)

    rmse_x = float(np.sqrt(np.mean(ex**2)))
    rmse_y = float(np.sqrt(np.mean(ey**2)))
    rmse_z = float(np.sqrt(np.mean(ez**2)))
    rmse_tot = float(np.sqrt(np.mean(e_total**2)))
    max_err_x = float(np.max(np.abs(ex)))
    max_err_y = float(np.max(np.abs(ey)))
    max_err_z = float(np.max(np.abs(ez)))
    max_err_3d = float(np.max(e_total))

    print("=== EVALUATION RESULTS ===")
    print(f"RMSE X:           {rmse_x:.6f} m")
    print(f"RMSE Y:           {rmse_y:.6f} m")
    print(f"RMSE Z:           {rmse_z:.6f} m")
    print(f"RMSE Total:       {rmse_tot:.6f} m")
    print(f"Max Abs Error X:  {max_err_x:.6f} m")
    print(f"Max Abs Error Y:  {max_err_y:.6f} m")
    print(f"Max Abs Error Z:  {max_err_z:.6f} m")
    print(f"Max Abs Error 3D: {max_err_3d:.6f} m")
    print(f"Samples:          {len(df)}")
    print(f"Duration:         {t[-1]:.2f} s")
    print("==========================")

    # Save plots next to the CSV
    out_dir = csv_path.parent
    plt.style.use("bmh")

    fig, axs = plt.subplots(1, 3, figsize=(18, 5))

    # A: XY trajectory
    axs[0].plot(df["des_x"], df["des_y"], "--", color="black", label="Desired")
    axs[0].plot(df["act_x"], df["act_y"], alpha=0.7, color="blue", label="Actual")
    axs[0].set_title("XY Trajectory: Desired vs Actual")
    axs[0].set_xlabel("X [m]"); axs[0].set_ylabel("Y [m]")
    axs[0].legend(); axs[0].axis("equal")

    # B: Error over time
    axs[1].plot(t, ex * 1000, label="err_x", alpha=0.8)
    axs[1].plot(t, ey * 1000, label="err_y", alpha=0.8)
    axs[1].plot(t, ez * 1000, label="err_z", alpha=0.8)
    axs[1].set_title(f"Error — RMSE={rmse_tot*1000:.2f} mm")
    axs[1].set_xlabel("Time [s]"); axs[1].set_ylabel("Error [mm]")
    axs[1].legend()

    # C: Velocity magnitude
    axs[2].plot(t, df["vel_mag"], color="green", label="||vel||")
    axs[2].set_title("Commanded Velocity Magnitude")
    axs[2].set_xlabel("Time [s]"); axs[2].set_ylabel("Velocity [m/s]")
    axs[2].legend()

    plt.tight_layout()
    fig.savefig(out_dir / "evaluation_summary.png", dpi=150)
    plt.close(fig)
    print(f"Plot saved to {out_dir / 'evaluation_summary.png'}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 evaluator.py <robot_evaluation.csv>")
        sys.exit(1)
    evaluate_performance(sys.argv[1])
