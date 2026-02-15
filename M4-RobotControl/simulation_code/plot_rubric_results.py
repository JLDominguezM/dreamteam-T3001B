#!/usr/bin/env python3
"""
Generate position vs time plots for each experiment.

One plot per experiment showing the 5 joints:
- Current position (solid line)
- Target (dashed line)
- Phases marked with background colors

Usage:
    python3 plot_rubric_results.py --input logs_rubrica/ --output plots_rubrica/
"""

import argparse
import sys
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from typing import Dict
import numpy as np

JOINT_COLORS = {
    "shoulder_pan": "#FF6B6B",
    "shoulder_lift": "#4ECDC4",
    "elbow_flex": "#95E1D3",
    "wrist_flex": "#F38181",
    "wrist_roll": "#AA96DA",
}

JOINT_LABELS = {
    "shoulder_pan": "Shoulder Pan",
    "shoulder_lift": "Shoulder Lift",
    "elbow_flex": "Elbow Flex",
    "wrist_flex": "Wrist Flex",
    "wrist_roll": "Wrist Roll",
}

PHASE_COLORS = {
    "move_to_zero": "#FFE5E5",
    "hold_zero": "#E5F3FF",
    "return_home": "#E5FFE5",
    "hold_home": "#FFF9E5",
}


def detect_phases(df: pd.DataFrame) -> Dict[str, tuple]:
    """Detect the 4 phases of the experiment (2s each, total 8s)"""
    total_time = df["time"].max()
    phase_duration = total_time / 4.0
    
    phases = {
        "move_to_zero": (0.0, phase_duration),
        "hold_zero": (phase_duration, 2 * phase_duration),
        "return_home": (2 * phase_duration, 3 * phase_duration),
        "hold_home": (3 * phase_duration, total_time),
    }
    
    return phases


def load_experiment_csv(csv_path: Path) -> pd.DataFrame:
    """Load the experiment's CSV"""
    
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    
    df = pd.read_csv(csv_path)

    required_cols = ["time"]
    if not all(col in df.columns for col in required_cols):
        raise ValueError(f"CSV missing required columns: {required_cols}")
    
    return df

def plot_experiment(
    df: pd.DataFrame,
    combo_id: str,
    output_path: Path,
    joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
):
    """
    Generate position vs time plot for an experiment.
    
    Args:
        df: DataFrame with the CSV data
        combo_id: ID of the experiment (e.g., P_1_MuyBajo)
        output_path: Output path for the PNG
        joint_names: Lista de joints a graficar
    """

    phases = detect_phases(df)
    
    # Crear figura
    fig, ax = plt.subplots(figsize=(14, 8))
    
    # Fondo de colores por fase
    phase_labels = {
        "move_to_zero": "Move to 0°",
        "hold_zero": "Hold 0°",
        "return_home": "Return Home",
        "hold_home": "Hold Home",
    }
    
    for phase_name, (t_start, t_end) in phases.items():
        ax.axvspan(
            t_start, t_end,
            alpha=0.15,
            color=PHASE_COLORS[phase_name],
            label=phase_labels[phase_name]
        )
    
    # Graficar cada joint
    for joint_name in joint_names:
        pos_col = joint_name
        target_col = f"target_{joint_name}"
        
        if pos_col not in df.columns:
            print(f"Warning: {pos_col} not found in CSV, skipping")
            continue
        
        color = JOINT_COLORS.get(joint_name, "#333333")
        label = JOINT_LABELS.get(joint_name, joint_name)
        
        ax.plot(
            df["time"].values, df[pos_col].values,
            color=color,
            linewidth=2.0,
            label=label,
            alpha=0.9,
        )

        if target_col in df.columns:
            ax.plot(
                df["time"].values, df[target_col].values,
                color=color,
                linewidth=1.5,
                linestyle="--",
                alpha=0.5,
            )

    ax.set_xlabel("Time (s)", fontsize=12, fontweight="bold")
    ax.set_ylabel("Position (degrees)", fontsize=12, fontweight="bold")
    ax.set_title(
        f"Experiment: {combo_id}\nJoint Positions vs Time",
        fontsize=14,
        fontweight="bold",
        pad=20
    )
    
    ax.grid(True, alpha=0.3, linestyle=":", linewidth=0.8)
    ax.set_xlim(df["time"].min(), df["time"].max())
    

    handles, labels = ax.get_legend_handles_labels()

    phase_handles = handles[-4:]
    phase_labels = labels[-4:]
    joint_handles = handles[:-4]
    joint_labels = labels[:-4]

    phase_legend = ax.legend(
        phase_handles, phase_labels,
        loc="upper left",
        fontsize=9,
        framealpha=0.9,
        title="Phases",
        title_fontsize=10
    )
    
    ax.legend(
        joint_handles, joint_labels,
        loc="upper right",
        fontsize=9,
        framealpha=0.9,
        title="Joints",
        title_fontsize=10
    )
    
    ax.add_artist(phase_legend)
    
    plt.tight_layout()
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)

    print(f"Saved graph: {output_path}")

def main():
    parser = argparse.ArgumentParser(
        description="Generate position vs time plots for rubric experiments"
    )
    parser.add_argument(
        '--input',
        type=str,
        required=True,
        help='Directory with input CSVs (e.g., logs_rubrica/)'
    )
    parser.add_argument(
        '--output',
        type=str,
        required=True,
        help='Directory for output PNG plots'
    )
    parser.add_argument(
        '--combo-id',
        type=str,
        help='Generate plot only for a specific combo_id'
    )
    parser.add_argument(
        '--families',
        type=str,
        help='Filter by families (comma-separated, e.g., P,PD)'
    )
    
    args = parser.parse_args()
    
    input_dir = Path(args.input)
    output_dir = Path(args.output)
    
    if not input_dir.exists():
        print(f"ERROR: Input directory not found: {input_dir}", file=sys.stderr)
        sys.exit(1)
    
    if args.combo_id:
        csv_files = [input_dir / f"log_{args.combo_id}.csv"]
        if not csv_files[0].exists():
            print(f"ERROR: CSV not found: {csv_files[0]}", file=sys.stderr)
            sys.exit(1)
    else:
        csv_files = sorted(input_dir.glob("log_*.csv"))
    
    if not csv_files:
        print(f"ERROR: No CSV files in {input_dir}", file=sys.stderr)
        sys.exit(1)

    if args.families:
        families = [f.strip().upper() for f in args.families.split(',')]
        csv_files = [
            f for f in csv_files
            if any(f.stem.startswith(f"log_{fam}_") for fam in families)
        ]
    
    print(f"\n{'='*80}")
    print(f"Generating plots")
    print(f"{'='*80}")
    print(f"Input:  {input_dir}")
    print(f"Output: {output_dir}")
    print(f"Total:  {len(csv_files)} plots")
    print(f"{'='*80}\n")
    
    success = 0
    failed = []
    
    for i, csv_path in enumerate(csv_files, 1):
        combo_id = csv_path.stem.replace("log_", "")
        output_path = output_dir / f"plot_{combo_id}.png"
        
        print(f"[{i}/{len(csv_files)}] {combo_id}...", end=" ")
        
        try:
            df = load_experiment_csv(csv_path)
            plot_experiment(df, combo_id, output_path)
            success += 1
        except Exception as e:
            print(f"ERROR: {e}")
            failed.append(combo_id)
            import traceback
            traceback.print_exc()

    print(f"\n{'='*80}")
    print(f"Summary")
    print(f"{'='*80}")
    print(f"OK: {success}/{len(csv_files)}")
    
    if failed:
        print(f"Failed: {len(failed)}")
        for combo_id in failed:
            print(f"  - {combo_id}")
    
    print(f"{'='*80}")
    
    if success == len(csv_files):
        print(f"\nAll plots generated\n")
        print(f"Plots are located at: {output_dir.resolve()}\n")
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
