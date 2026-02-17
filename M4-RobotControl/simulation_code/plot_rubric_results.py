#!/usr/bin/env python3
"""
Generate PNG plots for each PID experiment and a combined HTML dashboard.

One PNG plot per experiment showing the 5 joints:
- Current position (solid line)
- Target (dashed line)
- Phases marked with background colors
- Title with PID gains per joint

Usage:
    python3 plot_rubric_results.py --input logs_rubrica/ --output plots_rubrica/ --config configs_rubric.json
"""

import argparse
import base64
import json
import sys
from pathlib import Path
from io import BytesIO

import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

JOINT_COLORS = {
    "shoulder_pan": "#e74c3c",
    "shoulder_lift": "#2980b9",
    "elbow_flex": "#27ae60",
    "wrist_flex": "#f39c12",
    "wrist_roll": "#8e44ad",
}

JOINT_LABELS = {
    "shoulder_pan": "Shoulder Pan",
    "shoulder_lift": "Shoulder Lift",
    "elbow_flex": "Elbow Flex",
    "wrist_flex": "Wrist Flex",
    "wrist_roll": "Wrist Roll",
}

PHASE_COLORS = {
    "move_to_zero": (1.0, 0.8, 0.8, 0.25),
    "hold_zero": (0.8, 0.85, 1.0, 0.25),
    "return_home": (0.8, 1.0, 0.8, 0.25),
    "hold_home": (1.0, 0.96, 0.8, 0.25),
}

PHASE_LABELS = {
    "move_to_zero": "Move to 0°",
    "hold_zero": "Hold 0°",
    "return_home": "Return Home",
    "hold_home": "Hold Home",
}

JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def detect_phases(df):
    total_time = df["time"].max()
    phase_duration = total_time / 4.0
    return {
        "move_to_zero": (0.0, phase_duration),
        "hold_zero": (phase_duration, 2 * phase_duration),
        "return_home": (2 * phase_duration, 3 * phase_duration),
        "hold_home": (3 * phase_duration, total_time),
    }


def load_experiment_csv(csv_path):
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    df = pd.read_csv(csv_path)
    if "time" not in df.columns:
        raise ValueError("CSV missing 'time' column")
    return df


def load_gains_from_config(config_path, combo_id):
    if not config_path.exists():
        return None
    try:
        with open(config_path, 'r') as f:
            configs = json.load(f)
        for cfg in configs:
            if cfg["combo_id"] == combo_id:
                return cfg.get("gains_per_joint", None)
    except Exception:
        pass
    return None


def build_gains_title(combo_id, gains):
    """Build a multi-line title string showing PID gains per joint."""
    family = combo_id.split("_")[0]
    title = f"Experiment: {combo_id}  (Family: {family})\n"
    if gains:
        parts = []
        for jn in JOINT_NAMES:
            if jn in gains:
                g = gains[jn]
                label = JOINT_LABELS.get(jn, jn)
                active = []
                if g["kp"] > 0:
                    active.append(f"Kp={g['kp']:.1f}")
                if g["ki"] > 0:
                    active.append(f"Ki={g['ki']:.1f}")
                if g["kd"] > 0:
                    active.append(f"Kd={g['kd']:.1f}")
                parts.append(f"{label}: {', '.join(active) if active else 'all=0'}")
        title += "  |  ".join(parts)
    return title


def plot_experiment_png(df, combo_id, output_path, gains=None):
    """Generate a PNG plot for one experiment using matplotlib."""

    phases = detect_phases(df)
    fig, ax = plt.subplots(figsize=(14, 7))

    # Phase background
    for phase_name, (t_start, t_end) in phases.items():
        color = PHASE_COLORS[phase_name]
        ax.axvspan(t_start, t_end, color=color, zorder=0)
        mid = (t_start + t_end) / 2.0
        ax.text(mid, ax.get_ylim()[0] if ax.get_ylim()[0] != 0 else -5, PHASE_LABELS[phase_name],
                ha='center', va='bottom', fontsize=8, color='gray', alpha=0.7)

    # Convert time to numpy array for matplotlib compatibility
    time_arr = df["time"].values

    # Plot each joint
    for joint_name in JOINT_NAMES:
        pos_col = joint_name
        target_col = f"target_{joint_name}"
        if pos_col not in df.columns:
            continue

        color = JOINT_COLORS.get(joint_name, "#333333")
        label = JOINT_LABELS.get(joint_name, joint_name)

        # Actual position (solid)
        ax.plot(time_arr, df[pos_col].values, color=color, linewidth=2, label=f"{label} (actual)")

        # Target (dashed)
        if target_col in df.columns:
            ax.plot(time_arr, df[target_col].values, color=color, linewidth=1.2,
                    linestyle="--", alpha=0.5, label=f"{label} (target)")

    # Phase labels on top
    for phase_name, (t_start, t_end) in phases.items():
        mid = (t_start + t_end) / 2.0
        ax.text(mid, ax.get_ylim()[1] * 0.98 if ax.get_ylim()[1] != 0 else 95,
                PHASE_LABELS[phase_name],
                ha='center', va='top', fontsize=8, color='gray',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.7))

    # Title with gains
    title = build_gains_title(combo_id, gains)
    ax.set_title(title, fontsize=10, fontweight='bold', pad=15)

    ax.set_xlabel("Time (s)", fontsize=11)
    ax.set_ylabel("Position (degrees)", fontsize=11)
    ax.set_xlim(df["time"].min(), df["time"].max())
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8, ncol=2, framealpha=0.9)

    plt.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(output_path), dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"Saved: {output_path}")


def plot_to_base64(df, combo_id, gains=None):
    """Generate a PNG plot and return as base64 string."""

    phases = detect_phases(df)
    fig, ax = plt.subplots(figsize=(14, 7))

    # Phase background
    for phase_name, (t_start, t_end) in phases.items():
        color = PHASE_COLORS[phase_name]
        ax.axvspan(t_start, t_end, color=color, zorder=0)

    # Convert time to numpy array for matplotlib compatibility
    time_arr = df["time"].values

    # Plot each joint
    for joint_name in JOINT_NAMES:
        pos_col = joint_name
        target_col = f"target_{joint_name}"
        if pos_col not in df.columns:
            continue

        color = JOINT_COLORS.get(joint_name, "#333333")
        label = JOINT_LABELS.get(joint_name, joint_name)

        ax.plot(time_arr, df[pos_col].values, color=color, linewidth=2, label=f"{label} (actual)")
        if target_col in df.columns:
            ax.plot(time_arr, df[target_col].values, color=color, linewidth=1.2,
                    linestyle="--", alpha=0.5, label=f"{label} (target)")

    # Phase labels
    for phase_name, (t_start, t_end) in phases.items():
        mid = (t_start + t_end) / 2.0
        ylims = ax.get_ylim()
        ax.text(mid, ylims[1] - (ylims[1] - ylims[0]) * 0.02,
                PHASE_LABELS[phase_name],
                ha='center', va='top', fontsize=8, color='gray',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.7))

    title = build_gains_title(combo_id, gains)
    ax.set_title(title, fontsize=10, fontweight='bold', pad=15)
    ax.set_xlabel("Time (s)", fontsize=11)
    ax.set_ylabel("Position (degrees)", fontsize=11)
    ax.set_xlim(df["time"].min(), df["time"].max())
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8, ncol=2, framealpha=0.9)

    plt.tight_layout()

    buf = BytesIO()
    fig.savefig(buf, format='png', dpi=150, bbox_inches='tight')
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode('utf-8')


def generate_dashboard(csv_files, output_path, config_path):
    """Generate a single HTML dashboard with all experiments as embedded PNG images."""

    families_order = ["P", "PD", "PI", "PID"]
    family_groups = {f: [] for f in families_order}

    for csv_path in csv_files:
        combo_id = csv_path.stem.replace("log_", "")
        family = combo_id.split("_")[0]
        if family in family_groups:
            family_groups[family].append((combo_id, csv_path))

    family_descriptions = {
        "P": "Proportional only (Ki=0, Kd=0) — Steady-state error, no damping",
        "PD": "Proportional + Derivative (Ki=0) — Damping, steady-state error remains",
        "PI": "Proportional + Integral (Kd=0) — Eliminates SS error, may oscillate",
        "PID": "Full PID — Eliminates SS error with damping",
    }

    family_colors = {
        "P": "#e74c3c",
        "PD": "#e67e22",
        "PI": "#27ae60",
        "PID": "#2980b9",
    }

    html_parts = []
    html_parts.append("""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>PID Experiments Dashboard - SO-101</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #f5f6fa; color: #333; }
        .header {
            background: linear-gradient(135deg, #2c3e50, #3498db);
            color: white; padding: 30px 40px; text-align: center;
        }
        .header h1 { font-size: 28px; margin-bottom: 8px; }
        .header p { font-size: 14px; opacity: 0.85; }
        .nav {
            background: #2c3e50; padding: 12px 40px;
            display: flex; gap: 12px; justify-content: center;
            position: sticky; top: 0; z-index: 100;
        }
        .nav a {
            color: white; text-decoration: none; padding: 8px 20px;
            border-radius: 20px; font-size: 14px; font-weight: 600;
            transition: background 0.2s;
        }
        .nav a:hover { opacity: 0.85; }
        .family-section { padding: 30px 40px; }
        .family-header {
            display: flex; align-items: center; gap: 16px;
            margin-bottom: 20px; padding-bottom: 12px;
            border-bottom: 3px solid #ddd;
        }
        .family-badge {
            font-size: 24px; font-weight: 800; color: white;
            padding: 8px 20px; border-radius: 8px; min-width: 80px; text-align: center;
        }
        .family-desc { font-size: 15px; color: #666; }
        .plots-grid {
            display: grid;
            grid-template-columns: 1fr;
            gap: 20px;
        }
        .plot-card {
            background: white; border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.08);
            overflow: hidden; padding: 10px;
        }
        .plot-card img {
            width: 100%;
            height: auto;
            display: block;
        }
        .gains-table {
            width: 100%; border-collapse: collapse; margin: 8px 0;
            font-size: 12px;
        }
        .gains-table th {
            background: #f0f0f0; padding: 4px 8px; text-align: center;
            border: 1px solid #ddd; font-weight: 600;
        }
        .gains-table td {
            padding: 4px 8px; text-align: center; border: 1px solid #ddd;
        }
        .footer {
            text-align: center; padding: 20px;
            color: #999; font-size: 12px; margin-top: 30px;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>PID Experiments Dashboard</h1>
        <p>SO-101 Robot | 20 experiments | 4 families (P, PD, PI, PID) | Random gains [0, 300] | 15s per experiment</p>
    </div>
    <div class="nav">
""")

    # Navigation links
    for family in families_order:
        color = family_colors[family]
        html_parts.append(f'        <a href="#family-{family}" style="background:{color};">{family} (5)</a>\n')

    html_parts.append("    </div>\n")

    # Family sections
    for family in families_order:
        experiments = family_groups[family]
        if not experiments:
            continue

        desc = family_descriptions.get(family, "")
        color = family_colors[family]

        html_parts.append(f"""
    <div class="family-section" id="family-{family}">
        <div class="family-header">
            <span class="family-badge" style="background:{color};">{family}</span>
            <span class="family-desc">{desc}</span>
        </div>
        <div class="plots-grid">
""")

        for combo_id, csv_path in sorted(experiments):
            try:
                df = load_experiment_csv(csv_path)
                gains = load_gains_from_config(config_path, combo_id)

                # Generate base64 PNG
                img_b64 = plot_to_base64(df, combo_id, gains=gains)

                # Build gains table
                gains_html = ""
                if gains:
                    gains_html = '<table class="gains-table"><tr><th>Joint</th><th>Kp</th><th>Ki</th><th>Kd</th></tr>'
                    for jn in JOINT_NAMES:
                        if jn in gains:
                            g = gains[jn]
                            label = JOINT_LABELS.get(jn, jn)
                            gains_html += f'<tr><td>{label}</td><td>{g["kp"]:.1f}</td><td>{g["ki"]:.1f}</td><td>{g["kd"]:.1f}</td></tr>'
                    gains_html += '</table>'

                html_parts.append(f"""
            <div class="plot-card">
                {gains_html}
                <img src="data:image/png;base64,{img_b64}" alt="{combo_id}">
            </div>
""")
                print(f"  Dashboard: embedded {combo_id}")
            except Exception as e:
                html_parts.append(f'            <div class="plot-card" style="padding:30px;color:red;">Error: {combo_id} - {e}</div>\n')

        html_parts.append("        </div>\n    </div>\n")

    html_parts.append("""
    <div class="footer">
        PID Experiments Pipeline - T3001B | Generated with Matplotlib
    </div>
</body>
</html>
""")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        f.write("".join(html_parts))

    print(f"Dashboard saved: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate PNG plots and HTML dashboard for rubric experiments"
    )
    parser.add_argument(
        '--input', type=str, required=True,
        help='Directory with input CSVs (e.g., logs_rubrica/)'
    )
    parser.add_argument(
        '--output', type=str, required=True,
        help='Directory for output PNG plots and dashboard'
    )
    parser.add_argument(
        '--config', type=str, default='configs_rubric.json',
        help='JSON config file (to display gains in plots)'
    )
    parser.add_argument(
        '--combo-id', type=str,
        help='Generate plot only for a specific combo_id'
    )
    parser.add_argument(
        '--families', type=str,
        help='Filter by families (comma-separated, e.g., P,PD)'
    )

    args = parser.parse_args()

    input_dir = Path(args.input)
    output_dir = Path(args.output)
    config_path = Path(args.config)

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
    print(f"Generating PNG plots + HTML dashboard")
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
            gains = load_gains_from_config(config_path, combo_id)
            plot_experiment_png(df, combo_id, output_path, gains=gains)
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

    # Generate combined dashboard
    if success > 0:
        print(f"\nGenerating combined dashboard...")
        dashboard_path = output_dir / "dashboard.html"
        try:
            all_csvs = sorted(input_dir.glob("log_*.csv"))
            generate_dashboard(all_csvs, dashboard_path, config_path)
        except Exception as e:
            print(f"ERROR generating dashboard: {e}")
            import traceback
            traceback.print_exc()

    if success == len(csv_files):
        print(f"\nAll plots generated")
        print(f"PNG plots at: {output_dir.resolve()}")
        print(f"Dashboard: {(output_dir / 'dashboard.html').resolve()}\n")
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
