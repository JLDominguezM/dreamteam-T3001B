#!/usr/bin/env python3
"""
Generate interactive Plotly HTML plots for each PID experiment.

One HTML plot per experiment showing the 5 joints:
- Current position (solid line)
- Target (dashed line)
- Phases marked with background colors

Usage:
    python3 plot_rubric_results.py --input logs_rubrica/ --output plots_rubrica/
"""

import argparse
import json
import sys
from pathlib import Path
import pandas as pd
import plotly.graph_objects as go
from typing import Dict

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
    "move_to_zero": "rgba(255,200,200,0.25)",
    "hold_zero": "rgba(200,220,255,0.25)",
    "return_home": "rgba(200,255,200,0.25)",
    "hold_home": "rgba(255,245,200,0.25)",
}

PHASE_LABELS = {
    "move_to_zero": "Move to 0°",
    "hold_zero": "Hold 0°",
    "return_home": "Return Home",
    "hold_home": "Hold Home",
}


def detect_phases(df: pd.DataFrame) -> Dict[str, tuple]:
    """Detect the 4 phases of the experiment (equal duration)"""
    total_time = df["time"].max()
    phase_duration = total_time / 4.0

    return {
        "move_to_zero": (0.0, phase_duration),
        "hold_zero": (phase_duration, 2 * phase_duration),
        "return_home": (2 * phase_duration, 3 * phase_duration),
        "hold_home": (3 * phase_duration, total_time),
    }


def load_experiment_csv(csv_path: Path) -> pd.DataFrame:
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)
    if "time" not in df.columns:
        raise ValueError(f"CSV missing 'time' column")

    return df


def load_gains_from_config(config_path: Path, combo_id: str) -> dict | None:
    """Load gains_per_joint for a specific combo_id from the config JSON."""
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


def plot_experiment(
    df: pd.DataFrame,
    combo_id: str,
    output_path: Path,
    gains: dict | None = None,
    joint_names=("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"),
):
    """Generate interactive Plotly HTML plot for an experiment."""

    phases = detect_phases(df)
    fig = go.Figure()

    # Phase background rectangles
    for phase_name, (t_start, t_end) in phases.items():
        fig.add_vrect(
            x0=t_start, x1=t_end,
            fillcolor=PHASE_COLORS[phase_name],
            layer="below",
            line_width=0,
            annotation_text=PHASE_LABELS[phase_name],
            annotation_position="top left",
            annotation_font_size=10,
            annotation_font_color="gray",
        )

    # Plot each joint
    for joint_name in joint_names:
        pos_col = joint_name
        target_col = f"target_{joint_name}"

        if pos_col not in df.columns:
            continue

        color = JOINT_COLORS.get(joint_name, "#333333")
        label = JOINT_LABELS.get(joint_name, joint_name)

        # Build hover text with gains info
        gain_text = ""
        if gains and joint_name in gains:
            g = gains[joint_name]
            gain_text = f"<br>Kp={g['kp']}, Ki={g['ki']}, Kd={g['kd']}"

        # Actual position (solid)
        fig.add_trace(go.Scatter(
            x=df["time"],
            y=df[pos_col],
            mode="lines",
            name=f"{label} (actual)",
            line=dict(color=color, width=2),
            hovertemplate=f"{label}<br>t=%{{x:.3f}}s<br>pos=%{{y:.2f}}°{gain_text}<extra></extra>",
        ))

        # Target (dashed)
        if target_col in df.columns:
            fig.add_trace(go.Scatter(
                x=df["time"],
                y=df[target_col],
                mode="lines",
                name=f"{label} (target)",
                line=dict(color=color, width=1.5, dash="dash"),
                opacity=0.5,
                hovertemplate=f"{label} target<br>t=%{{x:.3f}}s<br>target=%{{y:.2f}}°<extra></extra>",
            ))

    # Build subtitle with gains
    subtitle = ""
    if gains:
        family = combo_id.split("_")[0]
        parts = []
        for jn in joint_names:
            if jn in gains:
                g = gains[jn]
                active = []
                if g["kp"] > 0: active.append(f"Kp={g['kp']}")
                if g["ki"] > 0: active.append(f"Ki={g['ki']}")
                if g["kd"] > 0: active.append(f"Kd={g['kd']}")
                parts.append(f"{JOINT_LABELS.get(jn, jn)}: {', '.join(active) if active else 'all=0'}")
        subtitle = "<br>".join(parts)

    fig.update_layout(
        title=dict(
            text=f"Experiment: {combo_id}<br><span style='font-size:11px;color:gray'>{subtitle}</span>",
            font_size=16,
        ),
        xaxis_title="Time (s)",
        yaxis_title="Position (degrees)",
        template="plotly_white",
        hovermode="x unified",
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="center",
            x=0.5,
            font_size=10,
        ),
        margin=dict(l=60, r=30, t=120, b=50),
        height=600,
    )

    fig.update_xaxes(
        range=[df["time"].min(), df["time"].max()],
        gridcolor="rgba(0,0,0,0.1)",
        gridwidth=0.5,
    )
    fig.update_yaxes(
        gridcolor="rgba(0,0,0,0.1)",
        gridwidth=0.5,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(output_path), include_plotlyjs="cdn")
    print(f"Saved: {output_path}")


def build_experiment_figure(
    df: pd.DataFrame,
    combo_id: str,
    gains: dict | None = None,
    joint_names=("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"),
) -> go.Figure:
    """Build a Plotly figure for one experiment (reusable for dashboard)."""

    phases = detect_phases(df)
    fig = go.Figure()

    for phase_name, (t_start, t_end) in phases.items():
        fig.add_vrect(
            x0=t_start, x1=t_end,
            fillcolor=PHASE_COLORS[phase_name],
            layer="below",
            line_width=0,
        )

    for joint_name in joint_names:
        pos_col = joint_name
        target_col = f"target_{joint_name}"
        if pos_col not in df.columns:
            continue

        color = JOINT_COLORS.get(joint_name, "#333333")
        label = JOINT_LABELS.get(joint_name, joint_name)

        gain_text = ""
        if gains and joint_name in gains:
            g = gains[joint_name]
            gain_text = f"<br>Kp={g['kp']}, Ki={g['ki']}, Kd={g['kd']}"

        fig.add_trace(go.Scatter(
            x=df["time"], y=df[pos_col],
            mode="lines",
            name=f"{label}",
            line=dict(color=color, width=2),
            hovertemplate=f"{label}<br>t=%{{x:.3f}}s<br>pos=%{{y:.2f}}°{gain_text}<extra></extra>",
        ))

        if target_col in df.columns:
            fig.add_trace(go.Scatter(
                x=df["time"], y=df[target_col],
                mode="lines",
                name=f"{label} (target)",
                line=dict(color=color, width=1.5, dash="dash"),
                opacity=0.4,
                showlegend=False,
                hoverinfo="skip",
            ))

    # Gains subtitle
    subtitle = combo_id
    if gains:
        parts = []
        for jn in joint_names:
            if jn in gains:
                g = gains[jn]
                active = []
                if g["kp"] > 0: active.append(f"Kp={g['kp']}")
                if g["ki"] > 0: active.append(f"Ki={g['ki']}")
                if g["kd"] > 0: active.append(f"Kd={g['kd']}")
                short = JOINT_LABELS.get(jn, jn).split()[0][:4]
                parts.append(f"{short}: {', '.join(active)}" if active else f"{short}: 0")
        subtitle = f"{combo_id} | " + " | ".join(parts)

    fig.update_layout(
        title=dict(text=subtitle, font_size=12),
        xaxis_title="Time (s)",
        yaxis_title="Position (°)",
        template="plotly_white",
        hovermode="x unified",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="center", x=0.5, font_size=9),
        margin=dict(l=50, r=20, t=60, b=40),
        height=450,
    )

    fig.update_xaxes(range=[df["time"].min(), df["time"].max()], gridcolor="rgba(0,0,0,0.08)")
    fig.update_yaxes(gridcolor="rgba(0,0,0,0.08)")

    return fig


def generate_dashboard(
    csv_files: list[Path],
    output_path: Path,
    config_path: Path,
):
    """Generate a single HTML dashboard with all experiments grouped by family."""

    # Group CSV files by family
    families_order = ["P", "PD", "PI", "PID"]
    family_groups: dict[str, list[tuple[str, Path]]] = {f: [] for f in families_order}

    for csv_path in csv_files:
        combo_id = csv_path.stem.replace("log_", "")
        family = combo_id.split("_")[0]
        if family in family_groups:
            family_groups[family].append((combo_id, csv_path))

    family_descriptions = {
        "P": "Solo Proporcional (Ki=0, Kd=0) — Error permanente, sin damping",
        "PD": "Proporcional + Derivativo (Ki=0) — Con damping, error permanente",
        "PI": "Proporcional + Integral (Kd=0) — Elimina error SS, puede oscilar",
        "PID": "PID Completo — Elimina error SS con damping",
    }

    # Build HTML with embedded Plotly figures
    html_parts = []
    html_parts.append(f"""<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>PID Experiments Dashboard - SO-101</title>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        * {{ margin: 0; padding: 0; box-sizing: border-box; }}
        body {{ font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #f5f6fa; color: #333; }}
        .header {{
            background: linear-gradient(135deg, #2c3e50, #3498db);
            color: white; padding: 30px 40px; text-align: center;
        }}
        .header h1 {{ font-size: 28px; margin-bottom: 8px; }}
        .header p {{ font-size: 14px; opacity: 0.85; }}
        .nav {{
            background: #2c3e50; padding: 12px 40px;
            display: flex; gap: 12px; justify-content: center;
            position: sticky; top: 0; z-index: 100;
        }}
        .nav a {{
            color: white; text-decoration: none; padding: 8px 20px;
            border-radius: 20px; font-size: 14px; font-weight: 600;
            transition: background 0.2s;
        }}
        .nav a:hover {{ background: rgba(255,255,255,0.15); }}
        .nav a.P {{ background: #e74c3c; }}
        .nav a.PD {{ background: #e67e22; }}
        .nav a.PI {{ background: #27ae60; }}
        .nav a.PID {{ background: #2980b9; }}
        .family-section {{ padding: 30px 40px; }}
        .family-header {{
            display: flex; align-items: center; gap: 16px;
            margin-bottom: 20px; padding-bottom: 12px;
            border-bottom: 3px solid #ddd;
        }}
        .family-badge {{
            font-size: 24px; font-weight: 800; color: white;
            padding: 8px 20px; border-radius: 8px; min-width: 80px; text-align: center;
        }}
        .family-badge.P {{ background: #e74c3c; }}
        .family-badge.PD {{ background: #e67e22; }}
        .family-badge.PI {{ background: #27ae60; }}
        .family-badge.PID {{ background: #2980b9; }}
        .family-desc {{ font-size: 15px; color: #666; }}
        .plots-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(600px, 1fr));
            gap: 20px;
        }}
        .plot-card {{
            background: white; border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.08);
            overflow: hidden;
        }}
        .plot-container {{ width: 100%; }}
        .footer {{
            text-align: center; padding: 20px;
            color: #999; font-size: 12px; margin-top: 30px;
        }}
    </style>
</head>
<body>
    <div class="header">
        <h1>PID Experiments Dashboard</h1>
        <p>SO-101 Robot | 20 experiments | 4 families (P, PD, PI, PID) | Random gains [0, 300] | 15s per experiment</p>
    </div>
    <div class="nav">
        <a href="#family-P" class="P">P (5)</a>
        <a href="#family-PD" class="PD">PD (5)</a>
        <a href="#family-PI" class="PI">PI (5)</a>
        <a href="#family-PID" class="PID">PID (5)</a>
    </div>
""")

    plot_id = 0
    for family in families_order:
        experiments = family_groups[family]
        if not experiments:
            continue

        desc = family_descriptions.get(family, "")
        html_parts.append(f"""
    <div class="family-section" id="family-{family}">
        <div class="family-header">
            <span class="family-badge {family}">{family}</span>
            <span class="family-desc">{desc}</span>
        </div>
        <div class="plots-grid">
""")

        for combo_id, csv_path in sorted(experiments):
            try:
                df = load_experiment_csv(csv_path)
                gains = load_gains_from_config(config_path, combo_id)
                fig = build_experiment_figure(df, combo_id, gains=gains)

                div_id = f"plot-{plot_id}"
                plot_json = fig.to_json()

                html_parts.append(f"""
            <div class="plot-card">
                <div class="plot-container" id="{div_id}"></div>
                <script>
                    (function() {{
                        var data = {plot_json};
                        Plotly.newPlot('{div_id}', data.data, data.layout, {{responsive: true}});
                    }})();
                </script>
            </div>
""")
                plot_id += 1
            except Exception as e:
                html_parts.append(f'            <div class="plot-card" style="padding:30px;color:red;">Error: {combo_id} - {e}</div>\n')

        html_parts.append("        </div>\n    </div>\n")

    html_parts.append("""
    <div class="footer">
        PID Experiments Pipeline - T3001B | Generated with Plotly
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
        description="Generate interactive Plotly HTML plots for rubric experiments"
    )
    parser.add_argument(
        '--input', type=str, required=True,
        help='Directory with input CSVs (e.g., logs_rubrica/)'
    )
    parser.add_argument(
        '--output', type=str, required=True,
        help='Directory for output HTML plots'
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
    print(f"Generating interactive HTML plots")
    print(f"{'='*80}")
    print(f"Input:  {input_dir}")
    print(f"Output: {output_dir}")
    print(f"Total:  {len(csv_files)} plots")
    print(f"{'='*80}\n")

    success = 0
    failed = []

    for i, csv_path in enumerate(csv_files, 1):
        combo_id = csv_path.stem.replace("log_", "")
        output_path = output_dir / f"plot_{combo_id}.html"

        print(f"[{i}/{len(csv_files)}] {combo_id}...", end=" ")

        try:
            df = load_experiment_csv(csv_path)
            gains = load_gains_from_config(config_path, combo_id)
            plot_experiment(df, combo_id, output_path, gains=gains)
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
            # Use all CSVs that succeeded (re-gather from output dir)
            all_csvs = sorted(input_dir.glob("log_*.csv"))
            generate_dashboard(all_csvs, dashboard_path, config_path)
        except Exception as e:
            print(f"ERROR generating dashboard: {e}")
            import traceback
            traceback.print_exc()

    if success == len(csv_files):
        print(f"\nAll plots generated")
        print(f"HTML plots at: {output_dir.resolve()}")
        print(f"Dashboard: {(output_dir / 'dashboard.html').resolve()}\n")
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
