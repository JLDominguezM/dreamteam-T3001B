#!/usr/bin/env python3
"""
Calculate quantitative metrics for the rubric experiments.

MMetrics per joint:
- SSE: Steady state error
- Max Overshoot: Maximum overshoot (%)
- Settling Time: Time until ±2° of the target
- Rise Time: Time until 90% of the change
- RMSE: Root mean square error
- Max Velocity: Maximum velocity

Usage:
    python3 analyze_metrics.py --input logs_rubrica/ --output METRICS.md
"""

import argparse
import sys
from pathlib import Path
import pandas as pd
import numpy as np
from typing import Dict, List
import json


JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
SETTLING_TOLERANCE_DEG = 2.0
RISE_PERCENT = 0.9


def detect_phases(df: pd.DataFrame) -> Dict[str, tuple]:
    total_time = df["time"].max()
    phase_duration = total_time / 4.0
    
    return {
        "move_to_zero": (0.0, phase_duration),
        "hold_zero": (phase_duration, 2 * phase_duration),
        "return_home": (2 * phase_duration, 3 * phase_duration),
        "hold_home": (3 * phase_duration, total_time),
    }


def calculate_steady_state_error(df: pd.DataFrame, joint: str, phase: str, phases: Dict) -> float:
    """Average error in the last 0.5s of the hold phase"""
    t_start, t_end = phases[phase]
    hold_window_start = t_end - 0.5
    
    mask = (df["time"] >= hold_window_start) & (df["time"] <= t_end)
    pos = df.loc[mask, joint].values
    target = df.loc[mask, f"target_{joint}"].values
    
    if len(pos) == 0:
        return np.nan
    
    error = np.abs(pos - target)
    return float(np.mean(error))


def calculate_max_overshoot(df: pd.DataFrame, joint: str, phase: str, phases: Dict) -> float:
    """Maximum overshoot (% of total change)"""
    t_start, t_end = phases[phase]
    mask = (df["time"] >= t_start) & (df["time"] <= t_end)
    
    pos = df.loc[mask, joint].values
    target = df.loc[mask, f"target_{joint}"].values
    
    if len(pos) == 0:
        return np.nan
    
    initial_pos = pos[0]
    final_target = target[-1]
    total_change = final_target - initial_pos
    
    if abs(total_change) < 0.1:
        return 0.0
    
    if total_change > 0:
        overshoot = np.max(pos - final_target)
    else:
        overshoot = np.min(pos - final_target)
    
    overshoot_percent = (abs(overshoot) / abs(total_change)) * 100.0
    
    return float(max(0.0, overshoot_percent))


def calculate_settling_time(df: pd.DataFrame, joint: str, phase: str, phases: Dict) -> float:
    """Time until entering and staying within ±tolerance of the target"""
    t_start, t_end = phases[phase]
    mask = (df["time"] >= t_start) & (df["time"] <= t_end)
    
    time = df.loc[mask, "time"].values
    pos = df.loc[mask, joint].values
    target = df.loc[mask, f"target_{joint}"].values
    
    if len(pos) == 0:
        return np.nan
    
    final_target = target[-1]
    error = np.abs(pos - final_target)
    
    outside_band = error > SETTLING_TOLERANCE_DEG
    
    if not np.any(outside_band):
        return 0.0  # Siempre dentro de la banda
    
    last_outside_idx = np.where(outside_band)[0][-1]
    settling_time = time[last_outside_idx] - t_start
    
    return float(settling_time)


def calculate_rise_time(df: pd.DataFrame, joint: str, phase: str, phases: Dict) -> float:
    """
    Calculate the rise time: time to reach 90% of the total change.
    
    Returns:
        Time in seconds (since the start of the phase)
    """
    t_start, t_end = phases[phase]
    mask = (df["time"] >= t_start) & (df["time"] <= t_end)
    
    time = df.loc[mask, "time"].values
    pos = df.loc[mask, joint].values
    target = df.loc[mask, f"target_{joint}"].values
    
    if len(pos) == 0:
        return np.nan
    
    initial_pos = pos[0]
    final_target = target[-1]
    total_change = final_target - initial_pos
    
    if abs(total_change) < 0.1:
        return 0.0
    
    threshold = initial_pos + RISE_PERCENT * total_change
    
    if total_change > 0:
        crossed = pos >= threshold
    else:
        crossed = pos <= threshold
    
    if not np.any(crossed):
        return np.nan  # Nunca alcanzó el 90%
    
    first_cross_idx = np.where(crossed)[0][0]
    rise_time = time[first_cross_idx] - t_start
    
    return float(rise_time)


def calculate_rmse(df: pd.DataFrame, joint: str, phase: str, phases: Dict) -> float:
    """
    Calculate the RMSE (Root Mean Square Error) during a phase.
    
    Returns:
        RMSE in degrees
    """
    t_start, t_end = phases[phase]
    mask = (df["time"] >= t_start) & (df["time"] <= t_end)
    
    pos = df.loc[mask, joint].values
    target = df.loc[mask, f"target_{joint}"].values
    
    if len(pos) == 0:
        return np.nan
    
    error = pos - target
    rmse = float(np.sqrt(np.mean(error ** 2)))
    
    return rmse


def calculate_max_velocity(df: pd.DataFrame, joint: str, phase: str, phases: Dict) -> float:
    """
    Calculate the maximum velocity reached during a phase.
    
    Returns:
        Maximum velocity in deg/s
    """
    t_start, t_end = phases[phase]
    mask = (df["time"] >= t_start) & (df["time"] <= t_end)
    
    vel_col = f"{joint}_velocity"
    if vel_col not in df.columns:
        return np.nan
    
    vel = df.loc[mask, vel_col].values
    
    if len(vel) == 0:
        return np.nan
    
    max_vel = float(np.max(np.abs(vel)))
    
    return max_vel


def analyze_experiment(csv_path: Path) -> Dict:
    """
    Analyze a complete experiment and calculate all metrics.
    
    Returns:
        Dict with metrics organized by joint and phase
    """
    df = pd.read_csv(csv_path)
    phases = detect_phases(df)
    
    metrics = {
        "combo_id": csv_path.stem.replace("log_", ""),
        "joints": {}
    }
    
    for joint in JOINT_NAMES:
        joint_metrics = {
            "move_to_zero": {},
            "hold_zero": {},
            "return_home": {},
            "hold_home": {},
        }

        for move_phase in ["move_to_zero", "return_home"]:
            joint_metrics[move_phase] = {
                "max_overshoot_%": calculate_max_overshoot(df, joint, move_phase, phases),
                "settling_time_s": calculate_settling_time(df, joint, move_phase, phases),
                "rise_time_s": calculate_rise_time(df, joint, move_phase, phases),
                "rmse_deg": calculate_rmse(df, joint, move_phase, phases),
                "max_velocity_deg_s": calculate_max_velocity(df, joint, move_phase, phases),
            }

        for hold_phase in ["hold_zero", "hold_home"]:
            joint_metrics[hold_phase] = {
                "sse_deg": calculate_steady_state_error(df, joint, hold_phase, phases),
                "rmse_deg": calculate_rmse(df, joint, hold_phase, phases),
                "max_velocity_deg_s": calculate_max_velocity(df, joint, hold_phase, phases),
            }
        
        metrics["joints"][joint] = joint_metrics
    
    return metrics



def generate_markdown_table(all_metrics: List[Dict], output_path: Path):
    """Generate a markdown table with summary of metrics"""
    
    with open(output_path, 'w') as f:
        f.write("# Análisis Cuantitativo - Experimentos de Rúbrica\n\n")
        f.write("## Resumen por Experimento\n\n")
        f.write("Métricas promediadas sobre todos los joints.\n\n")

        f.write("| Combo ID | Familia | SSE (°) | Overshoot (%) | Settling (s) | Rise (s) | RMSE (°) |\n")
        f.write("|----------|---------|---------|---------------|--------------|----------|----------|\n")
        
        for metrics in all_metrics:
            combo_id = metrics["combo_id"]
            family = combo_id.split("_")[0]

            sse_list = []
            overshoot_list = []
            settling_list = []
            rise_list = []
            rmse_list = []
            
            for joint_metrics in metrics["joints"].values():

                sse_list.append(joint_metrics["hold_zero"]["sse_deg"])
                sse_list.append(joint_metrics["hold_home"]["sse_deg"])
                

                overshoot_list.append(joint_metrics["move_to_zero"]["max_overshoot_%"])
                overshoot_list.append(joint_metrics["return_home"]["max_overshoot_%"])
                
                settling_list.append(joint_metrics["move_to_zero"]["settling_time_s"])
                settling_list.append(joint_metrics["return_home"]["settling_time_s"])
                
                rise_list.append(joint_metrics["move_to_zero"]["rise_time_s"])
                rise_list.append(joint_metrics["return_home"]["rise_time_s"])
                
                rmse_list.append(joint_metrics["move_to_zero"]["rmse_deg"])
                rmse_list.append(joint_metrics["return_home"]["rmse_deg"])
            

            sse_avg = np.nanmean(sse_list)
            overshoot_avg = np.nanmean(overshoot_list)
            settling_avg = np.nanmean(settling_list)
            rise_avg = np.nanmean(rise_list)
            rmse_avg = np.nanmean(rmse_list)
            
            f.write(f"| {combo_id:<20} | {family:<7} | {sse_avg:7.2f} | {overshoot_avg:13.2f} | "
                   f"{settling_avg:12.3f} | {rise_avg:8.3f} | {rmse_avg:8.2f} |\n")

        f.write("\n---\n\n")
        f.write("## Details per Joint\n\n")
        
        for metrics in all_metrics:
            combo_id = metrics["combo_id"]
            f.write(f"\n### {combo_id}\n\n")
            
            for joint, joint_metrics in metrics["joints"].items():
                f.write(f"#### {joint.replace('_', ' ').title()}\n\n")

                m = joint_metrics["move_to_zero"]
                f.write(f"**Move to 0°:**\n")
                f.write(f"- Overshoot: {m['max_overshoot_%']:.2f}%\n")
                f.write(f"- Settling Time: {m['settling_time_s']:.3f}s\n")
                f.write(f"- Rise Time: {m['rise_time_s']:.3f}s\n")
                f.write(f"- RMSE: {m['rmse_deg']:.2f}°\n")
                f.write(f"- Max Velocity: {m['max_velocity_deg_s']:.1f}°/s\n\n")

                m = joint_metrics["hold_zero"]
                f.write(f"**Hold 0°:**\n")
                f.write(f"- Steady-State Error: {m['sse_deg']:.2f}°\n")
                f.write(f"- RMSE: {m['rmse_deg']:.2f}°\n\n")
        
        f.write("\n---\n\n")
        f.write("## Definitions\n\n")
        f.write("- **SSE (Steady-State Error)**: Average absolute error in the last 0.5s of a hold phase\n")
        f.write("- **Overshoot**: Maximum overshoot as % of total change\n")
        f.write("- **Settling Time**: Time to enter and remain within ±2° of target\n")
        f.write("- **Rise Time**: Time to reach 90% of total change\n")
        f.write("- **RMSE**: Root Mean Square Error during the phase\n")
        f.write("- **Max Velocity**: Maximum velocity reached during the phase\n")

def main():
    parser = argparse.ArgumentParser(
        description="Calculate quantitative metrics for rubric experiments"
    )
    parser.add_argument(
        '--input',
        type=str,
        required=True,
        help='Directory containing the input CSVs (e.g., logs_rubrica/)'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='METRICS.md',
        help='Output markdown file'
    )
    parser.add_argument(
        '--json',
        type=str,
        help='Also save metrics in JSON format'
    )
    parser.add_argument(
        '--combo-id',
        type=str,
        help='Analyze only a specific experiment'
    )
    
    args = parser.parse_args()
    
    input_dir = Path(args.input)
    output_path = Path(args.output)
    
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
        print(f"ERROR: No CSVs in {input_dir}", file=sys.stderr)
        sys.exit(1)
    
    print(f"\n{'='*80}")
    print(f"Calculating metrics")
    print(f"{'='*80}")
    print(f"Input:  {input_dir}")
    print(f"Output: {output_path}")
    print(f"Total:  {len(csv_files)} experiments")
    print(f"{'='*80}\n")
    
    all_metrics = []
    
    for i, csv_path in enumerate(csv_files, 1):
        combo_id = csv_path.stem.replace("log_", "")
        print(f"[{i}/{len(csv_files)}] {combo_id}...", end=" ")
        
        try:
            metrics = analyze_experiment(csv_path)
            all_metrics.append(metrics)
            print("OK")
        except Exception as e:
            print(f"ERROR: {e}")
            import traceback
            traceback.print_exc()

    print(f"\nGenerating table...")
    generate_markdown_table(all_metrics, output_path)
    print(f"Saved: {output_path}")

    if args.json:
        json_path = Path(args.json)
        with open(json_path, 'w') as f:
            json.dump(all_metrics, f, indent=2)
        print(f"JSON: {json_path}")
    
    print(f"\n{'='*80}")
    print(f"Analysis complete")
    print(f"{'='*80}\n")


if __name__ == "__main__":
    main()
