#!/usr/bin/env python3
"""
Script that generates PID configurations for experiments with SO-101.
Creates 20 different combinations testing families P, PD, PI, and PID.

Each joint gets a random gain value between 0 and MAX_GAIN (300).
The family determines which gains are active (P: only Kp, PD: Kp+Kd, etc.)
"""

import json
import random
import argparse
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Dict

# Maximum value for any PID gain (per joint)
MAX_GAIN = 300

JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


@dataclass
class ExperimentConfig:
    combo_id: str
    family: str
    description: str
    gains_per_joint: Dict[str, Dict[str, float]]
    expected_behavior: str


def _rand_gain() -> float:
    """Generate a random gain value between 0 and MAX_GAIN."""
    return round(random.uniform(0, MAX_GAIN), 2)


def _random_joints(use_kp: bool, use_ki: bool, use_kd: bool) -> Dict[str, Dict[str, float]]:
    """Generate random gains per joint, respecting which terms are active."""
    gains = {}
    for jn in JOINT_NAMES:
        gains[jn] = {
            "kp": _rand_gain() if use_kp else 0.0,
            "ki": _rand_gain() if use_ki else 0.0,
            "kd": _rand_gain() if use_kd else 0.0,
        }
    return gains


def generate_all_configurations(seed: int | None = None) -> list[ExperimentConfig]:
    if seed is not None:
        random.seed(seed)

    configs = []

    # FAMILIA P (Ki=0, Kd=0) - 5 variantes con Kp random [0, 300] por joint
    for i in range(1, 6):
        gains = _random_joints(use_kp=True, use_ki=False, use_kd=False)
        configs.append(ExperimentConfig(
            combo_id=f"P_{i}",
            family="P",
            description=f"Solo proporcional, random Kp por joint",
            gains_per_joint=gains,
            expected_behavior="Sin Ki: error permanente. Sin Kd: sin damping"
        ))

    # FAMILIA PD (Ki=0) - 5 variantes con Kp y Kd random [0, 300] por joint
    for i in range(1, 6):
        gains = _random_joints(use_kp=True, use_ki=False, use_kd=True)
        configs.append(ExperimentConfig(
            combo_id=f"PD_{i}",
            family="PD",
            description=f"Proporcional-Derivativo, random Kp y Kd por joint",
            gains_per_joint=gains,
            expected_behavior="Mejor damping, error permanente sin Ki"
        ))

    # FAMILIA PI (Kd=0) - 5 variantes con Kp y Ki random [0, 300] por joint
    for i in range(1, 6):
        gains = _random_joints(use_kp=True, use_ki=True, use_kd=False)
        configs.append(ExperimentConfig(
            combo_id=f"PI_{i}",
            family="PI",
            description=f"Proporcional-Integral, random Kp y Ki por joint",
            gains_per_joint=gains,
            expected_behavior="Elimina error SS, sin Kd puede oscilar"
        ))

    # FAMILIA PID completo - 5 variantes con Kp, Ki, Kd random [0, 300] por joint
    for i in range(1, 6):
        gains = _random_joints(use_kp=True, use_ki=True, use_kd=True)
        configs.append(ExperimentConfig(
            combo_id=f"PID_{i}",
            family="PID",
            description=f"PID completo, random Kp Ki Kd por joint",
            gains_per_joint=gains,
            expected_behavior="Los 3 terminos activos, elimina error SS y damping"
        ))

    return configs


def main():
    parser = argparse.ArgumentParser(description="Generate PID configs for experiments")
    parser.add_argument('--output', type=str, default='configs_rubric.json',
                       help='Output JSON file')
    parser.add_argument('--print-table', action='store_true',
                       help='Show summary table')
    parser.add_argument('--seed', type=int, default=None,
                       help='Random seed for reproducibility')

    args = parser.parse_args()
    configs = generate_all_configurations(seed=args.seed)

    output_path = Path(args.output)
    with open(output_path, 'w') as f:
        json.dump([asdict(c) for c in configs], f, indent=2)

    print(f"\n{'='*80}")
    print(f"Configs generated: {len(configs)}")
    print(f"Max gain per joint: {MAX_GAIN}")
    print(f"Saved to: {output_path}")
    print(f"{'='*80}\n")

    if args.print_table:
        print("\nSummary of configurations:\n")
        print(f"{'Combo ID':<12} {'Family':<6} ", end="")
        for jn in JOINT_NAMES:
            print(f"| {jn[:10]:^30s} ", end="")
        print()
        print(f"{'':12s} {'':6s} ", end="")
        for _ in JOINT_NAMES:
            print(f"| {'Kp':>8s} {'Ki':>8s} {'Kd':>8s}  ", end="")
        print()
        print("-" * 170)

        for config in configs:
            print(f"{config.combo_id:<12} {config.family:<6} ", end="")
            for jn in JOINT_NAMES:
                g = config.gains_per_joint[jn]
                print(f"| {g['kp']:>8.1f} {g['ki']:>8.1f} {g['kd']:>8.1f}  ", end="")
            print()

        print(f"\nAll values are random in [0, {MAX_GAIN}]")
        print()


if __name__ == "__main__":
    main()
