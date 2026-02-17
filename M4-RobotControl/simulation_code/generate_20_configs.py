#!/usr/bin/env python3
"""
Script that generates PID configurations for experiments with SO-101.
Creates 20 different combinations testing families P, PD, PI, and PID.

Each joint has different scale factors based on its physical characteristics:
- shoulder_lift needs higher Ki (supports more weight)
- wrist_roll can use lower gains (less inertia)
"""

import json
import argparse
from pathlib import Path
from dataclasses import dataclass, asdict
from typing import Dict


@dataclass
class JointScaleFactors:
    kp_scale: float
    ki_scale: float
    kd_scale: float
    description: str


@dataclass
class ExperimentConfig:
    combo_id: str
    family: str
    description: str
    kp_base: float
    ki_base: float
    kd_base: float
    gains_per_joint: Dict[str, Dict[str, float]]
    expected_behavior: str

JOINT_SCALE_FACTORS = {
    "shoulder_pan": JointScaleFactors(
        kp_scale=1.5,
        ki_scale=1.5,
        kd_scale=1.2,
        description="High rotational inertia"
    ),
    "shoulder_lift": JointScaleFactors(
        kp_scale=2.0,
        ki_scale=2.5,
        kd_scale=1.5,
        description="Supports arm weight, needs high Ki"
    ),
    "elbow_flex": JointScaleFactors(
        kp_scale=1.5,
        ki_scale=2.0,
        kd_scale=1.3,
        description="Medium inertia, variable gravity"
    ),
    "wrist_flex": JointScaleFactors(
        kp_scale=1.0,
        ki_scale=1.0,
        kd_scale=1.0,
        description="Base reference"
    ),
    "wrist_roll": JointScaleFactors(
        kp_scale=0.6,
        ki_scale=0.5,
        kd_scale=0.8,
        description="Minimal inertia, fast rotation, little gravity effect"
    ),
}


def apply_scale_factors(kp_base: float, ki_base: float, kd_base: float) -> Dict[str, Dict[str, float]]:
    """
    Apply scale factors to the base gains for each joint.
    
    Returns:
        Dict with structure: {joint_name: {kp: X, ki: Y, kd: Z}}
    """
    gains = {}
    for joint_name, factors in JOINT_SCALE_FACTORS.items():
        gains[joint_name] = {
            "kp": round(kp_base * factors.kp_scale, 2),
            "ki": round(ki_base * factors.ki_scale, 2),
            "kd": round(kd_base * factors.kd_scale, 2),
        }
    return gains


def generate_all_configurations() -> list[ExperimentConfig]:
    configs = []
    
    p_configs = [
        (300, "MuyBajo", "Kp muy bajo, respuesta lenta"),
        (600, "Bajo", "Kp bajo, suave pero lento"),
        (1000, "Medio", "Kp medio, balance"),
        (1500, "Alto", "Kp alto, rápido con overshoot"),
        (2500, "MuyAlto", "Kp muy alto, riesgo de oscilación"),
    ]
    
    for i, (kp_base, level, desc) in enumerate(p_configs, 1):
        configs.append(ExperimentConfig(
            combo_id=f"P_{i}_{level}",
            family="P",
            description=desc,
            kp_base=kp_base,
            ki_base=0.0,
            kd_base=0.0,
            gains_per_joint=apply_scale_factors(kp_base, 0.0, 0.0),
            expected_behavior=f"Sin Ki: error permanente. Velocidad {'lenta' if kp_base < 800 else 'rápida' if kp_base > 1200 else 'media'}"
        ))
    
    # FAMILIA PD (Ki=0) - 5 variants
    pd_configs = [
        (1000, 20, "PocoDamp", "PD con poco damping, Kd bajo"),
        (1000, 50, "Critico", "PD amortiguamiento crítico óptimo"),
        (1000, 100, "OverDamp", "Sobreamortiguado, lento"),
        (1800, 90, "Rigido", "Kp y Kd altos, muy rígido"),
        (1200, 180, "ExcesivoKd", "Kd muy alto, amplifica ruido"),
    ]
    
    for i, (kp_base, kd_base, level, desc) in enumerate(pd_configs, 1):
        configs.append(ExperimentConfig(
            combo_id=f"PD_{i}_{level}",
            family="PD",
            description=desc,
            kp_base=kp_base,
            ki_base=0.0,
            kd_base=kd_base,
            gains_per_joint=apply_scale_factors(kp_base, 0.0, kd_base),
            expected_behavior=f"Mejor damping, error permanente sin Ki, ratio Kd/Kp={kd_base/kp_base:.2f}"
        ))
    
    # Control PI (sin D) - 5 variantes
    pi_configs = [
        (800, 40, "KiBajo", "Ki bajo, elimina error lentamente"),
        (800, 80, "KiMedio", "PI balanceado"),
        (800, 160, "KiAlto", "Ki alto, puede saturar"),
        (500, 150, "KpBajo_KiAlto", "Kp bajo, Ki alto, lento pero preciso"),
        (1200, 240, "Agresivo", "Ambos altos, rápido pero inestable"),
    ]
    
    for i, (kp_base, ki_base, level, desc) in enumerate(pi_configs, 1):
        configs.append(ExperimentConfig(
            combo_id=f"PI_{i}_{level}",
            family="PI",
            description=desc,
            kp_base=kp_base,
            ki_base=ki_base,
            kd_base=0.0,
            gains_per_joint=apply_scale_factors(kp_base, ki_base, 0.0),
            expected_behavior=f"Elimina error SS, sin Kd puede oscilar, Ki/Kp={ki_base/kp_base:.2f}"
        ))
    
    # Control PID completo - 5 variantes
    pid_configs = [
        (800, 80, 40, "Conservador", "Ganancias bajas, seguro"),
        (1200, 120, 60, "Balanceado", "Uso general"),
        (1800, 180, 90, "Agresivo", "Respuesta rápida"),
        (2000, 250, 50, "KiDominante", "Ki alto, riesgo de windup"),
        (1000, 100, 150, "KdDominante", "Kd alto, muy amortiguado"),
    ]
    
    for i, (kp_base, ki_base, kd_base, level, desc) in enumerate(pid_configs, 1):
        configs.append(ExperimentConfig(
            combo_id=f"PID_{i}_{level}",
            family="PID",
            description=desc,
            kp_base=kp_base,
            ki_base=ki_base,
            kd_base=kd_base,
            gains_per_joint=apply_scale_factors(kp_base, ki_base, kd_base),
            expected_behavior=f"Los 3 términos activos, elimina error SS y damping"
        ))
    
    return configs

def main():
    parser = argparse.ArgumentParser(description="Generate PID configs for experiments")
    parser.add_argument('--output', type=str, default='configs_rubric.json',
                       help='Output JSON file')
    parser.add_argument('--print-table', action='store_true',
                       help='Show summary table')

    args = parser.parse_args()
    configs = generate_all_configurations()
    
    output_path = Path(args.output)
    with open(output_path, 'w') as f:
        json.dump([asdict(c) for c in configs], f, indent=2)
    
    print(f"\n{'='*80}")
    print(f"Configs generated: {len(configs)}")
    print(f"Saved to: {output_path}")
    print(f"{'='*80}\n")
    
    if args.print_table:
        print("\nSummary of configurations:\n")
        print(f"{'Combo ID':<20} {'Family':<8} {'Kp_base':<10} {'Ki_base':<10} {'Kd_base':<10}")
        print("-" * 70)
        
        for config in configs:
            print(f"{config.combo_id:<20} {config.family:<8} {config.kp_base:<10.0f} "
                  f"{config.ki_base:<10.0f} {config.kd_base:<10.0f}")

        print("\nScaling factors per joint:")
        for joint_name, factors in JOINT_SCALE_FACTORS.items():
            print(f"  {joint_name}: Kp×{factors.kp_scale}, Ki×{factors.ki_scale}, Kd×{factors.kd_scale}")
        print()


if __name__ == "__main__":
    main()
