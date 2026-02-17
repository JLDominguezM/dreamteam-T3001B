#!/usr/bin/env python3
"""
Runs a single experiment and saves CSV (without GUI).

Usage:
    python3 run_rubric_experiment.py --config configs_rubric.json --combo-id P_1_MuyBajo

Phases (2s each):
    1. Home → 0°
    2. Hold 0°
    3. 0° → Home
    4. Hold Home
"""

import argparse
import json
import sys
from pathlib import Path
import numpy as np
import mujoco

from so101_control import JointPID, PIDGains, PerturbationModel
from so101_mujoco_pid_utils import (
    DEFAULT_JOINTS,
    move_to_pose_pid,
    hold_position_pid,
    build_default_perturbations,
)
from CSVDataLogger import CSVDataLogger


# Poses for experiment
HOME_POSE_DEG = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -90.0,
    "elbow_flex": 90.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
}

ZERO_POSE_DEG = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
}


def build_custom_pid(gains_per_joint: dict, joint_names=DEFAULT_JOINTS) -> JointPID:
    """
    Builds a JointPID object from JSON gains.
    
    Args:
        gains_per_joint: Dict with structure {joint_name: {kp: X, ki: Y, kd: Z}}
        
    Returns:
        JointPID configured with specified gains
    """
    pid_gains = {}
    
    for joint_name in joint_names:
        if joint_name not in gains_per_joint:
            raise ValueError(f"Missing gains for joint: {joint_name}")
        
        g = gains_per_joint[joint_name]
        
        # i_limit and tau_limit maintained from default values
        # (you can adjust them per joint if needed)
        i_limits = {
            "shoulder_pan": 2.0,
            "shoulder_lift": 2.0,
            "elbow_flex": 2.0,
            "wrist_flex": 2.0,
            "wrist_roll": 2.0,
        }
        
        tau_limits = {
            "shoulder_pan": 8.0,
            "shoulder_lift": 18.0,
            "elbow_flex": 15.0,
            "wrist_flex": 6.0,
            "wrist_roll": 3.0,
        }
        
        pid_gains[joint_name] = PIDGains(
            kp=g["kp"],
            ki=g["ki"],
            kd=g["kd"],
            i_limit=i_limits.get(joint_name, 2.0),
            tau_limit=tau_limits.get(joint_name, 10.0),
        )
    
    return JointPID(joint_names, pid_gains)


def load_config(config_file: Path, combo_id: str) -> dict:
    """Loads specific combo_id configuration from JSON"""
    
    if not config_file.exists():
        raise FileNotFoundError(f"Config file not found: {config_file}")
    
    with open(config_file, 'r') as f:
        all_configs = json.load(f)
    
    # Search for combo_id
    for config in all_configs:
        if config["combo_id"] == combo_id:
            return config
    
    raise ValueError(f"combo_id '{combo_id}' not found in {config_file}")


def run_experiment(
    model_path: str,
    config: dict,
    output_csv: Path,
    joint_names=DEFAULT_JOINTS,
    realtime: bool = False,
):
    """
    Runs complete experiment (4 phases) and saves CSV.
    
    Args:
        model_path: Path to MuJoCo XML model
        config: Dict with experiment configuration
        output_csv: Output CSV path
        joint_names: List of active joints
        realtime: If True, runs in realtime (slower)
    """
    
    print(f"\n{'='*80}")
    print(f"EXPERIMENT: {config['combo_id']}")
    print(f"{'='*80}")
    print(f"Family: {config['family']}")
    print(f"Description: {config['description']}")
    print(f"Gains per joint:")
    for jn, g in config['gains_per_joint'].items():
        print(f"  {jn}: Kp={g['kp']}, Ki={g['ki']}, Kd={g['kd']}")
    print(f"{'='*80}\n")
    
    # Load model
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)
    
    # Build custom PID
    pid = build_custom_pid(config["gains_per_joint"], joint_names)
    
    # Perturbations (keep consistent across experiments)
    perturb = build_default_perturbations(joint_names)
    
    # Reset to home pose
    for jn in joint_names:
        d.qpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)] = np.deg2rad(HOME_POSE_DEG[jn])
    
    mujoco.mj_forward(m, d)
    
    # Create CSV logger
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    
    # 4 phases of equal duration -> 15s total
    phase_dur = 3.75

    # Start with target = ZERO_POSE_DEG (first phase)
    with CSVDataLogger(str(output_csv), joint_names, ZERO_POSE_DEG) as logger:

        # Phase 1: Move Home → 0°
        print(f"Phase 1/4: Home → 0° ({phase_dur}s)")
        move_to_pose_pid(
            m, d, viewer=None,
            target_pose_deg=ZERO_POSE_DEG,
            duration=phase_dur,
            realtime=realtime,
            joint_names=joint_names,
            pid=pid,
            perturb=perturb,
            plotter=logger,
        )

        # Phase 2: Hold 0°
        print(f"Phase 2/4: Hold 0° ({phase_dur}s)")
        hold_position_pid(
            m, d, viewer=None,
            hold_pose_deg=ZERO_POSE_DEG,
            duration=phase_dur,
            realtime=realtime,
            joint_names=joint_names,
            pid=pid,
            perturb=perturb,
            plotter=logger,
        )

        # Phase 3: Move 0° → Home
        print(f"Phase 3/4: 0° → Home ({phase_dur}s)")
        logger.update_target(HOME_POSE_DEG)
        move_to_pose_pid(
            m, d, viewer=None,
            target_pose_deg=HOME_POSE_DEG,
            duration=phase_dur,
            realtime=realtime,
            joint_names=joint_names,
            pid=pid,
            perturb=perturb,
            plotter=logger,
        )

        # Phase 4: Hold Home
        print(f"Phase 4/4: Hold Home ({phase_dur}s)")
        hold_position_pid(
            m, d, viewer=None,
            hold_pose_deg=HOME_POSE_DEG,
            duration=phase_dur,
            realtime=realtime,
            joint_names=joint_names,
            pid=pid,
            perturb=perturb,
            plotter=logger,
        )
    
    print(f"\nCSV saved: {output_csv}")
    print(f"{'='*80}\n")


def main():
    parser = argparse.ArgumentParser(
        description="Run rubric experiment with CSV logging"
    )
    parser.add_argument(
        '--config',
        type=str,
        required=True,
        help='JSON file with 20 configurations'
    )
    parser.add_argument(
        '--combo-id',
        type=str,
        required=True,
        help='Combo ID to execute (e.g.: P_1_MuyBajo, PID_3_Agresivo)'
    )
    parser.add_argument(
        '--output',
        type=str,
        required=True,
        help='Output directory for CSV (will create: output/log_<combo_id>.csv)'
    )
    parser.add_argument(
        '--model',
        type=str,
        default='model/scene_urdf.xml',
        help='Path to MuJoCo XML model'
    )
    parser.add_argument(
        '--realtime',
        action='store_true',
        help='Run in realtime (slower, useful for debugging)'
    )
    
    args = parser.parse_args()
    
    # Validate files
    config_path = Path(args.config)
    model_path = Path(args.model)
    output_dir = Path(args.output)
    
    if not config_path.exists():
        print(f"ERROR: Config file not found: {config_path}", file=sys.stderr)
        sys.exit(1)
    
    if not model_path.exists():
        print(f"ERROR: Model file not found: {model_path}", file=sys.stderr)
        sys.exit(1)
    
    # Load configuration
    try:
        config = load_config(config_path, args.combo_id)
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    
    # Output CSV name
    output_csv = output_dir / f"log_{args.combo_id}.csv"
    
    # Run experiment
    try:
        run_experiment(
            model_path=str(model_path),
            config=config,
            output_csv=output_csv,
            realtime=args.realtime,
        )
    except Exception as e:
        print(f"\nERROR during experiment: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
