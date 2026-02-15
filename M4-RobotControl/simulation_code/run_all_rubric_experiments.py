#!/usr/bin/env python3
"""
Execute the 20 batch experiments.

Usage:
    python3 run_all_rubric_experiments.py --config configs_rubric.json --output logs_rubrica/

Options:
    --skip-existing: Skip experiments that already have CSV
    --families: Filter by families (e.g., P,PD)
    --start-from: Start from a specific combo_id
"""

import argparse
import json
import sys
import time
from pathlib import Path
from datetime import datetime

from run_rubric_experiment import run_experiment, load_config


def load_all_configs(config_file: Path) -> list[dict]:
    if not config_file.exists():
        raise FileNotFoundError(f"Config file not found: {config_file}")
    
    with open(config_file, 'r') as f:
        all_configs = json.load(f)
    
    return all_configs


def filter_configs(
    configs: list[dict],
    families: list[str] | None = None,
    start_from: str | None = None,
) -> list[dict]:
    """Filter configs by family or start point"""
    filtered = configs
    
    if families:
        families_upper = [f.upper() for f in families]
        filtered = [c for c in filtered if c["family"] in families_upper]

    if start_from:
        found = False
        result = []
        for c in filtered:
            if c["combo_id"] == start_from:
                found = True
            if found:
                result.append(c)
        
        if not found:
            raise ValueError(f"start_from combo_id '{start_from}' not found in filtered configs")
        
        filtered = result
    
    return filtered


def check_existing_csv(output_dir: Path, combo_id: str) -> bool:
    """Check if the experiment's CSV already exists"""
    csv_path = output_dir / f"log_{combo_id}.csv"
    return csv_path.exists()


def main():
    parser = argparse.ArgumentParser(
        description="Execute the 20 batch experiments automatically"
    )
    parser.add_argument(
        '--config',
        type=str,
        default='configs_rubric.json',
        help='JSON file with the 20 configurations'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='logs_rubrica/',
        help='Output directory for the CSVs'
    )
    parser.add_argument(
        '--model',
        type=str,
        default='model/scene_urdf.xml',
        help='Path to the MuJoCo XML model'
    )
    parser.add_argument(
        '--skip-existing',
        action='store_true',
        help='Skip experiments that already have CSV'
    )
    parser.add_argument(
        '--families',
        type=str,
        help='Filter by families (e.g., P,PD,PID)'
    )
    parser.add_argument(
        '--start-from',
        type=str,
        help='Start from a specific combo_id (e.g., PD_3_OverDamp)'
    )
    parser.add_argument(
        '--realtime',
        action='store_true',
        help='Run in real-time (slower, for debugging)'
    )
    
    args = parser.parse_args()
    
    config_path = Path(args.config)
    model_path = Path(args.model)
    output_dir = Path(args.output)
    
    if not config_path.exists():
        print(f"ERROR: Config not found: {config_path}", file=sys.stderr)
        sys.exit(1)
    
    if not model_path.exists():
        print(f"ERROR: Model not found: {model_path}", file=sys.stderr)
        sys.exit(1)
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    try:
        all_configs = load_all_configs(config_path)
    except Exception as e:
        print(f"ERROR loading configs: {e}", file=sys.stderr)
        sys.exit(1)
    
    families_list = args.families.split(',') if args.families else None
    
    try:
        configs_to_run = filter_configs(
            all_configs,
            families=families_list,
            start_from=args.start_from,
        )
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    
    # Contar experimentos
    total = len(configs_to_run)
    
    if args.skip_existing:
        existing_count = sum(1 for c in configs_to_run if check_existing_csv(output_dir, c["combo_id"]))
        to_run_count = total - existing_count
    else:
        existing_count = 0
        to_run_count = total
    
    # Initial banner
    print("\n" + "="*80)
    print("-- BATCH EXPERIMENT EXECUTION - ACADEMIC RUBRIC")
    print("="*80)
    print(f"Config: {config_path}")
    print(f"Output: {output_dir}")
    print(f"Model:  {model_path}")
    print(f"Total experiments: {total}")

    if args.skip_existing:
        print(f"Already executed: {existing_count}")
        print(f"To execute:  {to_run_count}")

    if families_list:
        print(f"Filtered families: {', '.join(families_list)}")
    
    if args.start_from:
        print(f"Starting from: {args.start_from}")

    print("="*80 + "\n")
    
    if to_run_count == 0:
        print("All experiments have already been executed (use without --skip-existing to re-run)")
        sys.exit(0)

    start_time = time.time()
    completed = 0
    skipped = 0
    failed = []
    
    for i, config in enumerate(configs_to_run, 1):
        combo_id = config["combo_id"]
        output_csv = output_dir / f"log_{combo_id}.csv"

        if args.skip_existing and check_existing_csv(output_dir, combo_id):
            print(f"[{i}/{total}]  SKIP: {combo_id} (CSV already exists)")
            skipped += 1
            continue
        
        print(f"\n{'─'*80}")
        print(f"[{i}/{total}] EXECUTING: {combo_id}")
        print(f"{'─'*80}")
        
        try:
            exp_start = time.time()
            
            run_experiment(
                model_path=str(model_path),
                config=config,
                output_csv=output_csv,
                realtime=args.realtime,
            )
            
            exp_duration = time.time() - exp_start
            completed += 1
            
            print(f" [{i}/{total}] COMPLETED: {combo_id} ({exp_duration:.1f}s)")
            
        except Exception as e:
            print(f"\nERROR in {combo_id}: {e}", file=sys.stderr)
            failed.append(combo_id)
            import traceback
            traceback.print_exc()
            print()
    
    total_duration = time.time() - start_time
    
    print("\n" + "="*80)
    print("FINAL SUMMARY")
    print("="*80)
    print(f"Total time:    {total_duration/60:.1f} minutes ({total_duration:.1f}s)")
    print(f"Completed:     {completed}/{to_run_count}")

    if skipped > 0:
        print(f"Skipped:        {skipped}")

    if failed:
        print(f"Failed:        {len(failed)}")
        print(f"\n   Failed experiments:")
        for combo_id in failed:
            print(f"   - {combo_id}")
    
    print("="*80)
    
    if completed == to_run_count:
        print("ALL THE EXPERIMENTS COMPLETED SUCCESSFULLY!\n")
        print(f"The CSVs are located at: {output_dir.resolve()}\n")
        sys.exit(0)
    else:
        print(f"\n{completed}/{to_run_count} experiments completed\n")
        sys.exit(1)


if __name__ == "__main__":
    main()
