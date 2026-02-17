# run_all_configs.py
"""
Automated PID config tester.
Runs each configuration from configs_rubric.json through the MuJoCo simulation
(headless, no viewer) and generates a Plotly graph for each test.
"""
import json
import os
import math

import numpy as np
import mujoco
import plotly.graph_objects as go

from so101_control import JointPID, PIDGains
from so101_mujoco_pid_utils import (
    move_to_pose_pid,
    hold_position_pid,
    build_default_perturbations,
    DEFAULT_JOINTS,
)
from so101_mujoco_utils2 import set_initial_pose, get_positions_dict

# --------------- paths ---------------
CONFIGS_PATH = "configs_rubric.json"
MODEL_PATH   = "model/scene_urdf.xml"
OUTPUT_DIR   = "rubric_results"

# --------------- poses (same as run_mujoco_simulation.py) ---------------
starting_position = {
    "shoulder_pan":  -4.4003158666,
    "shoulder_lift": -92.2462050161,
    "elbow_flex":     89.9543738355,
    "wrist_flex":     55.1185398916,
    "wrist_roll":      0.0,
    "gripper":         0.0,
}

desired_zero = {
    "shoulder_pan":  0.0,
    "shoulder_lift": 0.0,
    "elbow_flex":    0.0,
    "wrist_flex":    0.0,
    "wrist_roll":    0.0,
    "gripper":       0.0,
}

# Per-joint torque / integral limits (same as build_default_pid)
TAU_LIMITS = {
    "shoulder_pan":  8.0,
    "shoulder_lift": 18.0,
    "elbow_flex":    15.0,
    "wrist_flex":    6.0,
    "wrist_roll":    3.0,
}
I_LIMITS = {jn: 2.0 for jn in DEFAULT_JOINTS}


# =====================================================================
# Data collector (same protocol as RealtimeJointPlotter.sample)
# =====================================================================
class DataCollector:
    """Records joint positions over time during a headless simulation run."""

    JOINT_NAMES = list(DEFAULT_JOINTS)

    def __init__(self):
        self.times: list[float] = []
        self.positions: dict[str, list[float]] = {jn: [] for jn in self.JOINT_NAMES}
        self._t0: float | None = None

    def sample(self, m, d, now=None):
        t = float(d.time)
        if self._t0 is None:
            self._t0 = t

        pos = get_positions_dict(m, d)          # degrees
        self.times.append(t - self._t0)
        for jn in self.JOINT_NAMES:
            self.positions[jn].append(pos[jn])

    def reset(self):
        self.times.clear()
        self.positions = {jn: [] for jn in self.JOINT_NAMES}
        self._t0 = None


# =====================================================================
# Build a JointPID from a config entry
# =====================================================================
def build_pid_from_config(config: dict, joint_names=DEFAULT_JOINTS) -> JointPID:
    gpj = config["gains_per_joint"]
    gains = {}
    for jn in joint_names:
        g = gpj[jn]
        gains[jn] = PIDGains(
            kp=g["kp"],
            ki=g["ki"],
            kd=g["kd"],
            i_limit=I_LIMITS.get(jn, 2.0),
            tau_limit=TAU_LIMITS.get(jn, 6.0),
        )
    return JointPID(list(joint_names), gains)


# =====================================================================
# Run one config through the full simulation sequence (headless)
# =====================================================================
def run_single_config(config: dict) -> DataCollector:
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)
    set_initial_pose(m, d, starting_position)

    pid = build_pid_from_config(config)
    collector = DataCollector()

    # Same motion sequence as run_mujoco_simulation.py
    # viewer=None  → headless,  realtime=False → full speed
    move_to_pose_pid(m, d, None, desired_zero,         duration=2.0, realtime=False, pid=pid, plotter=collector)
    hold_position_pid(m, d, None, desired_zero,         duration=2.0, realtime=False, pid=pid, plotter=collector)
    move_to_pose_pid(m, d, None, starting_position,     duration=2.0, realtime=False, pid=pid, plotter=collector)
    hold_position_pid(m, d, None, starting_position,    duration=2.0, realtime=False, pid=pid, plotter=collector)

    return collector


# =====================================================================
# Create a Plotly figure (matches Dash /DASH-UPDATE style)
# =====================================================================
JOINT_COLORS = {
    "shoulder_pan":  "#636EFA",
    "shoulder_lift": "#EF553B",
    "elbow_flex":    "#00CC96",
    "wrist_flex":    "#AB63FA",
    "wrist_roll":    "#FFA15A",
}


def make_figure(config: dict, collector: DataCollector) -> go.Figure:
    combo_id    = config["combo_id"]
    family      = config["family"]
    description = config["description"]
    kp          = config["kp_base"]
    ki          = config["ki_base"]
    kd          = config["kd_base"]
    expected    = config["expected_behavior"]

    title_text = (
        f"<b>{combo_id}</b>  [{family}] — {description}<br>"
        f"<span style='font-size:13px'>"
        f"Kp={kp}   Ki={ki}   Kd={kd}   |   {expected}"
        f"</span>"
    )

    fig = go.Figure()
    for jn in collector.JOINT_NAMES:
        fig.add_trace(go.Scatter(
            x=collector.times,
            y=collector.positions[jn],
            mode="lines",
            name=jn,
            line=dict(color=JOINT_COLORS.get(jn), width=2),
        ))

    fig.update_layout(
        title=dict(text=title_text, x=0.5, xanchor="center"),
        xaxis_title="Time (s)",
        yaxis_title="Joint Position (deg)",
        margin=dict(l=55, r=30, t=100, b=50),
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1),
        template="plotly_white",
        height=500,
        width=1100,
    )
    return fig


# =====================================================================
# Main
# =====================================================================
def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    with open(CONFIGS_PATH) as f:
        configs = json.load(f)

    total = len(configs)
    print(f"Loaded {total} configurations from {CONFIGS_PATH}")
    print(f"Output directory: {OUTPUT_DIR}/\n")

    all_figures: list[tuple[dict, go.Figure]] = []

    for i, cfg in enumerate(configs, 1):
        cid = cfg["combo_id"]
        print(f"[{i}/{total}] {cid:25s} — {cfg['description']}  ...", end="", flush=True)

        collector = run_single_config(cfg)
        fig = make_figure(cfg, collector)

        # Save individual HTML
        html_path = os.path.join(OUTPUT_DIR, f"{cid}.html")
        fig.write_html(html_path, include_plotlyjs="cdn")
        print(f"  OK  ({len(collector.times)} samples)")

        all_figures.append((cfg, fig))

    # ---- combined report ----
    print(f"\nGenerating combined report ({total} graphs)...")
    combined_path = os.path.join(OUTPUT_DIR, "all_configs_report.html")

    families_order = ["P", "PD", "PI", "PID"]
    grouped: dict[str, list[tuple[dict, go.Figure]]] = {f: [] for f in families_order}
    for cfg, fig in all_figures:
        grouped.setdefault(cfg["family"], []).append((cfg, fig))

    with open(combined_path, "w") as out:
        out.write(
            "<!DOCTYPE html><html><head><meta charset='utf-8'>\n"
            "<title>PID Configs Rubric — All Tests</title>\n"
            "<script src='https://cdn.plot.ly/plotly-2.35.2.min.js'></script>\n"
            "<style>\n"
            "  body  { font-family: 'Segoe UI', sans-serif; max-width:1200px; margin:0 auto; padding:20px; background:#fafafa; }\n"
            "  h1    { text-align:center; color:#333; }\n"
            "  h2    { margin-top:40px; padding:8px 16px; background:#e8eaf6; border-radius:6px; color:#283593; }\n"
            "  .card { background:#fff; border:1px solid #ddd; border-radius:8px; margin:24px 0; padding:16px;\n"
            "          box-shadow:0 2px 4px rgba(0,0,0,0.06); }\n"
            "  .meta { font-size:13px; color:#666; margin-bottom:8px; }\n"
            "</style>\n"
            "</head><body>\n"
            "<h1>PID Configurations Rubric — Automated Test Results</h1>\n"
            f"<p style='text-align:center;color:#888;'>{total} configurations tested</p>\n"
        )

        graph_idx = 0
        for fam in families_order:
            if not grouped[fam]:
                continue
            out.write(f"<h2>Family: {fam}</h2>\n")
            for cfg, fig in grouped[fam]:
                div_id = f"graph-{graph_idx}"
                out.write(
                    f"<div class='card'>\n"
                    f"  <div class='meta'>"
                    f"Kp(base)={cfg['kp_base']}  Ki(base)={cfg['ki_base']}  Kd(base)={cfg['kd_base']}"
                    f"</div>\n"
                )
                out.write(fig.to_html(full_html=False, include_plotlyjs=False, div_id=div_id))
                out.write("</div>\n")
                graph_idx += 1

        out.write("</body></html>\n")

    print(f"\nAll done!")
    print(f"  Individual graphs : {OUTPUT_DIR}/<combo_id>.html")
    print(f"  Combined report   : {combined_path}")


if __name__ == "__main__":
    main()
