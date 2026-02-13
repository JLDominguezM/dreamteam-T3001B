import os
import mujoco
import numpy as np
import matplotlib.pyplot as plt
from so101_mujoco_pid_utils import (
    move_to_pose_pid, hold_position_pid, 
    JointPID, PIDGains, build_default_perturbations,
    DEFAULT_JOINTS
)

MODEL_PATH = "model/scene_urdf.xml"
RESULTS_DIR = "results"
os.makedirs(RESULTS_DIR, exist_ok=True)


JOINT_TARGET = "shoulder_lift" 
TARGET_ANGLE = 30.0 

experiments = {
    "P_01":   {"kp": 5000.0,  "ki": 0.0, "kd": 0.0},
    "P_02":   {"kp": 10000.0, "ki": 0.0, "kd": 0.0},
    "P_03":   {"kp": 15000.0, "ki": 0.0, "kd": 0.0}, 
    "P_04":   {"kp": 20000.0, "ki": 0.0, "kd": 0.0},
    "P_05":   {"kp": 28000.0, "ki": 0.0, "kd": 0.0},

    "PD_01":  {"kp": 20000.0, "ki": 0.0, "kd": 1.0},
    "PD_02":  {"kp": 20000.0, "ki": 0.0, "kd": 3.0},
    "PD_03":  {"kp": 20000.0, "ki": 0.0, "kd": 5.0},
    "PD_04": {"kp": 20000.0, "ki": 0.0, "kd": 7.0},
    "PD_05": {"kp": 20000.0, "ki": 0.0, "kd": 10.0},

    "PI_01":  {"kp": 20000.0, "ki": 10.0,   "kd": 0.0},
    "PI_02":   {"kp": 20000.0, "ki": 100.0,  "kd": 0.0}, 
    "PI_03":  {"kp": 20000.0, "ki": 500.0,  "kd": 0.0},
    "PI_04": {"kp": 20000.0, "ki": 2000.0, "kd": 0.0},
    "PI_05": {"kp": 20000.0, "ki": 5000.0, "kd": 0.0}, 

    "PID_01": {"kp": 20000.0, "ki": 10.0,  "kd": 5.0}, 
    "PID_02": {"kp": 20000.0, "ki": 100.0, "kd": 5.0}, 
    "PID_03": {"kp": 20000.0, "ki": 500.0, "kd": 5.0}, 
    "PID_04": {"kp": 20000.0, "ki": 1000.0,"kd": 5.0}, 
    "PID_05": {"kp": 25000.0, "ki": 2000.0,"kd": 150.0},
}

class DataCollector:
    def __init__(self, joint_name, model):
        self.joint_name = joint_name
        self.times = []
        self.positions = []
        self.joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        self.qadr = model.jnt_qposadr[self.joint_id]

    def sample(self, m, d, now=None):
        self.times.append(d.time)
        self.positions.append(np.rad2deg(d.qpos[self.qadr]))

def run_experiment(exp_name, gains):
    print(f"--> Ejecutando: {exp_name} ... ", end="")
    
    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)
    
    m.actuator_forcerange[:, :] = [-100.0, 100.0]
    m.actuator_ctrlrange[:, :] = [-100.0, 100.0]

    pid_gains_dict = {}
    for name in DEFAULT_JOINTS:
        pid_gains_dict[name] = PIDGains(
            kp=gains["kp"], 
            ki=gains["ki"], 
            kd=gains["kd"], 
            i_limit=100.0,  
            tau_limit=100.0 
        )
    
    my_pid = JointPID(DEFAULT_JOINTS, pid_gains_dict)
    perturb = build_default_perturbations(DEFAULT_JOINTS)
    collector = DataCollector(JOINT_TARGET, m)

    move_to_pose_pid(m, d, None, 
                     target_pose_deg={j: 0.0 for j in DEFAULT_JOINTS},
                     duration=1.0, realtime=False, pid=my_pid, perturb=perturb, plotter=collector)
    
    
    target_pose = {j: TARGET_ANGLE if j == JOINT_TARGET else 0.0 for j in DEFAULT_JOINTS}
    
    hold_position_pid(m, d, None, 
                      hold_pose_deg=target_pose,
                      duration=2.0,
                      realtime=False, pid=my_pid, perturb=perturb, plotter=collector)

    plt.figure(figsize=(10, 6))

    plt.plot(collector.times, collector.positions, label=f'Respuesta Real', linewidth=2)
    
    plt.axhline(y=TARGET_ANGLE, color='r', linestyle='--', label=f'Setpoint ({TARGET_ANGLE}°)')
    
    plt.title(f"Experimento: {exp_name}\n{gains}")
    plt.xlabel("Tiempo (s)")
    plt.ylabel("Ángulo (grados)")
    plt.grid(True, which='both', linestyle='--', alpha=0.7)
    plt.legend()
    plt.tight_layout()
    
    plt.savefig(f"{RESULTS_DIR}/{exp_name}.png")
    plt.close()
    print("OK")

if __name__ == "__main__":
    print(f"Iniciando Step Response Tests.")
    print(f"Target: {JOINT_TARGET} saltando a {TARGET_ANGLE} grados.")
    
    for name, params in experiments.items():
        try:
            run_experiment(name, params)
        except Exception as e:
            print(f"ERROR en {name}: {e}")
            
    print(f"\n¡Listo graficas generadas en carpeta '{RESULTS_DIR}'")