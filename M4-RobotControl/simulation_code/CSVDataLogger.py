"""
CSV logger compatible with _PlotterProto of so101_mujoco_pid_utils.
"""

import csv
from pathlib import Path
import numpy as np
from so101_control import get_q_qd_dict


class CSVDataLogger:
    """Logger that saves simulation data to CSV"""
    
    def __init__(self, output_file: str, joint_names: list[str], target_pose_deg: dict[str, float]):
        self.output_file = Path(output_file)
        self.joint_names = joint_names
        self.target_pose_rad = {jn: np.deg2rad(target_pose_deg[jn]) for jn in joint_names}
        
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        
        self.file = open(self.output_file, 'w', newline='')
        self.writer = csv.writer(self.file)
        
        header = ['time']
        for jn in joint_names:
            header.extend([f'{jn}', f'target_{jn}', f'{jn}_velocity'])
        self.writer.writerow(header)
        
        print(f"CSV Logger: {self.output_file}")
    
    def sample(self, m, d, now: float = None):
        """Guard a data point (compatible with _PlotterProto)"""
        t = now if now is not None else float(d.time)
        
        q, qd = get_q_qd_dict(m, d, self.joint_names)
        
        row = [t]
        for jn in self.joint_names:
            row.append(np.rad2deg(q[jn]))
            row.append(np.rad2deg(self.target_pose_rad[jn]))
            row.append(np.rad2deg(qd[jn]))
        
        self.writer.writerow(row)
    
    def update_target(self, new_target_pose_deg: dict[str, float]):
        """Update target pose when changing between phases"""
        self.target_pose_rad = {jn: np.deg2rad(new_target_pose_deg[jn]) for jn in self.joint_names}
    
    def close(self):
        """Close the CSV file"""
        if hasattr(self, 'file') and not self.file.closed:
            self.file.close()
            print(f"CSV saved: {self.output_file}")

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
