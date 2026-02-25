import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores
from pathlib import Path

def extract_data(bag_path):
    velocities = []
    typestore = get_typestore(Stores.ROS2_HUMBLE) 
    
    try:
        with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
            connections = [c for c in reader.connections if c.topic == '/servo_server/delta_twist_cmds']
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                v = msg.twist.linear
                velocities.append([timestamp / 1e9, v.x, v.y, v.z])
    except Exception as e:
        print(f"Error en {bag_path}: {e}")
                
    return np.array(velocities)

# --- PROCESAMIENTO ---
bags = {'Baseline': 'P6.1', 'Sine': 'P6.2', 'Gaussian': 'P6.3'}
results = {}

for label, path in bags.items():
    p = Path(path)
    if p.exists():
        print(f"🔍 Analizando {label} en {path}...")
        vel = extract_data(path)
        if len(vel) > 0:
            results[label] = {'vel': vel}
            print(f"✅ {label}: {len(vel)} comandos de velocidad procesados.")

# --- INTEGRACIÓN CINEMÁTICA Y GRÁFICAS ---
if results:
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))
    colors = {'Baseline': '#1f77b4', 'Sine': '#ff7f0e', 'Gaussian': '#2ca02c'}

    # 1. Integración de Euler (Velocidad -> Posición)
    for label, data in results.items():
        t_vel = data['vel'][:, 0]
        dt = np.diff(t_vel)
        dt = np.insert(dt, 0, 0) # Mantener misma longitud
        
        # Posición = suma acumulada de (v * dt)
        pos_x = np.cumsum(data['vel'][:, 1] * dt)
        pos_y = np.cumsum(data['vel'][:, 2] * dt)
        pos_z = np.cumsum(data['vel'][:, 3] * dt)
        
        data['pos_int'] = np.column_stack((t_vel, pos_x, pos_y, pos_z))

    # Recortar al tamaño mínimo para comparar parejo
    min_len = min(len(data['pos_int']) for data in results.values())
    
    # Referencia para el RMSE
    base_pos = results['Baseline']['pos_int'][:min_len, 1:]

    print("\n--- REPORTE TÉCNICO TASK 7 ---")
    rmse_vals = []
    labels = list(results.keys())

    for label in labels:
        data = results[label]
        
        # Tiempos y Posiciones normalizados
        t = data['pos_int'][:min_len, 0] - data['pos_int'][0, 0]
        pos_x = data['pos_int'][:min_len, 1]
        
        # Gráfica 1: Posición (Ahora sí se verán las ondas)
        axs[0].plot(t, pos_x, label=label, color=colors[label])
        
        # Gráfica 2: Velocidad (Tu gráfica que ya se veía genial)
        t_v = data['vel'][:min_len, 0] - data['vel'][0, 0]
        mag = np.linalg.norm(data['vel'][:min_len, 1:], axis=1)
        axs[1].plot(t_v, mag, label=label, color=colors[label], alpha=0.7)

        # Cálculos Matemáticos
        current_pos = data['pos_int'][:min_len, 1:]
        err = current_pos - base_pos
        rmse = np.sqrt(np.mean(err**2))
        max_err = np.max(np.abs(err))
        rmse_vals.append(rmse)
        
        print(f"Experimento {label:8}: RMSE Total = {rmse:.5f} m | Max Error = {max_err:.5f} m")

    # Configuración Gráfica 1
    axs[0].set_title("Reconstructed EE Displacement (X-axis) - Kinematic Integration")
    axs[0].set_ylabel("Displacement [m]")
    axs[0].legend()
    axs[0].grid(True)

    # Configuración Gráfica 2
    axs[1].axhline(y=0.10, color='r', linestyle='--', label="Limit (0.10 m/s)")
    axs[1].set_title("Commanded Velocity Magnitude (Saturation Check)")
    axs[1].set_ylabel("Speed [m/s]")
    axs[1].set_ylim([0, 0.15])
    axs[1].legend()
    axs[1].grid(True)

    # Configuración Gráfica 3
    axs[2].bar(labels, rmse_vals, color=[colors[l] for l in labels])
    axs[2].set_title("Total RMSE Comparison (Relative to Baseline)")
    axs[2].set_ylabel("RMSE [m]")
    axs[2].grid(axis='y')

    plt.tight_layout()
    plt.savefig("Analisis_FRIDA_Definitivo.png")
    print("\n🚀 ¡Éxito! Gráficas guardadas en 'Analisis_FRIDA_Definitivo.png'")
    plt.show()