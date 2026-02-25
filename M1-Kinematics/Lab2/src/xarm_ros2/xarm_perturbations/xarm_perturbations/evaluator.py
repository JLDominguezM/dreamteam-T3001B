import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def evaluate_performance(csv_file):
    # Cargar datos
    df = pd.DataFrame(pd.read_csv(csv_file))
    
    # 1. Cálculos Matemáticos
    # RMSE por eje
    rmse_x = np.sqrt(np.mean(df['err_x']**2))
    rmse_y = np.sqrt(np.mean(df['err_y']**2))
    rmse_z = np.sqrt(np.mean(df['err_z']**2))
    
    # RMSE Total (Magnitud del error 3D)
    total_rmse = np.sqrt(np.mean(df['err_x']**2 + df['err_y']**2 + df['err_z']**2))
    
    # Error Máximo Absoluto
    max_err_x = np.max(np.abs(df['err_x']))
    max_err_y = np.max(np.abs(df['err_y']))
    max_err_z = np.max(np.abs(df['err_z']))
    max_total_err = np.max(np.sqrt(df['err_x']**2 + df['err_y']**2 + df['err_z']**2))

    print("=== RESULTADOS DE EVALUACIÓN ===")
    print(f"RMSE X: {rmse_x:.6f} m")
    print(f"RMSE Y: {rmse_y:.6f} m")
    print(f"RMSE Z: {rmse_z:.6f} m")
    print(f"RMSE TOTAL: {total_rmse:.6f} m")
    print("--------------------------------")
    print(f"Max Abs Error X: {max_err_x:.6f} m")
    print(f"Max Abs Error Y: {max_err_y:.6f} m")
    print(f"Max Abs Error Z: {max_err_z:.6f} m")
    print(f"Max Abs Error 3D: {max_total_err:.6f} m")
    print("================================")

    # 2. Generación de Gráficas
    plt.style.use('bmh')
    fig, axs = plt.subplots(1, 3, figsize=(18, 5))

    # Gráfica A: Desired vs Actual (Plano XY)
    axs[0].plot(df['des_x'], df['des_y'], label='Deseada (Lemniscata)', linestyle='--', color='black')
    axs[0].plot(df['act_x'], df['act_y'], label='Actual', alpha=0.7, color='blue')
    axs[0].set_title('Trayectoria XY: Deseada vs Actual')
    axs[0].set_xlabel('X [m]')
    axs[0].set_ylabel('Y [m]')
    axs[0].legend()
    axs[0].axis('equal')

    # Gráfica B: Error over time
    axs[1].plot(df['time'], df['err_x'], label='Error X', color='blue', alpha=0.8)
    axs[1].plot(df['time'], df['err_y'], label='Error Y', color='red', alpha=0.8)
    axs[1].plot(df['time'], df['err_z'], label='Error Z', color='magenta', alpha=0.8)
    axs[1].set_title('Error Cartesiano en el Tiempo')
    axs[1].set_xlabel('Tiempo [s]')
    axs[1].set_ylabel('Error [m]')
    axs[1].legend()

    # Gráfica C: Commanded Velocity Magnitude
    axs[2].plot(df['time'], df['vel_mag'], label='Magnitud Vel', color='green')
    axs[2].axhline(y=0.10, color='red', linestyle='--', label='Max Speed (0.10 m/s)')
    axs[2].set_title('Magnitud de Velocidad Comandada')
    axs[2].set_xlabel('Tiempo [s]')
    axs[2].set_ylabel('Velocidad [m/s]')
    axs[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Asegúrate de correr este script en la misma carpeta donde se guardó el CSV
    evaluate_performance('sin.csv')