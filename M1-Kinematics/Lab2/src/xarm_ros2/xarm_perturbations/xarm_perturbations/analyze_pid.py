#!/usr/bin/env python3
"""
analyze_pid.py — Analiza robot_evaluation.csv y sugiere ganancias PID.

Modos de uso:
  1) Análisis de CSV:
       python3 analyze_pid.py [ruta_al_csv]

  2) Calcular ganancias desde identificación de planta (new.py):
       python3 analyze_pid.py --sysid K Tau [freq_hz]
       Ejemplo: python3 analyze_pid.py --sysid 0.1299 0.1001 0.06
"""
import sys
import pathlib
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

# ── Modo --sysid: calcular ganancias desde K y Tau ───────────────────────────
if '--sysid' in sys.argv:
    idx = sys.argv.index('--sysid')
    try:
        K   = float(sys.argv[idx + 1])
        Tau = float(sys.argv[idx + 2])
        freq = float(sys.argv[idx + 3]) if len(sys.argv) > idx + 3 else 0.06
    except (IndexError, ValueError):
        print("Uso: python3 analyze_pid.py --sysid K Tau [freq_hz]")
        sys.exit(1)

    w_traj = 2 * np.pi * freq          # frecuencia angular de la trayectoria

    print(f"\n{'='*58}")
    print(f"  CÁLCULO DE GANANCIAS PID  (K={K:.4f}, τ={Tau:.4f}s, f={freq}Hz)")
    print(f"{'='*58}")
    print(f"  Planta identificada: G(s) = {K:.4f} / ({Tau:.4f}s + 1)")
    print(f"  Lazo de posición:    G_pos(s) = K / (s·(τs+1))")
    print(f"  Trayectoria ω = 2π·{freq} = {w_traj:.3f} rad/s\n")

    # ── Método 1: Cancelación polo-cero (óptimo para 1er orden) ──────────────
    # Kd = Kp·τ → cancela polo lento → PM ≈ 90° siempre
    # Ancho de banda deseado: n·ω_traj (n=10 agresivo, n=5 conservador)
    results = {}
    for label, n in [("Agresivo  (10·ω)", 10), ("Moderado   (7·ω)", 7), ("Conserv.  (5·ω)", 5)]:
        wc   = n * w_traj
        kp   = wc / K
        kd   = kp * Tau      # cancelación polo-cero
        ki   = 0.0
        pm   = 90.0          # con cancelación polo-cero PM ≈ 90° siempre
        results[label] = (kp, kd, ki, wc, pm)

    # ── Método 2: Ziegler-Nichols (tabla clásica para 1er orden + τ) ─────────
    # Con θ≈0 se usa la forma Kp = 1/(K·τ) → banda ancha = 1/τ
    kp_zn = 1.0 / (K * Tau)
    kd_zn = kp_zn * Tau / 4.0
    ki_zn = 0.0
    wc_zn = kp_zn * K
    pm_zn = 90.0 - np.degrees(np.arctan(wc_zn * Tau))
    results["Z-N approx (θ≈0)"] = (kp_zn, kd_zn, ki_zn, wc_zn, pm_zn)

    print(f"  {'Método':<22} {'Kp':>8} {'Kd':>8} {'Ki':>6} {'ωc(rad/s)':>10} {'PM(°)':>7}")
    print(f"  {'-'*62}")
    for lbl, (kp, kd, ki, wc, pm) in results.items():
        print(f"  {lbl:<22} {kp:>8.3f} {kd:>8.3f} {ki:>6.3f} {wc:>10.3f} {pm:>7.1f}")

    print(f"\n  ► Recomendado (Moderado, cancelación polo-cero):")
    kp_r, kd_r, ki_r, _, _ = results["Moderado   (7·ω)"]
    print(f"    --ros-args -p kp:='[{kp_r:.3f},{kp_r:.3f},0.0]' "
          f"-p kd:='[{kd_r:.3f},{kd_r:.3f},0.0]' "
          f"-p ki:='[0.0,0.0,0.0]'\n")

    print(f"  NOTA: Kd ≈ Kp·τ cancela el polo lento del servo → PM≈90°")
    print(f"  Tus ganancias actuales Kp=32.18, Kd=3.42 → Td={3.42/32.18:.4f}s ≈ τ={Tau:.4f}s ✓\n")
    sys.exit(0)

# ── Cargar CSV ───────────────────────────────────────────────────────────────
csv_path = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else \
           pathlib.Path(__file__).parent / 'robot_evaluation.csv'

if not csv_path.exists():
    print(f"[ERROR] No se encontró: {csv_path}")
    sys.exit(1)

df = pd.read_csv(csv_path)
print(f"\n✅ CSV cargado: {csv_path}  ({len(df)} muestras)\n")

# ── Métricas por eje ─────────────────────────────────────────────────────────
axes = ['x', 'y', 'z']
metrics = {}
for ax in axes:
    e = df[f'err_{ax}'].to_numpy()
    de = np.diff(e) / np.diff(df['time'].to_numpy())
    metrics[ax] = {
        'mae':  np.mean(np.abs(e)),
        'rmse': np.sqrt(np.mean(e**2)),
        'max':  np.max(np.abs(e)),
        'bias': np.mean(e),          # sesgo persistente → necesita Ki
        'osc':  np.std(de),          # oscilación (varianza de derivada)
    }

print("─── Métricas de error ──────────────────────────────────────────────")
print(f"{'Eje':<5} {'MAE [m]':>10} {'RMSE [m]':>10} {'Max [m]':>10} {'Bias [m]':>10} {'Oscil.':>10}")
for ax in axes:
    m = metrics[ax]
    print(f"  {ax.upper():<4} {m['mae']:>10.4f} {m['rmse']:>10.4f} {m['max']:>10.4f} {m['bias']:>10.4f} {m['osc']:>10.4f}")

# ── Leer ganancias actuales del CSV (no disponibles directamente) ─────────────
# Como el CSV no guarda Kp/Kd/Ki, pedimos al usuario o usamos defaults
print("\n─── Ganancias actuales (ingresa los valores usados) ────────────────")
try:
    kp_in = input("  Kp [x,y,z] (Enter para [30.18, 30.18, 0.0]): ").strip()
    kp = np.array(eval(kp_in)) if kp_in else np.array([30.18, 30.18, 0.0])
    kd_in = input("  Kd [x,y,z] (Enter para [3.42, 3.42, 0.0]): ").strip()
    kd = np.array(eval(kd_in)) if kd_in else np.array([3.42, 3.42, 0.0])
    ki_in = input("  Ki [x,y,z] (Enter para [0.0, 0.0, 0.0]): ").strip()
    ki = np.array(eval(ki_in)) if ki_in else np.array([0.0, 0.0, 0.0])
except Exception:
    kp = np.array([30.18, 30.18, 0.0])
    kd = np.array([3.42, 3.42, 0.0])
    ki = np.array([0.0, 0.0, 0.0])

# ── Sugerencia de ganancias ──────────────────────────────────────────────────
OSC_THRESH  = 0.05   # m/s²: si oscilación > esto → sistema inestable
BIAS_THRESH = 0.003  # m:    sesgo persistente que justifica Ki
MAE_HIGH    = 0.008  # m:    error demasiado alto, subir Kp
MAE_OK      = 0.003  # m:    error aceptable, no tocar Kp

new_kp = kp.copy()
new_kd = kd.copy()
new_ki = ki.copy()
reasons = {ax: [] for ax in axes}

for i, ax in enumerate(axes):
    m = metrics[ax]
    if m['osc'] > OSC_THRESH:
        # Oscilando: bajar Kp, subir Kd
        new_kp[i] *= 0.80
        new_kd[i] *= 1.30
        reasons[ax].append(f"oscilación alta ({m['osc']:.4f}) → Kp↓20%, Kd↑30%")
    elif m['mae'] > MAE_HIGH:
        # Error grande sin oscilación: subir Kp
        new_kp[i] *= 1.25
        reasons[ax].append(f"MAE alto ({m['mae']*1000:.1f} mm) → Kp↑25%")
    elif m['mae'] < MAE_OK:
        # Muy bien, no tocar
        reasons[ax].append(f"MAE OK ({m['mae']*1000:.1f} mm) → sin cambio")

    if abs(m['bias']) > BIAS_THRESH and m['osc'] <= OSC_THRESH:
        new_ki[i] = max(ki[i], 0.5)
        reasons[ax].append(f"sesgo {m['bias']*1000:.1f} mm → Ki={new_ki[i]:.2f}")

print("\n─── Sugerencia de ganancias ────────────────────────────────────────")
for i, ax in enumerate(axes):
    print(f"  {ax.upper()}: Kp {kp[i]:.3f}→{new_kp[i]:.3f}  "
          f"Kd {kd[i]:.3f}→{new_kd[i]:.3f}  "
          f"Ki {ki[i]:.3f}→{new_ki[i]:.3f}")
    for r in reasons[ax]:
        print(f"       ↳ {r}")

print(f"\n  --ros-args "
      f"-p kp:='[{new_kp[0]:.3f},{new_kp[1]:.3f},{new_kp[2]:.3f}]' "
      f"-p kd:='[{new_kd[0]:.3f},{new_kd[1]:.3f},{new_kd[2]:.3f}]' "
      f"-p ki:='[{new_ki[0]:.3f},{new_ki[1]:.3f},{new_ki[2]:.3f}]'\n")

# ── Gráficas ─────────────────────────────────────────────────────────────────
t   = df['time'].to_numpy()
fig, axes_plt = plt.subplots(2, 2, figsize=(13, 9))
fig.suptitle('Análisis PID — Lemniscata', fontsize=13, fontweight='bold')

# 1. Trayectoria XY
ax1 = axes_plt[0, 0]
ax1.plot(df['des_x'].to_numpy(), df['des_y'].to_numpy(), 'b--', lw=1.5, label='Deseada')
ax1.plot(df['act_x'].to_numpy(), df['act_y'].to_numpy(), 'r-',  lw=1.0, label='Real')
ax1.set_xlabel('X [m]'); ax1.set_ylabel('Y [m]')
ax1.set_title('Trayectoria XY'); ax1.legend(); ax1.axis('equal'); ax1.grid(True)

# 2. Error por eje vs tiempo
ax2 = axes_plt[0, 1]
for ax, color in zip(axes, ['tab:blue', 'tab:orange', 'tab:green']):
    ax2.plot(t, df[f'err_{ax}'].to_numpy() * 1000, color=color, lw=0.8, label=f'e_{ax.upper()}')
ax2.axhline(0, color='k', lw=0.5)
ax2.set_xlabel('t [s]'); ax2.set_ylabel('Error [mm]')
ax2.set_title('Error por eje'); ax2.legend(); ax2.grid(True)

# 3. Error XY total (norma)
ax3 = axes_plt[1, 0]
err_xy = np.sqrt(df['err_x'].to_numpy()**2 + df['err_y'].to_numpy()**2) * 1000
ax3.plot(t, err_xy, 'purple', lw=0.9)
ax3.axhline(np.mean(err_xy), color='red', ls='--', lw=1, label=f'MAE={np.mean(err_xy):.2f} mm')
ax3.set_xlabel('t [s]'); ax3.set_ylabel('||e_XY|| [mm]')
ax3.set_title('Error XY total'); ax3.legend(); ax3.grid(True)

# 4. Velocidad comandada
ax4 = axes_plt[1, 1]
ax4.plot(t, df['vel_mag'].to_numpy(), 'darkorange', lw=0.8)
ax4.set_xlabel('t [s]'); ax4.set_ylabel('|v| [m/s]')
ax4.set_title('Velocidad comandada'); ax4.grid(True)

fig.tight_layout()
plot_path = csv_path.parent / 'pid_analysis.png'
fig.savefig(plot_path, dpi=150)
print(f"📈 Gráfica guardada en: {plot_path}")
plt.show()
