# Análisis Cuantitativo - Experimentos de Rúbrica

## Resumen por Experimento

Métricas promediadas sobre todos los joints.

| Combo ID | Familia | SSE (°) | Overshoot (%) | Settling (s) | Rise (s) | RMSE (°) |
|----------|---------|---------|---------------|--------------|----------|----------|
| PD_1_PocoDamp        | PD      |    0.04 |          0.05 |        0.782 |    0.721 |    20.79 |
| PD_2_Critico         | PD      |    0.12 |          0.13 |        0.782 |    0.720 |    20.81 |
| PD_3_OverDamp        | PD      |    0.21 |          0.33 |        0.783 |    0.722 |    20.82 |
| PD_4_Rigido          | PD      |    0.12 |          0.16 |        0.782 |    0.721 |    20.81 |
| PD_5_ExcesivoKd      | PD      |    0.30 |          0.48 |        0.784 |    0.721 |    20.82 |
| PID_1_Conservador    | PID     |    0.10 |          0.13 |        0.782 |    0.720 |    20.81 |
| PID_2_Balanceado     | PID     |    0.11 |          0.13 |        0.781 |    0.720 |    20.80 |
| PID_3_Agresivo       | PID     |    0.10 |          0.10 |        0.782 |    0.721 |    20.81 |
| PID_4_KiDominante    | PID     |    0.06 |          0.06 |        0.782 |    0.721 |    20.80 |
| PID_5_KdDominante    | PID     |    0.23 |          0.34 |        0.782 |    0.721 |    20.83 |
| PI_1_KiBajo          | PI      |    0.04 |          0.06 |        0.782 |    0.721 |    20.79 |
| PI_2_KiMedio         | PI      |    0.03 |          0.06 |        0.782 |    0.721 |    20.79 |
| PI_3_KiAlto          | PI      |    0.03 |          0.05 |        0.782 |    0.721 |    20.79 |
| PI_4_KpBajo_KiAlto   | PI      |    0.04 |          0.06 |        0.782 |    0.720 |    20.80 |
| PI_5_Agresivo        | PI      |    0.02 |          0.05 |        0.782 |    0.720 |    20.79 |
| P_1_MuyBajo          | P       |    0.09 |          0.11 |        0.782 |    0.721 |    20.81 |
| P_2_Bajo             | P       |    0.05 |          0.07 |        0.782 |    0.721 |    20.80 |
| P_3_Medio            | P       |    0.03 |          0.06 |        0.782 |    0.721 |    20.79 |
| P_4_Alto             | P       |    0.02 |          0.05 |        0.782 |    0.720 |    20.79 |
| P_5_MuyAlto          | P       |    0.02 |          0.05 |        0.782 |    0.720 |    20.78 |

---

## Details per Joint


### PD_1_PocoDamp

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.01°
- Max Velocity: 0.1°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.01°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.01%
- Settling Time: 1.952s
- Rise Time: 1.798s
- RMSE: 51.87°
- Max Velocity: 97.9°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.04°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.06%
- Settling Time: 1.954s
- Rise Time: 1.800s
- RMSE: 51.87°
- Max Velocity: 66.8°/s

**Hold 0°:**
- Steady-State Error: 0.05°
- RMSE: 0.06°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 2.9°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 0.8°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°


### PD_2_Critico

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 18.3°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.03°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.21%
- Settling Time: 1.944s
- Rise Time: 1.790s
- RMSE: 51.56°
- Max Velocity: 106.0°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.06°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.11%
- Settling Time: 1.944s
- Rise Time: 1.790s
- RMSE: 51.63°
- Max Velocity: 92.0°/s

**Hold 0°:**
- Steady-State Error: 0.06°
- RMSE: 0.06°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.04°
- Max Velocity: 18.9°/s

**Hold 0°:**
- Steady-State Error: 0.05°
- RMSE: 0.04°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 7.8°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.03°


### PD_3_OverDamp

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 20.3°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.03°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.52%
- Settling Time: 1.938s
- Rise Time: 1.782s
- RMSE: 51.18°
- Max Velocity: 109.5°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.07°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.43%
- Settling Time: 1.936s
- Rise Time: 1.784s
- RMSE: 51.30°
- Max Velocity: 98.9°/s

**Hold 0°:**
- Steady-State Error: 0.06°
- RMSE: 0.07°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.04°
- Max Velocity: 21.2°/s

**Hold 0°:**
- Steady-State Error: 0.05°
- RMSE: 0.04°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 10.6°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.03°


### PD_4_Rigido

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 20.4°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.03°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.29%
- Settling Time: 1.942s
- Rise Time: 1.790s
- RMSE: 51.56°
- Max Velocity: 109.0°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.06°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.06%
- Settling Time: 1.946s
- Rise Time: 1.790s
- RMSE: 51.59°
- Max Velocity: 98.3°/s

**Hold 0°:**
- Steady-State Error: 0.06°
- RMSE: 0.06°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.04°
- Max Velocity: 21.6°/s

**Hold 0°:**
- Steady-State Error: 0.05°
- RMSE: 0.04°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 10.9°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.03°


### PD_5_ExcesivoKd

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 19.0°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.03°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.80%
- Settling Time: 1.926s
- Rise Time: 1.772s
- RMSE: 50.76°
- Max Velocity: 122.2°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.11°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.51%
- Settling Time: 1.932s
- Rise Time: 1.772s
- RMSE: 51.07°
- Max Velocity: 101.2°/s

**Hold 0°:**
- Steady-State Error: 0.05°
- RMSE: 0.08°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.04°
- Max Velocity: 25.8°/s

**Hold 0°:**
- Steady-State Error: 0.05°
- RMSE: 0.04°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 9.3°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.03°


### PID_1_Conservador

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 19.1°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.03°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.20%
- Settling Time: 1.946s
- Rise Time: 1.792s
- RMSE: 51.60°
- Max Velocity: 100.8°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.06°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.14%
- Settling Time: 1.946s
- Rise Time: 1.794s
- RMSE: 51.67°
- Max Velocity: 89.8°/s

**Hold 0°:**
- Steady-State Error: 0.06°
- RMSE: 0.06°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.04°
- Max Velocity: 18.1°/s

**Hold 0°:**
- Steady-State Error: 0.05°
- RMSE: 0.04°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 7.7°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.03°


### PID_2_Balanceado

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 19.2°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.03°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.17%
- Settling Time: 1.948s
- Rise Time: 1.794s
- RMSE: 51.58°
- Max Velocity: 112.1°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.06°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.02%
- Settling Time: 1.944s
- Rise Time: 1.792s
- RMSE: 51.65°
- Max Velocity: 95.6°/s

**Hold 0°:**
- Steady-State Error: 0.06°
- RMSE: 0.06°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.04°
- Max Velocity: 19.5°/s

**Hold 0°:**
- Steady-State Error: 0.05°
- RMSE: 0.04°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 8.8°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.03°


### PID_3_Agresivo

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 18.9°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.03°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.19%
- Settling Time: 1.950s
- Rise Time: 1.796s
- RMSE: 51.59°
- Max Velocity: 104.9°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.06°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.07%
- Settling Time: 1.946s
- Rise Time: 1.792s
- RMSE: 51.63°
- Max Velocity: 98.6°/s

**Hold 0°:**
- Steady-State Error: 0.05°
- RMSE: 0.06°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.04°
- Max Velocity: 20.9°/s

**Hold 0°:**
- Steady-State Error: 0.05°
- RMSE: 0.04°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 10.9°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.03°


### PID_4_KiDominante

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 22.1°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.03°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.03%
- Settling Time: 1.952s
- Rise Time: 1.798s
- RMSE: 51.77°
- Max Velocity: 103.2°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.05°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.16%
- Settling Time: 1.950s
- Rise Time: 1.798s
- RMSE: 51.80°
- Max Velocity: 92.5°/s

**Hold 0°:**
- Steady-State Error: 0.05°
- RMSE: 0.05°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.04°
- Max Velocity: 19.7°/s

**Hold 0°:**
- Steady-State Error: 0.05°
- RMSE: 0.04°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 8.2°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.01°


### PID_5_KdDominante

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 22.6°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.03°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.44%
- Settling Time: 1.936s
- Rise Time: 1.780s
- RMSE: 50.87°
- Max Velocity: 122.2°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.07°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.39%
- Settling Time: 1.932s
- Rise Time: 1.780s
- RMSE: 51.12°
- Max Velocity: 99.3°/s

**Hold 0°:**
- Steady-State Error: 0.06°
- RMSE: 0.06°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.04°
- Max Velocity: 25.4°/s

**Hold 0°:**
- Steady-State Error: 0.05°
- RMSE: 0.04°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 9.2°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.03°


### PI_1_KiBajo

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.01°
- Max Velocity: 0.1°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.01°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.04%
- Settling Time: 1.952s
- Rise Time: 1.798s
- RMSE: 51.85°
- Max Velocity: 90.5°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.02°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.09%
- Settling Time: 1.954s
- Rise Time: 1.800s
- RMSE: 51.86°
- Max Velocity: 80.1°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 5.1°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.02°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 1.4°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.02°


### PI_2_KiMedio

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.01°
- Max Velocity: 0.1°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.01°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.04%
- Settling Time: 1.952s
- Rise Time: 1.798s
- RMSE: 51.86°
- Max Velocity: 90.5°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.02°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.10%
- Settling Time: 1.954s
- Rise Time: 1.800s
- RMSE: 51.86°
- Max Velocity: 80.1°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 5.1°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.02°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 1.4°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.02°


### PI_3_KiAlto

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.01°
- Max Velocity: 0.1°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.01°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.06%
- Settling Time: 1.952s
- Rise Time: 1.800s
- RMSE: 51.87°
- Max Velocity: 90.5°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.02°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.11%
- Settling Time: 1.954s
- Rise Time: 1.800s
- RMSE: 51.87°
- Max Velocity: 80.1°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 5.1°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.02°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 1.4°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.02°


### PI_4_KpBajo_KiAlto

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 0.1°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.04%
- Settling Time: 1.952s
- Rise Time: 1.798s
- RMSE: 51.83°
- Max Velocity: 94.1°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.02°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.12%
- Settling Time: 1.954s
- Rise Time: 1.800s
- RMSE: 51.83°
- Max Velocity: 80.7°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 5.3°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.02°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 1.4°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.03°


### PI_5_Agresivo

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.01°
- Max Velocity: 0.1°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.01°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.08%
- Settling Time: 1.954s
- Rise Time: 1.800s
- RMSE: 51.89°
- Max Velocity: 89.1°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.01°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.12%
- Settling Time: 1.954s
- Rise Time: 1.800s
- RMSE: 51.89°
- Max Velocity: 80.6°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.01°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 4.8°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 1.3°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°


### P_1_MuyBajo

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 0.1°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.02%
- Settling Time: 1.946s
- Rise Time: 1.794s
- RMSE: 51.69°
- Max Velocity: 99.7°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.03°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.05%
- Settling Time: 1.950s
- Rise Time: 1.796s
- RMSE: 51.69°
- Max Velocity: 81.5°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.03°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 5.4°/s

**Hold 0°:**
- Steady-State Error: 0.04°
- RMSE: 0.03°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.03°
- Max Velocity: 1.5°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.03°


### P_2_Bajo

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 0.1°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.01%
- Settling Time: 1.950s
- Rise Time: 1.798s
- RMSE: 51.82°
- Max Velocity: 92.1°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.02°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.08%
- Settling Time: 1.952s
- Rise Time: 1.798s
- RMSE: 51.82°
- Max Velocity: 80.3°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.02°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 5.2°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.02°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 1.4°/s

**Hold 0°:**
- Steady-State Error: 0.03°
- RMSE: 0.02°


### P_3_Medio

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.01°
- Max Velocity: 0.1°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.01°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.05%
- Settling Time: 1.952s
- Rise Time: 1.798s
- RMSE: 51.87°
- Max Velocity: 89.2°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.02°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.09%
- Settling Time: 1.954s
- Rise Time: 1.800s
- RMSE: 51.87°
- Max Velocity: 80.4°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 4.9°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 1.3°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°


### P_4_Alto

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.01°
- Max Velocity: 0.1°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.01°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.07%
- Settling Time: 1.952s
- Rise Time: 1.800s
- RMSE: 51.90°
- Max Velocity: 86.9°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.01°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.10%
- Settling Time: 1.954s
- Rise Time: 1.800s
- RMSE: 51.90°
- Max Velocity: 80.8°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.01°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.01°
- Max Velocity: 5.1°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.01°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.02°
- Max Velocity: 1.3°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.02°


### P_5_MuyAlto

#### Shoulder Pan

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.01°
- Max Velocity: 0.0°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.01°

#### Shoulder Lift

**Move to 0°:**
- Overshoot: 0.09%
- Settling Time: 1.954s
- Rise Time: 1.800s
- RMSE: 51.92°
- Max Velocity: 87.7°/s

**Hold 0°:**
- Steady-State Error: 0.00°
- RMSE: 0.01°

#### Elbow Flex

**Move to 0°:**
- Overshoot: 0.11%
- Settling Time: 1.954s
- Rise Time: 1.800s
- RMSE: 51.92°
- Max Velocity: 81.2°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.01°

#### Wrist Flex

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.01°
- Max Velocity: 7.6°/s

**Hold 0°:**
- Steady-State Error: 0.01°
- RMSE: 0.01°

#### Wrist Roll

**Move to 0°:**
- Overshoot: 0.00%
- Settling Time: 0.000s
- Rise Time: 0.000s
- RMSE: 0.01°
- Max Velocity: 1.2°/s

**Hold 0°:**
- Steady-State Error: 0.02°
- RMSE: 0.01°


---

## Definitions

- **SSE (Steady-State Error)**: Average absolute error in the last 0.5s of a hold phase
- **Overshoot**: Maximum overshoot as % of total change
- **Settling Time**: Time to enter and remain within ±2° of target
- **Rise Time**: Time to reach 90% of total change
- **RMSE**: Root Mean Square Error during the phase
- **Max Velocity**: Maximum velocity reached during the phase
