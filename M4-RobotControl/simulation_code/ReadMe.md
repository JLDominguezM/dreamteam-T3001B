# SO-101 Robot Control - M4

Sistema para evaluar 20 configuraciones PID en el robot SO-101 mediante simulación en MuJoCo.

## Tabla de Contenidos

- [1. Setup Docker](#1-setup-docker)
- [2. Ejecutar Simulacion con GUI](#2-ejecutar-simulacion-con-gui)
- [3. Pipeline de Experimentos PID](#3-pipeline-de-experimentos-pid)
- [4. Uso Detallado](#4-uso-detallado)
- [5. Las 20 Configuraciones PID](#5-las-20-configuraciones-pid)
- [6. Resultados y Analisis](#6-resultados-y-analisis)
- [7. Archivos del Sistema](#7-archivos-del-sistema)
- [8. Troubleshooting](#8-troubleshooting)
- [Referencias](#referencias)

---

## 1. Setup Docker

### 1.1 Construir y levantar el contenedor

Desde la raiz del repositorio:

```bash
cd M4-RobotControl/simulation_code
docker compose up --build -d
```

Verifica que el contenedor este corriendo:

```bash
docker ps
```

Deberias ver `so101_container` en la lista con estado `Up`.

### 1.2 Entrar al contenedor

```bash
docker exec -it so101_container bash
```

Una vez dentro, el workspace de trabajo es:

```bash
cd /ros2_ws/src/simulation_code
```

---

## 2. Ejecutar Simulacion con GUI

Si necesitas visualizar la simulacion con interfaz grafica (MuJoCo viewer), primero debes configurar el display.

### 2.1 Permitir acceso al display (desde el host)

```bash
xhost +local:docker
```

### 2.2 Configurar la variable DISPLAY (dentro del contenedor)

```bash
export DISPLAY=:0
```

### 2.3 Lanzar la simulacion

```bash
cd /ros2_ws/src/simulation_code
python3 run_mujoco_simulation.py
```

> **Nota:** La GUI solo es necesaria para visualizacion. Los experimentos PID corren sin GUI (headless).

---

## 3. Pipeline de Experimentos PID

### Quick Start (un solo comando)

Dentro del contenedor, ejecuta todo el pipeline de una vez:

```bash
cd /ros2_ws/src/simulation_code

rm -rf logs_rubrica/ plots_rubrica/ && \
python3 generate_20_configs.py --output configs_rubric.json && \
python3 run_all_rubric_experiments.py --config configs_rubric.json --output logs_rubrica/ && \
python3 plot_rubric_results.py --input logs_rubrica/ --output plots_rubrica/ && \
python3 analyze_metrics.py --input logs_rubrica/ --output METRICS.md --json metrics.json
```

Tambien puedes usar el script interactivo con progreso visual:

```bash
./run_complete_pipeline.sh
```

**Tiempo estimado:** ~5-10 minutos.

### Salida generada

| Directorio/Archivo   | Contenido                                          |
|----------------------|-----------------------------------------------------|
| `logs_rubrica/`      | 20 CSVs con datos crudos (posicion, velocidad, tiempo) |
| `plots_rubrica/`     | 20 graficas PNG (posicion vs tiempo, 5 joints cada una) |
| `METRICS.md`         | Tabla comparativa con metricas cuantitativas         |
| `metrics.json`       | Datos estructurados en JSON                          |

---

## 4. Uso Detallado

Todos los comandos se ejecutan **dentro del contenedor** en `/ros2_ws/src/simulation_code`.

### Paso 1: Generar configuraciones

```bash
python3 generate_20_configs.py --output configs_rubric.json --print-table
```

Crea `configs_rubric.json` con las 20 configuraciones de ganancias escaladas por joint.

### Paso 2: Ejecutar un experimento individual (opcional)

```bash
python3 run_rubric_experiment.py \
  --config configs_rubric.json \
  --combo-id P_1_MuyBajo \
  --output logs_rubrica/
```

### Paso 3: Ejecutar los 20 experimentos

```bash
python3 run_all_rubric_experiments.py \
  --config configs_rubric.json \
  --output logs_rubrica/
```

**Opciones utiles:**

```bash
# Solo una familia (ej. P, PD, PI o PID)
python3 run_all_rubric_experiments.py --families P --output logs_rubrica/

# Multiples familias
python3 run_all_rubric_experiments.py --families P,PID --output logs_rubrica/

# Omitir experimentos ya ejecutados
python3 run_all_rubric_experiments.py --skip-existing --output logs_rubrica/

# Continuar desde un experimento especifico
python3 run_all_rubric_experiments.py --start-from PD_3_OverDamp --output logs_rubrica/
```

### Paso 4: Generar graficas

```bash
python3 plot_rubric_results.py --input logs_rubrica/ --output plots_rubrica/
```

### Paso 5: Calcular metricas

```bash
python3 analyze_metrics.py --input logs_rubrica/ --output METRICS.md --json metrics.json
```

Metricas calculadas por experimento:

| Metrica           | Descripcion                              |
|-------------------|------------------------------------------|
| **SSE**           | Error en estado estacionario (error final) |
| **Overshoot**     | Sobre-disparo maximo (%)                  |
| **Settling Time** | Tiempo para estabilizar (+-2°)            |
| **Rise Time**     | Tiempo para alcanzar 90% del cambio       |
| **RMSE**          | Error cuadratico medio                    |
| **Max Velocity**  | Velocidad maxima                          |

---

## 5. Las 20 Configuraciones PID

El sistema evalua **4 familias x 5 variantes = 20 configuraciones**.

### Filosofia de sintonizacion (Per-Joint Tuning)

Cada joint tiene factores de escala distintos segun sus caracteristicas fisicas:

| Joint          | Kp   | Ki   | Kd   | Justificacion                             |
|----------------|------|------|------|-------------------------------------------|
| shoulder_lift  | 2.0x | 2.5x | 1.5x | Soporta mas peso, necesita Ki alto        |
| shoulder_pan   | 1.5x | 1.5x | 1.2x | Alta inercia rotatoria                    |
| elbow_flex     | 1.5x | 2.0x | 1.3x | Inercia media, gravedad variable          |
| wrist_flex     | 1.0x | 1.0x | 1.0x | Referencia base                           |
| wrist_roll     | 0.6x | 0.5x | 0.8x | Menos inercia, mas rapido                 |

### Familia P (solo proporcional)

| ID             | Kp   | Comportamiento esperado     |
|----------------|------|-----------------------------|
| `P_1_MuyBajo`  | 300  | Muy lento                   |
| `P_2_Bajo`     | 600  | Lento pero suave            |
| `P_3_Medio`    | 1000 | Balance                     |
| `P_4_Alto`     | 1500 | Rapido, puede oscilar       |
| `P_5_MuyAlto`  | 2500 | Muy rapido, overshoot       |

### Familia PD (sin integral)

| ID               | Kp   | Kd  | Comportamiento esperado |
|------------------|------|-----|-------------------------|
| `PD_1_PocoDamp`  | 1000 | 20  | Poco damping            |
| `PD_2_Critico`   | 1000 | 50  | Damping critico         |
| `PD_3_OverDamp`  | 1000 | 100 | Sobreamortiguado        |
| `PD_4_Rigido`    | 1800 | 90  | Ambos altos             |
| `PD_5_ExcesivoKd`| 1200 | 180 | Kd muy alto             |

### Familia PI (sin derivativo)

| ID                  | Kp  | Ki  |
|---------------------|-----|-----|
| `PI_1_KiBajo`       | 800 | 40  |
| `PI_2_KiMedio`      | 800 | 80  |
| `PI_3_KiAlto`       | 800 | 160 |
| `PI_4_KpBajo_KiAlto`| 500 | 150 |
| `PI_5_Agresivo`     | 1200| 240 |

### Familia PID (completo)

| ID                 | Kp   | Ki  | Kd  |
|--------------------|------|-----|-----|
| `PID_1_Conservador`| 800  | 80  | 40  |
| `PID_2_Balanceado` | 1200 | 120 | 60  |
| `PID_3_Agresivo`   | 1800 | 180 | 90  |
| `PID_4_KiDominante`| 2000 | 250 | 50  |
| `PID_5_KdDominante`| 1000 | 100 | 150 |

---

## 6. Resultados y Analisis

### Fases del experimento

Cada experimento ejecuta 4 fases (8 segundos totales):

1. **Move Home -> 0°** (2s) - Interpolacion suave desde pose inicial
2. **Hold 0°** (2s) - Mantener posicion, evaluar estabilidad
3. **Move 0° -> Home** (2s) - Retorno a pose inicial
4. **Hold Home** (2s) - Mantener pose final

### Estructura del CSV

Cada CSV contiene las siguientes columnas:

```
time,
shoulder_pan, target_shoulder_pan, shoulder_pan_velocity,
shoulder_lift, target_shoulder_lift, shoulder_lift_velocity,
elbow_flex, target_elbow_flex, elbow_flex_velocity,
wrist_flex, target_wrist_flex, wrist_flex_velocity,
wrist_roll, target_wrist_roll, wrist_roll_velocity
```

> Las posiciones estan en grados (deg) y las velocidades en grados/s (deg/s).

### Acceder a los resultados desde el host

Los archivos generados dentro del contenedor estan disponibles gracias al volumen montado. Desde el host:

```bash
cd M4-RobotControl/simulation_code

ls -lh logs_rubrica/    # Ver CSVs
ls -lh plots_rubrica/   # Ver graficas

# Abrir una grafica
xdg-open plots_rubrica/plot_P_1_MuyBajo.png
```

---

## 7. Archivos del Sistema

### Scripts principales

| Archivo                          | Descripcion                                           |
|----------------------------------|-------------------------------------------------------|
| `generate_20_configs.py`         | Genera JSON con 20 configuraciones de ganancias       |
| `run_rubric_experiment.py`       | Ejecuta un experimento individual (sin GUI)           |
| `run_all_rubric_experiments.py`  | Ejecuta los 20 experimentos automaticamente           |
| `plot_rubric_results.py`         | Genera graficas PNG de posicion vs tiempo              |
| `analyze_metrics.py`             | Calcula metricas cuantitativas (SSE, overshoot, etc.) |
| `run_complete_pipeline.sh`       | Pipeline completo con progreso visual                 |
| `run_mujoco_simulation.py`       | Simulacion con visualizacion GUI                      |

### Modulos de soporte

| Archivo                    | Descripcion                                       |
|----------------------------|---------------------------------------------------|
| `so101_control.py`         | Clases PID, perturbaciones, control de joints     |
| `so101_mujoco_pid_utils.py`| Funciones core PID (move_to_pose, hold_position)  |
| `CSVDataLogger.py`         | Logger que guarda datos de simulacion a CSV        |

### Configuracion

| Archivo               | Descripcion                                    |
|-----------------------|------------------------------------------------|
| `configs_rubric.json` | 20 configuraciones de ganancias PID (generado) |
| `docker-compose.yml`  | Configuracion del contenedor Docker            |
| `Dockerfile`          | Imagen Docker con MuJoCo y dependencias        |

---

## 8. Troubleshooting

### El contenedor no puede acceder al display

```bash
# Desde el host
xhost +local:docker

# Dentro del contenedor
export DISPLAY=:0
```

### "Permission denied" al eliminar logs

Los archivos fueron creados dentro del contenedor. Eliminalos desde ahi:

```bash
docker exec -it so101_container bash
cd /ros2_ws/src/simulation_code
rm -rf logs_rubrica/ plots_rubrica/
```

### "Config file not found"

Genera el archivo de configuraciones primero:

```bash
python3 generate_20_configs.py --output configs_rubric.json
```

### "Model file not found"

Verifica que estes en el directorio correcto y que el modelo exista:

```bash
cd /ros2_ws/src/simulation_code
ls model/scene_urdf.xml
```

### "Multi-dimensional indexing" en graficas

Este error esta resuelto en la version actual. Si persiste, actualiza el repo:

```bash
git pull origin Dev-setup
```

### Ver logs en tiempo real durante experimentos

En otra terminal:

```bash
docker exec -it so101_container bash
cd /ros2_ws/src/simulation_code/logs_rubrica
watch -n 1 'ls -lht | head -5'
```

---

## Referencias

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [SO-101 Robot (Waveshare)](https://www.waveshare.com/wiki/SO-ARM101)
- [PID Controller (Wikipedia)](https://en.wikipedia.org/wiki/PID_controller)

---

**Dream Team - T3001B** | Fundamentos de Robotica | 2026
