# SO-101 Robot Control - M4

Sistema avanzado de control PID para el brazo robótico SO-101 con evaluación sistemática de 20 configuraciones.

## Descripción General

Este proyecto implementa un sistema completo de evaluación de controladores PID para el robot SO-101 utilizando:
- **Dos entornos de simulación**: MuJoCo (pruebas rápidas) + Gazebo (validación realista)
- **20 configuraciones PID** (4 familias: P, PD, PI, PID × 5 variantes cada una)
- **Sintonización basada en física** con factores de escala derivados de la dinámica del robot
- **Pipeline automatizado**: Generación de configs → Simulación → Análisis → Visualización
- **Métricas cuantitativas**: SSE, overshoot, settling time, rise time, RMSE, velocidad máxima

## Tabla de Contenidos

- [1. Setup Docker](#1-setup-docker)
- [2. Inicio Rápido](#2-inicio-rápido)
- [3. Entornos de Simulación](#3-entornos-de-simulación)
  - [3.1 MuJoCo (Pruebas Rápidas)](#31-mujoco-pruebas-rápidas)
  - [3.2 Gazebo (Validación Realista)](#32-gazebo-validación-realista)
- [4. Pipeline de Experimentos PID](#4-pipeline-de-experimentos-pid)
- [5. Uso Detallado](#5-uso-detallado)
- [6. Las 20 Configuraciones PID](#6-las-20-configuraciones-pid)
- [7. Resultados y Análisis](#7-resultados-y-análisis)
- [8. Archivos del Sistema](#8-archivos-del-sistema)
- [9. Troubleshooting](#9-troubleshooting)
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

## 2. Inicio Rápido

### Pipeline Completo de PID (Recomendado)

Dentro del contenedor, ejecuta todo en un comando:

```bash
cd /ros2_ws/src/simulation_code

# Pipeline automatizado completo
./run_complete_pipeline.sh
```

**¿Qué hace este comando?**
1. Genera 20 configuraciones PID
2. Ejecuta 20 experimentos en MuJoCo (~5-10 min)
3. Crea gráficas de posición (PNG)
4. Calcula métricas de rendimiento
5. Genera tabla comparativa `METRICS.md`

**Archivos generados:**
- `logs_rubrica/` - 20 archivos CSV con datos crudos
- `plots_rubrica/` - 20 gráficas PNG (posición vs tiempo)
- `METRICS.md` - Tabla de métricas cuantitativas
- `metrics.json` - Resultados en formato JSON estructurado

---

## 3. Entornos de Simulación

Este proyecto utiliza **dos simuladores complementarios**:

### 3.1 MuJoCo (Pruebas Rápidas)

MuJoCo se usa para **experimentación rápida** (20 configs en ~10 min).

#### Habilitar display (si usas GUI)

**Desde el host:**
```bash
xhost +local:docker
```

**Dentro del contenedor:**
```bash
export DISPLAY=:0
```

#### Ejecutar simulación MuJoCo

```bash
cd /ros2_ws/src/simulation_code
python3 run_mujoco_simulation.py
```

> **Nota:** Los experimentos corren sin GUI (headless). La GUI solo es para debugging visual.

---

### 3.2 Gazebo (Validación Realista)

Gazebo proporciona **física realista** para validación final de las mejores configuraciones PID.

#### Paso 1: Lanzar Gazebo

**Terminal 1 - Iniciar servidor de Gazebo:**
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

Espera hasta ver: `[gazebo-1] [INFO] [gazebo]: Gazebo started`

#### Paso 2: Cargar el Robot

**Terminal 2 - Spawner del URDF SO-101:**
```bash
cd /ros2_ws/src/simulation_code
ros2 run gazebo_ros spawn_entity.py \
  -entity so101_arm_real \
  -file model/so101_perfect.urdf
```

Deberías ver el robot aparecer en la vista 3D de Gazebo.

#### Paso 3: Reproducir Trayectoria

**Terminal 3 - Ejecutar trayectoria PID:**
```bash
cd /ros2_ws/src/simulation_code
python3 play_gazebo_trajectory.py
```

**Opcional: Reproducir resultado de experimento específico**
```bash
# Reproducir la mejor configuración de los experimentos de MuJoCo
python3 play_gazebo_trajectory.py --csv logs_rubrica/log_PID_2_Balanceado.csv
```

Esto envía comandos de posición mediante el plugin `gazebo_ros_joint_pose_trajectory`.

> **Consejo:** Usa Gazebo después de identificar las mejores configs en MuJoCo. Gazebo es más lento pero más realista.

---

## 4. PID Experiments Pipeline

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

## 5. Uso Detallado

Todos los comandos se ejecutan **dentro del contenedor** en `/ros2_ws/src/simulation_code`.

### Paso 1: Generar Configuraciones

```bash
python3 generate_20_configs.py --output configs_rubric.json --print-table
```

Crea `configs_rubric.json` con 20 configuraciones PID (ganancias escaladas por joint).

### Paso 2: Ejecutar Experimento Individual (Opcional)

```bash
python3 run_rubric_experiment.py \
  --config configs_rubric.json \
  --combo-id P_1_MuyBajo \
  --output logs_rubrica/
```

### Paso 3: Ejecutar los 20 Experimentos

```bash
python3 run_all_rubric_experiments.py \
  --config configs_rubric.json \
  --output logs_rubrica/
```

**Opciones Útiles:**

```bash
# Solo una familia (P, PD, PI o PID)
python3 run_all_rubric_experiments.py --families P --output logs_rubrica/

# Múltiples familias
python3 run_all_rubric_experiments.py --families P,PID --output logs_rubrica/

# Omitir experimentos ya completados
python3 run_all_rubric_experiments.py --skip-existing --output logs_rubrica/

# Continuar desde experimento específico
python3 run_all_rubric_experiments.py --start-from PD_3_OverDamp --output logs_rubrica/
```

### Paso 4: Generar Gráficas

```bash
python3 plot_rubric_results.py --input logs_rubrica/ --output plots_rubrica/
```

Crea gráficas PNG mostrando posición vs tiempo para los 5 joints con fondos de fases.

### Paso 5: Calcular Métricas

```bash
python3 analyze_metrics.py --input logs_rubrica/ --output METRICS.md --json metrics.json
```

**Métricas Calculadas por Experimento:**

| Métrica           | Descripción                                   | Valor Bueno   |
|-------------------|-----------------------------------------------|---------------|
| **SSE**           | Error en estado estacionario (error final)    | < 0.5°        |
| **Overshoot**     | Porcentaje de sobrepaso máximo                | < 5%          |
| **Settling Time** | Tiempo para estabilizar dentro de ±2°         | < 0.8s        |
| **Rise Time**     | Tiempo para alcanzar 90% del cambio objetivo  | < 0.6s        |
| **RMSE**          | Error cuadrático medio                        | < 1.0°        |
| **Max Velocity**  | Velocidad máxima del joint                    | < 50 deg/s    |

---

## 6. Las 20 Configuraciones PID

El sistema evalúa **4 familias × 5 variantes = 20 configuraciones**.

### Filosofía de Sintonización (Escalamiento por Joint Basado en Física)

Cada joint usa factores de escala diferentes basados en sus características físicas:

| Joint          | Kp   | Ki   | Kd   | Justificación                                |
|----------------|------|------|------|----------------------------------------------|
| shoulder_lift  | 2.0× | 2.5× | 1.5× | Soporta peso del brazo, necesita Ki alto     |
| shoulder_pan   | 1.5× | 1.5× | 1.2× | Alta inercia rotacional                      |
| elbow_flex     | 1.5× | 2.0× | 1.3× | Inercia media, torque gravitacional variable |
| wrist_flex     | 1.0× | 1.0× | 1.0× | Referencia base                              |
| wrist_roll     | 0.6× | 0.5× | 0.8× | Baja inercia, dinámica más rápida            |

**Principio de Ingeniería:** Las ganancias no son arbitrarias - consideran masa de eslabones, momentos de inercia y efectos gravitacionales. Esto es superior a usar ganancias uniformes en todos los joints.

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

## 7. Results and Analysis

### Experiment Phases

Each experiment runs 4 phases (8 seconds total):

1. **Move Home → 0°** (2s) - Smooth interpolation from home pose
2. **Hold 0°** (2s) - Position holding, stability evaluation
3. **Move 0° → Home** (2s) - Return to home pose
4. **Hold Home** (2s) - Final position holding

This sequence tests both **tracking performance** (phases 1,3) and **disturbance rejection** (phases 2,4).

---

## 7. Resultados y Análisis

### Fases del Experimento

Cada experimento ejecuta 4 fases (8 segundos totales):

1. **Move Home → 0°** (2s) - Interpolación suave desde pose inicial
2. **Hold 0°** (2s) - Mantener posición, evaluación de estabilidad
3. **Move 0° → Home** (2s) - Retorno a pose inicial
4. **Hold Home** (2s) - Mantener pose final

Esta secuencia prueba tanto **rendimiento de seguimiento** (fases 1,3) como **rechazo de perturbaciones** (fases 2,4).

### Estructura de Datos CSV

Cada CSV contiene:

```
time,
shoulder_pan, target_shoulder_pan, shoulder_pan_velocity,
shoulder_lift, target_shoulder_lift, shoulder_lift_velocity,
elbow_flex, target_elbow_flex, elbow_flex_velocity,
wrist_flex, target_wrist_flex, wrist_flex_velocity,
wrist_roll, target_wrist_roll, wrist_roll_velocity
```

**Unidades:** Posiciones en grados (deg), velocidades en grados/segundo (deg/s).

### Acceder a Resultados desde el Host

Los archivos generados dentro del contenedor están disponibles mediante volumen montado:

```bash
cd M4-RobotControl/simulation_code

ls -lh logs_rubrica/    # Ver archivos CSV
ls -lh plots_rubrica/   # Ver gráficas PNG
cat METRICS.md          # Ver tabla de métricas

# Abrir gráfica
xdg-open plots_rubrica/plot_P_1_MuyBajo.png
```

---

## 8. Archivos del Sistema

### Scripts Principales

| Archivo                          | Descripción                                            |
|----------------------------------|--------------------------------------------------------|
| `generate_20_configs.py`         | Genera JSON con 20 configuraciones PID                 |
| `run_rubric_experiment.py`       | Ejecuta experimento individual (sin GUI)               |
| `run_all_rubric_experiments.py`  | Ejecución por lotes de 20 experimentos                 |
| `plot_rubric_results.py`         | Crea gráficas PNG (posición vs tiempo)                 |
| `analyze_metrics.py`             | Calcula métricas cuantitativas (SSE, overshoot,...)    |
| `run_complete_pipeline.sh`       | Pipeline automatizado completo con seguimiento         |
| `run_mujoco_simulation.py`       | Simulación MuJoCo con visualización GUI                |
| `play_gazebo_trajectory.py`      | Reproduce trayectoria en Gazebo (validación realista)  |

### Módulos de Soporte

| Archivo                     | Descripción                                        |
|-----------------------------|----------------------------------------------------|
| `so101_control.py`          | Clases PID, perturbaciones, lógica de control      |
| `so101_mujoco_pid_utils.py` | Funciones PID core (move_to_pose, hold_position)   |
| `CSVDataLogger.py`          | Logger CSV para datos de simulación                |

### Configuración y Datos

| Archivo/Directorio    | Descripción                                         |
|-----------------------|-----------------------------------------------------|
| `configs_rubric.json` | 20 configuraciones de ganancias PID (auto-generado) |
| `logs_rubrica/`       | Archivos CSV de datos experimentales               |
| `plots_rubrica/`      | Gráficas PNG (seguimiento de posición)             |
| `METRICS.md`          | Tabla comparativa de métricas de rendimiento       |
| `metrics.json`        | Datos de métricas en formato JSON estructurado     |
| `docker-compose.yml`  | Configuración del contenedor Docker                |
| `Dockerfile`          | Imagen Docker con MuJoCo + ROS 2 + dependencias    |

### Archivos de Modelos

| Archivo                    | Descripción                                  |
|----------------------------|----------------------------------------------|
| `model/scene_urdf.xml`     | Escena MuJoCo con robot SO-101               |
| `model/so101_perfect.urdf` | URDF perfecto para Gazebo (física refinada)  |
| `model/so101.urdf`         | URDF estándar para Gazebo                    |
| `model/assets/*.stl`       | Archivos mesh 3D para visualización          |

---

## 9. Mejores Prácticas

### Fortalezas de la Implementación Actual

✅ **Arquitectura modular** - Separación clara de responsabilidades  
✅ **Sintonización basada en física** - No selección aleatoria de ganancias  
✅ **Pipeline automatizado** - Ejecución con un comando  
✅ **Simulación dual** - MuJoCo (rápido) + Gazebo (realista)  
✅ **Evaluación cuantitativa** - Métricas reproducibles  
✅ **Control de versiones** - Repositorio Git con estructura apropiada

### Mejoras Recomendadas

Para un sistema más robusto en producción, considera agregar:

1. **Pruebas Unitarias** - Usa `pytest` para testing automatizado
   ```bash
   pytest tests/ --cov=src/
   ```

2. **Pipeline CI/CD** - GitHub Actions para validación automatizada
   ```yaml
   # .github/workflows/test.yml
   - run: docker compose up -d
   - run: docker exec so101_container pytest tests/
   ```

3. **Type Hints & Linting** - Forzar calidad de código
   ```bash
   mypy src/
   flake8 src/
   black src/
   ```

4. **Framework de Logging** - Reemplazar `print()` con logging apropiado
   ```python
   import logging
   logger = logging.getLogger(__name__)
   logger.info("Experimento iniciado")
   ```

5. **Gestión de Configuración** - Usar YAML en vez de valores hard-coded
   ```yaml
   # config.yaml
   simulation:
     dt: 0.01
     duration: 8.0
   ```

6. **Análisis Estadístico** - Ejecutar N repeticiones y reportar media ± desv. estándar
   ```python
   for seed in range(5):
       run_experiment(config, seed=seed)
   # Luego: t-test, ANOVA para significancia
   ```

7. **Experiment Tracking** - Usar MLflow o Weights & Biases
   ```python
   import mlflow
   mlflow.log_params({"kp": 1000, "ki": 100})
   mlflow.log_metrics({"sse": 0.3, "overshoot": 5.2})
   ```


---

## 10. Troubleshooting

### El Contenedor No Puede Acceder al Display

```bash
# Desde el host
xhost +local:docker

# Dentro del contenedor
export DISPLAY=:0
```

### "Permission Denied" al Eliminar Logs

Los archivos fueron creados dentro del contenedor. Elimínalos desde ahí:

```bash
docker exec -it so101_container bash
cd /ros2_ws/src/simulation_code
rm -rf logs_rubrica/ plots_rubrica/
```

### "Config File Not Found"

Genera el archivo de configuración primero:

```bash
python3 generate_20_configs.py --output configs_rubric.json
```

### "Model File Not Found"

Verifica que estés en el directorio correcto y el modelo exista:

```bash
cd /ros2_ws/src/simulation_code
ls model/scene_urdf.xml
ls model/so101_perfect.urdf
```

### Gazebo No Se Inicia

Verifica que el entorno ROS 2 esté cargado:

```bash
source /opt/ros/humble/setup.bash
ros2 launch gazebo_ros gazebo.launch.py
```

### El Robot No Aparece en Gazebo

Asegúrate de que Gazebo esté completamente cargado antes de hacer spawn:

```bash
# Espera ver este mensaje:
[gazebo-1] [INFO] [gazebo]: Gazebo started

# Luego haz spawn del robot
ros2 run gazebo_ros spawn_entity.py -entity so101_arm_real -file model/so101_perfect.urdf
```

### El Script play_gazebo No Tiene Efecto

Verifica que el robot fue spawneado y los nombres de joints coinciden:

```bash
# Lista topics para verificar que el robot está cargado
ros2 topic list | grep joint

# Deberías ver:
# /so101_arm_real/joint_trajectory
```

### Error "Multi-dimensional Indexing" en Gráficas

Esto está arreglado en la versión actual. Si persiste:

```bash
git pull origin Dev-setup
```

### Monitorear Experimentos en Tiempo Real

En otra terminal:

```bash
docker exec -it so101_container bash
cd /ros2_ws/src/simulation_code/logs_rubrica
watch -n 1 'ls -lht | head -5'
```

---

## Referencias

### Documentación
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Gazebo Classic Documentation](http://classic.gazebosim.org/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [SO-101 Robot Specifications (Waveshare)](https://www.waveshare.com/wiki/SO-ARM101)

### Teoría de Control
- Ogata, K. (2010). *Modern Control Engineering*. Prentice Hall.
- Åström, K. J., & Murray, R. M. (2021). *Feedback Systems: An Introduction for Scientists and Engineers*.
- [Brian Douglas - PID Control (YouTube)](https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y)
- [MATLAB PID Tuning Guide](https://www.mathworks.com/help/control/ug/designing-pid-controllers.html)

### Robótica
- Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control*. Pearson.
- Siciliano, B., et al. (2010). *Robotics: Modelling, Planning and Control*. Springer.
- [ROS Control Best Practices](http://wiki.ros.org/ros_control)

### Ingeniería de Software
- Martin, R. C. (2008). *Clean Code: A Handbook of Agile Software Craftsmanship*. Prentice Hall.
- [Python Testing with pytest](https://docs.pytest.org/)
- [Docker Best Practices](https://docs.docker.com/develop/dev-best-practices/)

---

## Estructura del Proyecto

```
simulation_code/
├── generate_20_configs.py        # Generación de configuraciones
├── run_rubric_experiment.py      # Experimento individual
├── run_all_rubric_experiments.py # Ejecución por lotes
├── plot_rubric_results.py        # Visualización
├── analyze_metrics.py            # Cálculo de métricas
├── run_complete_pipeline.sh      # Pipeline automatizado
├── play_gazebo_trajectory.py     # Validación en Gazebo
├── so101_control.py              # Clases controlador PID
├── so101_mujoco_pid_utils.py     # Utilidades PID core
├── CSVDataLogger.py              # Logging de datos
├── ReadMe.md                     # Este archivo
├── BEST_PRACTICES_REVIEW.md      # Análisis de calidad de código
├── model/
│   ├── scene_urdf.xml            # Escena MuJoCo
│   ├── so101_perfect.urdf        # URDF Gazebo (refinado)
│   └── assets/*.stl              # Meshes 3D
├── configs_rubric.json           # Configs PID generadas
├── logs_rubrica/                 # Datos CSV experimentales
├── plots_rubrica/                # Gráficas PNG
├── METRICS.md                    # Tabla de métricas
└── metrics.json                  # Resultados estructurados
```

---

**Dream Team - T3001B** | Fundamentos de Robótica | Tecnológico de Monterrey | 2026

**Licencia:** MIT (ver archivo LICENSE)

**Contacto:** Para preguntas o colaboración: [A01285873@tec.mx]
- [Docker Best Practices](https://docs.docker.com/develop/dev-best-practices/)

---

## Project Structure Summary

```
simulation_code/
├── generate_20_configs.py        # Config generation
├── run_rubric_experiment.py      # Single experiment
├── run_all_rubric_experiments.py # Batch execution
├── plot_rubric_results.py        # Visualization
├── analyze_metrics.py            # Metrics calculation
├── run_complete_pipeline.sh      # Automated pipeline
├── play_gazebo_trajectory.py     # Gazebo validation
├── so101_control.py              # PID controller classes
├── so101_mujoco_pid_utils.py     # Core PID utilities
├── CSVDataLogger.py              # Data logging
├── ReadMe.md                     # This file
├── model/
│   ├── scene_urdf.xml            # MuJoCo scene
│   ├── so101_perfect.urdf        # Gazebo URDF (refined)
│   └── assets/*.stl              # 3D meshes
├── configs_rubric.json           # Generated PID configs
├── logs_rubrica/                 # Experiment CSV data
├── plots_rubrica/                # PNG plots
├── METRICS.md                    # Metrics table
└── metrics.json                  # Structured results
```

---

**Dream Team - T3001B** | Fundamentals of Robotics | Tecnológico de Monterrey | 2026

**Contributors:** Jose Luis Dominguez Martinez, [Team Members]

**License:** MIT (see LICENSE file)

**Contact:** For questions or collaboration: [A01285873@tec.mx]
