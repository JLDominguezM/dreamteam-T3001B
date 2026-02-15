# SO-101 Robot Control - M4

## Quick Start

Sistema para evaluar 20 configs PID en el robot SO-101.

```bash
# Entrar al container
docker exec -it so101_container bash
cd /ros2_ws/src/simulation_code

# Correr todo (toma ~10 min)
rm -rf logs_rubrica/ plots_rubrica/ && \
python3 generate_20_configs.py --output configs_rubric.json && \
python3 run_all_rubric_experiments.py --config configs_rubric.json --output logs_rubrica/ && \
python3 plot_rubric_results.py --input logs_rubrica/ --output plots_rubrica/ && \
python3 analyze_metrics.py --input logs_rubrica/ --output METRICS.md --json metrics.json && \
echo "Listo! Revisa logs_rubrica/, plots_rubrica/ y METRICS.md"
```

Genera: 
- 20 CSVs con datos
- 20 plots PNG
- Tabla de métricas (SSE, overshoot, settling time, etc.)
- JSON con resultados

---

## Tabla de Contenidos
- [Setup Docker](#setup-docker)
- [Sistema de Experimentos](#sistema-de-experimentos)
- [Uso](#uso)
- [Archivos](#archivos)
- [Resultados](#resultados)

---

## Setup Docker

### Build container

```bash
cd M4-RobotControl/simulation_code
docker compose up --build -d
docker ps
```

### Acceder al container

```bash
docker exec -it so101_container bash
```

### 3. Configurar display (para visualización GUI)

**Desde el host:**
```bash
xhost +local:docker
```

**Dentro del contenedor:**
```bash
export DISPLAY=:0
```

### 4. Ejecutar simulación con visualización

```bash
cd /ros2_ws/src/simulation_code
python3 run_mujoco_simulation.py
```

---

## Sistema Automatizado de Rúbrica

Este sistema ejecuta **20 experimentos PID** (4 familias × 5 variantes) para evaluar el comportamiento del robot SO-101 bajo diferentes configuraciones de control.

### Filosofía de Sintonización

**Per-Joint Tuning**: Cada joint tiene factores distintos según sus características:

| Joint          | Kp   | Ki   | Kd   | Por qué                                   |
|----------------|------|------|------|-------------------------------------------|
| shoulder_lift  | 2.0× | 2.5× | 1.5× | Soporta más peso, necesita Ki alto        |
| shoulder_pan   | 1.5× | 1.5× | 1.2× | Alta inercia rotatoria                    |
| elbow_flex     | 1.5× | 2.0× | 1.3× | Inercia media, gravedad variable          |
| wrist_flex     | 1.0× | 1.0× | 1.0× | Referencia base                           |
| wrist_roll     | 0.6× | 0.5× | 0.8× | Menos inercia, más rápido                 |

### Las 20 Combinaciones

#### P (sin Ki, sin Kd)
- `P_1_MuyBajo`: Kp=300 - Muy lento
- `P_2_Bajo`: Kp=600 - Lento pero suave
- `P_3_Medio`: Kp=1000 - Balance
- `P_4_Alto`: Kp=1500 - Rápido, puede oscilar
- `P_5_MuyAlto`: Kp=2500 - Muy rápido, overshoot

#### PD (sin Ki)
- `PD_1_PocoDamp`: Kp=1000, Kd=20 - Poco damping
- `PD_2_Critico`: Kp=1000, Kd=50 - Damping crítico
- `PD_3_OverDamp`: Kp=1000, Kd=100 - Sobreamortiguado
- `PD_4_Rigido`: Kp=1800, Kd=90 - Ambos altos
- `PD_5_ExcesivoKd`: Kp=1200, Kd=180 - Kd muy alto

#### PI (sin Kd)
- `PI_1_KiBajo`: Kp=800, Ki=40
- `PI_2_KiMedio`: Kp=800, Ki=80
- `PI_3_KiAlto`: Kp=800, Ki=160
- `PI_4_KpBajo_KiAlto`: Kp=500, Ki=150
- `PI_5_Agresivo`: Kp=1200, Ki=240

#### PID (completo)
- `PID_1_Conservador`: Kp=800, Ki=80, Kd=40
- `PID_2_Balanceado`: Kp=1200, Ki=120, Kd=60
- `PID_3_Agresivo`: Kp=1800, Ki=180, Kd=90
- `PID_4_KiDominante`: Kp=2000, Ki=250, Kd=50
- `PID_5_KdDominante`: Kp=1000, Ki=100, Kd=150

---

## Uso

### IMPORTANTE: Correr todo dentro del container

```bash
docker exec -it so101_container bash
cd /ros2_ws/src/simulation_code
```

---

### Paso 1: Generar configs

```bash
python3 generate_20_configs.py --output configs_rubric.json --print-table
```

Esto crea `configs_rubric.json` con las 20 configuraciones de ganancias escaladas por joint.

### Paso 2: Ejecutar UN Experimento (opcional)

```bash
python3 run_rubric_experiment.py \
  --config configs_rubric.json \
  --combo-id P_1_MuyBajo \
  --output logs_rubrica/
```

Genera: `logs_rubrica/log_P_1_MuyBajo.csv`

### Paso 3: Ejecutar TODOS los Experimentos (20)

```bash
python3 run_all_rubric_experiments.py \
  --config configs_rubric.json \
  --output logs_rubrica/
```

⏱️ **Tiempo estimado**: ~5-10 minutos (sin visualización)

**Opciones útiles:**

```bash
# Solo familia P (5 experimentos)
python3 run_all_rubric_experiments.py --families P --output logs_rubrica/

# Omitir experimentos ya ejecutados
python3 run_all_rubric_experiments.py --skip-existing --output logs_rubrica/

# Continuar desde un punto específico
python3 run_all_rubric_experiments.py --start-from PD_3_OverDamp --output logs_rubrica/

# Filtrar múltiples familias
python3 run_all_rubric_experiments.py --families P,PID --output logs_rubrica/
```

### Paso 4: Generar Gráficas

```bash
python3 plot_rubric_results.py --input logs_rubrica/ --output plots_rubrica/
```

Esto genera una gráfica PNG por cada experimento mostrando los 5 joints con sus trayectorias.

### Paso 5: Análisis Cuantitativo (Métricas)

```bash
python3 analyze_metrics.py --input logs_rubrica/ --output METRICS.md --json metrics.json
```

Calcula métricas para cada experimento:
- **SSE**: Steady-State Error (error final)
- **Overshoot**: Sobre-disparo máximo (%)
- **Settling Time**: Tiempo para estabilizar (±2°)
- **Rise Time**: Tiempo para alcanzar 90% del cambio
- **RMSE**: Error cuadrático medio
- **Max Velocity**: Velocidad máxima

**Opciones:**

```bash
# Solo markdown (sin JSON)
python3 analyze_metrics.py --input logs_rubrica/ --output METRICS.md

# Analizar un experimento específico
python3 analyze_metrics.py --input logs_rubrica/ --combo-id P_1_MuyBajo --output metrics_p1.md
```

---

### 🎯 Pipeline Completo (Un solo comando)

**Opción 1: Script automatizado (recomendado)**

```bash
cd /ros2_ws/src/simulation_code
./run_complete_pipeline.sh
```

Este script interactivo ejecuta todo el pipeline con progreso visual y validaciones.

**Opción 2: Comando manual**

```bash
# Limpiar, generar configs, ejecutar 20 experimentos, crear gráficas y analizar métricas
cd /ros2_ws/src/simulation_code && \
rm -rf logs_rubrica/ plots_rubrica/ && \
python3 generate_20_configs.py --output configs_rubric.json && \
python3 run_all_rubric_experiments.py --config configs_rubric.json --output logs_rubrica/ && \
python3 plot_rubric_results.py --input logs_rubrica/ --output plots_rubrica/ && \
python3 analyze_metrics.py --input logs_rubrica/ --output METRICS.md --json metrics.json && \
echo "Listo! Revisa logs_rubrica/, plots_rubrica/ y METRICS.md"
```

**Resultado final:**
- `logs_rubrica/` → 20 CSVs con datos crudos
- `plots_rubrica/` → 20 gráficas PNG (posición vs tiempo)
- `METRICS.md` → Tabla comparativa con métricas cuantitativas
- `metrics.json` → Datos estructurados en JSON

---

### 📂 Acceder a los Resultados desde el Host

Los archivos generados dentro del contenedor están montados en:

```bash
# Desde tu host (fuera del contenedor)
cd /home/dominguez/FundamentacionRobotics/dreamteam-T3001B/M4-RobotControl/simulation_code

# Ver logs CSV
ls -lh logs_rubrica/

# Ver gráficas PNG
ls -lh plots_rubrica/

# Abrir una gráfica
xdg-open plots_rubrica/plot_P_1_MuyBajo.png
```

---

## 📁 Archivos del Sistema

### Scripts Principales

| Archivo                          | Descripción                                               |
|----------------------------------|-----------------------------------------------------------|
| `generate_20_configs.py`         | Genera JSON con 20 configuraciones de ganancias          |
| `run_rubric_experiment.py`       | Ejecuta UN experimento individual (sin GUI)              |
| `run_all_rubric_experiments.py`  | Ejecuta los 20 experimentos automáticamente              |
| `plot_rubric_results.py`         | Genera gráficas PNG de posición vs tiempo                |
| `analyze_metrics.py`             | Calcula métricas cuantitativas (SSE, overshoot, etc.)    |
| `run_complete_pipeline.sh`       | **AUTOMÁTICO**: Pipeline completo con progreso visual    |
| `CSVDataLogger.py`               | Logger que guarda datos de simulación a CSV              |
| `so101_mujoco_pid_utils.py`      | Funciones core PID (move_to_pose, hold_position)         |
| `so101_control.py`               | Clases PID, perturbaciones, control de joints            |

### Archivos de Configuración

| Archivo                 | Descripción                                          |
|-------------------------|------------------------------------------------------|
| `configs_rubric.json`   | 20 configuraciones de ganancias PID                  |
| `docker-compose.yml`    | Configuración del contenedor Docker                  |
| `Dockerfile`            | Imagen Docker con MuJoCo y dependencias              |

### Datos Generados

| Archivo/Directorio | Contenido                                                    |
|--------------------|--------------------------------------------------------------|
| `logs_rubrica/`    | CSVs de los 20 experimentos (position, velocity, time)      |
| `plots_rubrica/`   | Gráficas PNG de posición vs tiempo (5 joints por gráfica)   |
| `METRICS.md`       | Tabla markdown con métricas cuantitativas comparativas       |
| `metrics.json`     | Datos estructurados JSON con todas las métricas              |
| `configs_rubric.json` | Configuración de las 20 combinaciones de ganancias PID   |

---

## 📊 Resultados y Análisis

### Estructura del CSV

Cada experimento genera un CSV con las siguientes columnas:

```
time, shoulder_pan, target_shoulder_pan, shoulder_pan_velocity,
      shoulder_lift, target_shoulder_lift, shoulder_lift_velocity,
      elbow_flex, target_elbow_flex, elbow_flex_velocity,
      wrist_flex, target_wrist_flex, wrist_flex_velocity,
      wrist_roll, target_wrist_roll, wrist_roll_velocity
```

**Nota**: Las posiciones y velocidades están en grados (deg) y grados/s (deg/s).

### Fases del Experimento

Cada experimento ejecuta 4 fases (8 segundos totales):

1. **Move Home → 0°** (2s): Interpolación suave desde pose inicial
2. **Hold 0°** (2s): Mantener posición, evaluar estabilidad
3. **Move 0° → Home** (2s): Retorno a pose inicial
4. **Hold Home** (2s): Mantener pose final

### Métricas de Evaluación

Para análisis cuantitativo, se pueden calcular:

- **Error en estado estacionario**: |posición final - target|
- **Overshoot**: Máximo sobre-disparo respecto al target
- **Settling time**: Tiempo para entrar en ±2° del target
- **Rise time**: Tiempo para alcanzar 90% del target
- **Oscilación**: Desviación estándar en fase de hold

---

## 🔧 Troubleshooting

### Error: "Permission denied" al eliminar logs

Los archivos fueron creados dentro del contenedor, necesitas eliminarlos desde ahí:

```bash
# DENTRO del contenedor
docker exec -it so101_container bash
cd /ros2_ws/src/simulation_code
rm -rf logs_rubrica/ plots_rubrica/
```

### Error: "Config file not found"
```bash
# Generar el archivo JSON primero (dentro del contenedor)
python3 generate_20_configs.py --output configs_rubric.json
```

### Error: "Model file not found"
```bash
# Verificar que estás en el directorio correcto (dentro del contenedor)
cd /ros2_ws/src/simulation_code
ls model/scene_urdf.xml  # Debe existir
```

### Error: "Multi-dimensional indexing" en gráficas

Este error ya está resuelto en la versión actual del script. Si persiste:
```bash
# Actualizar el script plot_rubric_results.py desde el repositorio
git pull origin Dev-setup
```

### Contenedor no puede acceder al display
```bash
# Desde el host
xhost +local:docker

# Dentro del contenedor
export DISPLAY=:0
```

### Ver logs en tiempo real durante experimentos
```bash
# Abrir otra terminal y seguir el último CSV
docker exec -it so101_container bash
cd /ros2_ws/src/simulation_code/logs_rubrica
watch -n 1 'ls -lht | head -5'
```

---

## Próximos Pasos (Opcional)

### Análisis Cuantitativo

Puedes crear un script adicional para calcular métricas:

- **Error en estado estacionario (SSE)**: |posición final - target|
- **Overshoot**: Máximo sobre-disparo (%)
- **Settling time**: Tiempo para entrar en ±2° del target
- **Rise time**: Tiempo para alcanzar 90% del target
- **RMSE**: Error cuadrático medio

### Tabla Comparativa

Genera una tabla markdown con las métricas de los 20 experimentos:

```python
# Ejemplo de estructura
python3 analyze_metrics.py --input logs_rubrica/ --output METRICS.md
```

### Documento de Filosofía

Documenta la justificación física de los factores de escala:

```markdown
# PHILOSOPHY.md
- shoulder_lift: 2.5× Ki por efecto gravitatorio
- wrist_roll: 0.6× Kp por baja inercia
- etc.
```

---

## Referencias

- **MuJoCo**: https://mujoco.readthedocs.io/
- **SO-101 Robot**: https://www.waveshare.com/wiki/SO-ARM101
- **Control PID**: https://en.wikipedia.org/wiki/PID_controller
- **Ziegler-Nichols**: Método clásico de sintonización PID

---

## Autores

- Dream Team - T3001B
- Fundamentos de Robótica
- 2026

