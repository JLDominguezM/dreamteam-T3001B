# Lab 2 - Kinematics with UFACTORY Lite6

## Descripción
Este laboratorio implementa control cinemático para el robot UFACTORY Lite6 utilizando ROS2.

## Estructura del Proyecto

```
Lab2/
├── src/
│   ├── lite6_drawing/       # Paquete principal para control del robot
│   ├── pymoveit2/          # Biblioteca para interfaz con MoveIt2 (submódulo)
│   └── xarm_ros2/          # Paquete ROS2 para control de xArm (submódulo)
└── README.md
```

## Dependencias

Este proyecto utiliza los siguientes repositorios externos como submódulos de Git:

- **pymoveit2**: [AndrejOrsula/pymoveit2](https://github.com/AndrejOrsula/pymoveit2)
  - Commit: `900c137499ec70d5a5cb216e24d88e91c7a508fc`
  - Interfaz Python para MoveIt2

- **xarm_ros2**: [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2)
  - Commit: `5bb832f72ca665f1236a9d8ed1c3a82f308db489`
  - Soporte ROS2 oficial para robots xArm/Lite6

## Configuración Inicial

Para clonar este proyecto con todos sus submódulos:

```bash
git clone --recursive <repository-url>
```

Si ya clonaste el repositorio sin submódulos:

```bash
git submodule update --init --recursive
```

## Instalación

1. Asegúrate de tener ROS2 instalado
2. Instala las dependencias necesarias:
   ```bash
   cd src
   rosdep install --from-paths . --ignore-src -r -y
   ```
3. Compila el workspace:
   ```bash
   cd ..
   colcon build
   ```

## Uso

[TODO: Agregar instrucciones de uso específicas del laboratorio]

## Autores

Dream Team - T3001B
