# Proyecto Multi-Robot ROS 2

Este proyecto contiene la infraestructura necesaria para la simulación y control de un sistema multi-robot que incluye un Dron (UAV), un Rover y un brazo robótico ABB.

## Descripción de los Paquetes

1. **multi_robot_bringup (C++)**: Paquete central que gestiona los archivos de lanzamiento (`launch`), los mundos de simulación (`world`), configuraciones generales, modelos 3D y mapas.
2. **uav_vision (Python)**: Enfocado en el procesamiento de imágenes, IA y visión artificial para el dron.
3. **rover_navigation (C++)**: Implementa la lógica de navegación, localización y mapeo (SLAM) para el robot terrestre.
4. **abb_bridge (Python)**: Actúa como puente de comunicación entre ROS 2 y RobotStudio para el brazo robótico.

## Estructura del Proyecto

```text
src/proyecto_multi/
├── multi_robot_bringup/      # Lanzamientos y mundos Gazebo
│   ├── launch/               # Archivos .launch.py
│   ├── world/                # Mundos (.sdf, .world)
│   ├── config/               # Configuraciones (.yaml, .rviz)
│   ├── models/               # Modelos 3D
│   └── maps/                 # Mapas generados
├── uav_vision/               # IA y Visión (Python)
│   ├── uav_vision/           # Código fuente Python
│   └── launch/               # Lanzamientos de visión
├── rover_navigation/         # Navegación (C++)
│   ├── src/                  # Código fuente C++
│   ├── launch/               # Lanzamientos de navegación
│   └── config/               # Parámetros de Nav2/SLAM
└── abb_bridge/               # Puente RobotStudio (Python)
    ├── abb_bridge/           # Código fuente Python
    └── launch/               # Lanzamientos del puente
```

## Guía de Compilación

### 1. Preparar el entorno
Asegúrate de estar en la raíz de tu espacio de trabajo (`ros2_ws`) y tener ROS 2 Jazzy activo:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
```

### 2. Compilar
Para compilar específicamente los paquetes de este proyecto:

```bash
colcon build --symlink-install --packages-select multi_robot_bringup uav_vision rover_navigation abb_bridge
```

### 3. Cargar el Workspace
Una vez terminada la compilación, activa los paquetes en tu terminal actual:

```bash
source install/setup.bash
```
