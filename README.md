# Proyecto Multi-Robot ROS 2

Este proyecto contiene la infraestructura necesaria para la simulación y control de un sistema multi-robot que incluye un Dron (UAV), un Rover y un brazo robótico ABB.

## 👥 Roles y Responsabilidades

### 👩‍💻 Rol 1: Especialista en Visión y Percepción (El "Ojo")
**Misión:** Darle inteligencia al dron.
- **Paquete principal:** `uav_vision`
- **Tareas:**
    - Configurar la cámara RGB-D del dron en Gazebo.
    - Diseñar y entrenar la red neuronal (CNN) con kernels grandes para filtrado de ruido y detección de objetivos.
    - Optimización de capas pooling y stride para rendimiento en tiempo real.
    - Programar el nodo de visión para traducir detección de píxeles a coordenadas 3D.

### 👩‍💻 Rol 2: Especialista en ROS y Navegación (El "Cerebro Móvil")
**Misión:** Dominar Gazebo y gestionar la movilidad del Rover.
- **Paquete principal:** `rover_navigation` y `multi_robot_bringup`
- **Tareas:**
    - Configuración de archivos Launch con namespaces para coexistencia Dron/Rover.
    - Configuración del árbol de transformaciones (TF Tree).
    - Ajuste del NavStack, mapeo con LiDAR y recepción de Goal Poses desde el sistema de visión.

### 👩‍💻 Rol 3: Especialista en Manipulación e Integración (El "Músculo")
**Misión:** Controlar RobotStudio y conectar Windows con Ubuntu.
- **Paquete principal:** `abb_bridge`
- **Tareas:**
    - Diseño de estación en RobotStudio y programación RAPID para trayectorias de recogida.
    - Configuración de comunicación red/firewall entre Windows (RobotStudio) y Ubuntu (ROS 2).
    - Creación del puente de comunicación (Socket TCP/Driver) para coordinar la llegada del Rover con la acción del brazo.

## 📁 Estructura del Proyecto

```text
src/proyecto_multi/
├── multi_robot_bringup/      # Lanzamientos y mundos Gazebo (Rol 2)
│   ├── launch/               # Archivos .launch.py
│   ├── world/                # Mundos (.sdf, .world)
│   ├── config/               # Configuraciones (.yaml, .rviz)
│   ├── models/               # Modelos 3D
│   └── maps/                 # Mapas generados
├── uav_vision/               # IA y Visión (Rol 1)
│   ├── uav_vision/           # Código fuente Python
│   └── launch/               # Lanzamientos de visión
├── rover_navigation/         # Navegación (Rol 2)
│   ├── src/                  # Código fuente C++
│   ├── launch/               # Lanzamientos de navegación
│   └── config/               # Parámetros de Nav2/SLAM
└── abb_bridge/               # Puente RobotStudio (Rol 3)
    ├── abb_bridge/           # Código fuente Python
    └── launch/               # Lanzamientos del puente
```

## 🛠️ Guía de Compilación

### 1. Preparar el entorno
Asegúrate de estar en la raíz de tu espacio de trabajo (`ros2_ws`) y tener ROS 2 Jazzy activo:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
```

### 2. Compilar
```bash
colcon build --symlink-install --packages-select multi_robot_bringup uav_vision rover_navigation abb_bridge
```

### 3. Cargar el Workspace
```bash
source install/setup.bash
```
