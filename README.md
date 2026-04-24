# Proyecto Multi-Robot ROS 2

Este proyecto contiene la infraestructura necesaria para la simulación y control de un sistema multi-robot que incluye un Dron (UAV), un Rover y un brazo robótico ABB.

## 👥 Roles y Responsabilidades

### 👩‍💻 Rol 1:Visión y Percepción 
**Misión:** Darle inteligencia al dron.
- **Paquete principal:** `uav_vision`
- **Tareas:**
    - Configurar la cámara RGB-D del dron en Gazebo.
    - Diseñar y entrenar la red neuronal para filtrado de ruido y detección de objetivos.
    - Optimización de capas pooling y stride para rendimiento en tiempo real.
    - Programar el nodo de visión para traducir detección de píxeles a coordenadas 3D.

### 👩‍💻 Rol 2:ROS y Navegación 
**Misión:** Gazebo y gestionar la movilidad del Rover.
- **Paquete principal:** `rover_navigation` y `multi_robot_bringup`
- **Tareas:**
    - Configuración de archivos Launch con namespaces para coexistencia Dron/Rover.
    - Configuración del árbol de transformaciones (TF Tree).
    - Ajuste del NavStack, mapeo con LiDAR y recepción de Goal Poses desde el sistema de visión.

### 👩‍💻 Rol 3: Manipulación e Integración
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
├── uav_vision/               # IA y Visión (Rol 1)
├── rover_navigation/         # Navegación (Rol 2)
└── abb_bridge/               # Puente RobotStudio (Rol 3)
```

## 🛠️ Guía de Compilación

### 1. Preparar el entorno
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

## 🚀 Guía de Trabajo con Git (Colaboración)

Para que el equipo trabaje unido sin borrar el código de las demás, seguid estos pasos:

### 1. Obtener el proyecto (Clonar)
Si es la primera vez que vas a trabajar en una máquina:
```bash
git clone https://github.com/Pae14/Proyecto_multi.git
```

### 2. Crear tu propia "Rama" (Branch)
**IMPORTANTE:** Nunca trabajéis directamente en la rama `main`. Cread una rama para vuestro rol:
```bash
git checkout -b rama-vision      # Para Rol 1
git checkout -b rama-navegacion  # Para Rol 2
git checkout -b rama-abb         # Para Rol 3
```

### 3. Guardar tus cambios (Commit)
Cuando hayas hecho una mejora en el código:
```bash
git add .
git commit -m "Descripción breve de lo que has hecho (ej: Añadida red neuronal)"
```

### 4. Subir tus cambios a GitHub (Push)
La primera vez que subas una rama nueva:
```bash
git push -u origin tu-nombre-de-rama
```
Las siguientes veces basta con:
```bash
git push
```

### 5. Mantenerte actualizada
Antes de empezar a trabajar cada día, descarga los cambios de tus compañeras:
```bash
git pull origin main
```

### 6. Fusionar cambios (Merge)
Cuando tu parte funcione perfectamente, ve a la web de GitHub y pulsa en **"New Pull Request"** para pasar tus cambios de tu rama a la rama `main`.
