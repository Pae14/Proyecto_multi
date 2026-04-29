# Proyecto Multi-Robot ROS 2

Este proyecto contiene la infraestructura necesaria para la simulación y control de un sistema multi-robot que incluye un Dron (UAV), un Rover y un brazo robótico ABB.

## 👥 Roles y Responsabilidades

### 👩‍💻 Rol 1: Visión y Percepción
**Misión:** Darle inteligencia al dron.
- **Paquete principal:** `uav_vision`
- **Tareas:**
    - Configurar la cámara RGB-D del dron en Gazebo.
    - Diseñar y entrenar la red neuronal para filtrado de ruido y detección de objetivos.
    - Programar el nodo de visión para traducir detección de píxeles a coordenadas 3D.

### 👩‍💻 Rol 2: ROS y Navegación 
**Misión:** Control de Gazebo y gestionar la movilidad del Rover.
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

## 🚀 Guía de Ejecución

Sigue este orden para poner en marcha el sistema:

### 1. Lanzar la simulación (Gazebo)
Este comando abre Gazebo con el mundo configurado, el Rover y el Dron:
```bash
ros2 launch multi_robot_bringup gazebo.launch.py
```

### 2. Lanzar el sistema de visión
En una **nueva terminal**, ejecuta el nodo que procesa la cámara del dron:
```bash
ros2 launch uav_vision vision.launch.py
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
Cuando tu parte funcione perfectamente, puedes fusionar tus cambios en la rama `main`.

#### Opción A: Por GitHub (Recomendado)
Ve a la web de GitHub y pulsa en **"New Pull Request"** para pasar los cambios de tu rama a `main`.

#### Opción B: Por comandos (CLI)
```bash
# 1. Asegúrate de estar en la rama main
git checkout main

# 2. Actualiza tu rama main local con lo último del servidor
git pull origin main

# 3. Fusiona tu rama de trabajo (ej: rama-vision) en main
git merge rama-vision
```bash
# 4. Sube la rama main actualizada a GitHub
git push origin main
```

### 7. Compilar tras los cambios
Cada vez que descargues cambios nuevos (`pull`) o fusiones una rama (`merge`), es fundamental volver a compilar el proyecto:
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select multi_robot_bringup uav_vision rover_navigation abb_bridge
source install/setup.bash
```
