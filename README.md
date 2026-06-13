# TFG — Evaluación y Optimización de la Navegación SLAM en Robots Móviles

**Autor:** Jesús Ruslan Esquinas Herrera  
**Tutor:** Fernando Díaz del Río  
**Grado en Ingeniería Informática - Ingeniería de Computadores**  
**Universidad de Sevilla**

---

## Estructura del repositorio

```
TFG-jesesqher/
├── Dockerfile                                  # Imagen Docker con ROS2 Humble
├── latex/                                      # Memoria del TFG en LaTeX
└── ros2/
    ├── Analysis/                               # Proyecto Python de análisis de datos
    │   ├── results/
    │   │   ├── static/                         # CSVs del nodo de métricas estático
    │   │   └── dynamic/                        # CSVs del nodo de métricas dinámico
    │   ├── screenshots/                        # Capturas de RViz y Gazebo
    │   ├── plots/
    │   │   ├── static/                         # Gráficas del script de análisis estático
    │   │   └── dynamic/                        # Gráficas del script de análisis dinámico
    │   ├── static.py                           # Análisis de métricas estáticas
    │   ├── dynamic.py                          # Análisis de métricas dinámicas
    │   └── requirements.txt
    ├── Simulated/
    │   ├── tfg_ws/                             # Workspace ROS2
    │   |   └── src/
    │   |       ├── turtlebot4_cpp_tutorials/   # Nodo de control básico
    │   |       └── turtlebot4_cpp_slam_nav/    # Nodos de navegación y métricas
    │   |           ├── config/                 # Yamls de Nav2 por algoritmo
    │   |           ├── launch/                 # Launch files
    │   |           └── src/                    # Nodos C++
    |   └── Results/                            # CSVs obtenidos de los nodos de medición y capturas de pantalla
    └── Tutorial/                               # Tutoriales iniciales de ROS2
```

---

## Requisitos previos

### Opción A — Ubuntu 22.04 (nativo)

- Ubuntu 22.04 LTS
- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Paquetes del TurtleBot4:

```bash
sudo apt install \
  ros-humble-turtlebot4-simulator \
  ros-humble-turtlebot4-navigation \
  ros-humble-turtlebot4-viz \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox
```

### Opción B — Docker (cualquier distribución Linux con Docker instalado)

- Docker
- GPU NVIDIA + drivers instalados (recomendado para rendimiento óptimo)
- `nvidia-container-toolkit` (para acceso a GPU desde Docker)

---

## Instalación del entorno con Docker

### 1. Construir la imagen

```bash
cd ~/TFG-jesesqher
docker build -t tfg_ros2_humble .
```

> La imagen base `osrf/ros:humble-desktop-full` debe estar disponible localmente
> o con acceso a internet para descargarla.

### 2. Arrancar el contenedor

```bash
xhost +local:docker

docker run -it \
  --name tfg_ros2 \
  --network host \
  --gpus all \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/TFG-jesesqher:/root/TFG-jesesqher \
  tfg_ros2_humble \
  bash
```

> Sin GPU NVIDIA, sustituye `--gpus all` por `-e LIBGL_ALWAYS_SOFTWARE=1`.
> La simulación irá más lenta.

### 3. Volver a entrar al contenedor (sesiones posteriores)

```bash
docker start tfg_ros2
docker exec -it tfg_ros2 bash
```

### 4. Compilar el workspace (primera vez dentro del contenedor)

```bash
cd /root/TFG-jesesqher/ros2/Simulated/tfg_ws
colcon build
source install/setup.bash
```

---

## Lanzar la simulación

### Simulación básica con Nav2

```bash
ros2 launch turtlebot4_cpp_slam_nav sim.launch.py
```

**Argumentos disponibles:**

| Argumento | Valores | Por defecto | Descripción |
|---|---|---|---|
| `planner` | `dijkstra`, `astar` | `dijkstra` | Algoritmo de planificación utilizado |
| `smooth` | `true`, `false` | `true` | Suavizado de trayectorias |
| `world` | `warehouse`, `depot`, `maze` | `warehouse` | Mundo de simulación |
| `launch_rviz` | `true`, `false` | `true` | Lanzar RViz |

**Ejemplos:**

```bash
# Dijkstra con suavizado (por defecto)
ros2 launch turtlebot4_cpp_slam_nav sim.launch.py

# A* sin suavizado
ros2 launch turtlebot4_cpp_slam_nav sim.launch.py planner:=astar smooth:=false

# Sin RViz
ros2 launch turtlebot4_cpp_slam_nav sim.launch.py launch_rviz:=false
```

El launch file automatiza la inicialización completa: pose inicial de AMCL, undock del robot y arranque del nodo de métricas.

### Simulación con métricas dinámicas (PC de alto rendimiento)

```bash
ros2 launch turtlebot4_cpp_slam_nav sim2.launch.py
```

---

## Análisis de datos

```bash
cd ~/TFG-jesesqher/ros2/Analysis
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# Análisis estático (métricas de planificación)
python static.py

# Análisis dinámico (métricas de ejecución en tiempo real)
python dynamic.py
```

Las gráficas se guardan en `Analysis/plots/static/` y `Analysis/plots/dynamic/`.