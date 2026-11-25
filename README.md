# UR3 – Control cinemático y planificación con ROS 2 Jazzy

Proyecto personal de robótica sobre un **Universal Robots UR3** simulado en **Gazebo Harmonic**. Va desde el control articular de bajo nivel, con perfiles de trayectoria y un lazo de control propios, hasta la planificación con evasión de obstáculos en **MoveIt 2**.

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![C++](https://img.shields.io/badge/C++-17-00599C?logo=cplusplus)
![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python)

<!-- TODO: añadir un GIF del robot rodeando el pilar, p. ej. media/fase2_pilar.gif
     (ojo: la carpeta docs/ está en .gitignore, por eso se usa media/) -->

---

## Contenido

- [Resumen](#resumen)
- [Estructura del repositorio](#estructura-del-repositorio)
- [Dependencias de terceros (Universal Robots)](#dependencias-de-terceros-universal-robots)
- [Guía de instalación paso a paso](#guía-de-instalación-paso-a-paso)
- [Uso](#uso)
- [Tests](#tests)
- [Problemas resueltos](#problemas-resueltos)
- [Hoja de ruta](#hoja-de-ruta)

---

## Resumen

| Fase | Contenido | Lenguaje | Estado |
|---|---|---|---|
| **1. Control cinemático** | Interpolación cúbica/quíntica, lazo *feedforward* + proporcional sobre `forward_velocity_controller`, servicio *Go Home* con perfil trapezoidal sincronizado | Python | ✅ Completada |
| **Herramientas de análisis** | Cinemática directa y jacobiano geométrico propios, a partir de la calibración del fabricante; detección de singularidades por SVD | Python | ✅ Completada |
| **2. Planificación con MoveIt 2** | `MoveGroupInterface` multihilo, obstáculo de colisión añadido en tiempo de ejecución, objetivo cartesiano XYZ con orientación libre | C++ | ✅ Completada |
| **3A. Trayectorias cartesianas** | *Pick & place* con aproximación y retirada lineales (`computeCartesianPath`) y transporte con OMPL | C++ | 🚧 En curso |
| **3B. Visión** | Cámara simulada en el efector final para *visual servoing* | C++/Python | 📋 Pendiente |

---

## Estructura del repositorio

El repositorio es la raíz de un **workspace de colcon**. Solo contiene los paquetes propios; los del fabricante se descargan aparte (ver la sección siguiente).

```
ur3-ros2-control-moveit/
├── ur3_dependencies.repos          # Lista de repositorios de UR que hay que descargar
├── .gitignore                      # Excluye build/, install/, log/ y los paquetes de UR
└── src/
    ├── my_ur_control/              # ament_python – Fase 1 y herramientas de análisis
    │   ├── my_ur_control/
    │   │   ├── custom_controller.py    # Lazo FF + Kp sobre forward_velocity_controller
    │   │   ├── interpolators.py        # Interpoladores cúbico y quíntico
    │   │   ├── go_home_service.py      # Servicio /go_home (std_srvs/SetBool)
    │   │   ├── trajectory_utils.py     # Perfil trapezoidal multi-articulación sincronizado
    │   │   ├── ur_kinematics.py        # FK + jacobiano desde default_kinematics.yaml (sin ROS)
    │   │   ├── singularity_utils.py    # Límites articulares y detección de singularidades (SVD)
    │   │   └── simple_trajectory.py    # Envío básico de JointTrajectory
    │   └── test/                       # pytest + launch_testing
    │
    └── ur_moveit_cpp/              # ament_cmake – Fase 2 en adelante
        ├── src/move_arm.cpp            # Planificación con evasión de un pilar
        └── launch/run_move_arm.launch.py
```

---

## Dependencias de terceros (Universal Robots)

Este proyecto se apoya en los paquetes oficiales de **Universal Robots**, que contienen el modelo del robot (cinemática, límites, mallas), la simulación en Gazebo y la configuración de MoveIt. Son propiedad de su autor y tienen su propia licencia, así que **no se incluyen en este repositorio** y **no se modifican**: se usan tal como los publica el fabricante.

| Paquete | Para qué se usa | Cómo se obtiene |
|---|---|---|
| `ur_description` ([repo](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)) | URDF/Xacro, `default_kinematics.yaml`, `joint_limits.yaml` del UR3 | Código fuente, con `ur3_dependencies.repos` |
| `ur_simulation_gz` ([repo](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation)) | Lanzar el UR3 en Gazebo Harmonic con ros2_control | Código fuente, con `ur3_dependencies.repos` |
| `ur_moveit_config` ([repo](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)) | SRDF, cinemática y límites para MoveIt 2 | Paquete binario de Jazzy (`apt`), lo instala `rosdep` |

Las carpetas de los dos primeros están en el `.gitignore`, así que aunque estén dentro de `src/` nunca se suben por error.

---

## Guía de instalación paso a paso

Probado en **Ubuntu 24.04** con **ROS 2 Jazzy**.

### 1. Requisitos previos

Necesitas ROS 2 Jazzy instalado ([guía oficial](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)). Después, instala las herramientas de compilación y gestión de dependencias:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool git

# Solo la primera vez en el equipo
sudo rosdep init
rosdep update
```

### 2. Clonar el repositorio

El repositorio ya es el workspace, así que se clona directamente con el nombre que quieras darle:

```bash
cd ~
git clone https://github.com/francisco-campayo/ur3-ros2-control-moveit.git ur_ws
cd ~/ur_ws
```

### 3. Descargar los paquetes de Universal Robots

```bash
vcs import src < ur3_dependencies.repos
```

Esto crea `src/Universal_Robots_ROS2_Description/` y `src/Universal_Robots_ROS2_GZ_Simulation/` en la rama `jazzy`.

### 4. Instalar el resto de dependencias

`rosdep` lee los `package.xml` de todos los paquetes e instala con `apt` lo que falte: MoveIt 2, `ur_moveit_config`, Gazebo Harmonic, `gz_ros2_control`, los controladores de UR, etc.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Compilar

```bash
colcon build --symlink-install
source install/setup.bash
```

Debe terminar con todos los paquetes en `Finished`.

> 💡 Cada terminal nueva necesita `source ~/ur_ws/install/setup.bash`. Para no repetirlo:
> `echo "source ~/ur_ws/install/setup.bash" >> ~/.bashrc`

### 6. Comprobar que funciona

```bash
# Terminal 1: simulación con MoveIt
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur3

# Terminal 2: el robot debe rodear el pilar y terminar sin errores
ros2 launch ur_moveit_cpp run_move_arm.launch.py
```

---

## Uso

### Simulación base

```bash
# Con MoveIt (Fase 2+)
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur3

# Solo control, arrancando el controlador de velocidad (Fase 1)
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur3 \
     initial_joint_controller:=forward_velocity_controller
```

### Fase 1 – Control cinemático (Python)

```bash
# Lazo feedforward + proporcional entre waypoints articulares
ros2 run my_ur_control custom_controller

# Servicio Go Home (necesita scaled_joint_trajectory_controller activo)
ros2 run my_ur_control go_home_service
ros2 service call /go_home std_srvs/srv/SetBool "{data: true}"
```

**Ley de control** de `custom_controller`, por articulación *i*:

$$\dot q_{cmd,i} = \dot q_{ref,i}(t) + K_p\,\big(q_{ref,i}(t) - q_i\big)$$

donde $q_{ref}$ y $\dot q_{ref}$ salen de un polinomio quíntico, que da velocidad y aceleración nulas en los extremos. El paso al siguiente waypoint exige dos condiciones: que haya terminado el tiempo nominal **y** que el error articular máximo sea menor de 0.01 rad.

### Fase 2 – Planificación con evasión (C++)

```bash
ros2 launch ur_moveit_cpp run_move_arm.launch.py
```

El nodo:
1. Añade a la escena de planificación un pilar de colisión de 10 × 10 × 40 cm.
2. Pide un objetivo **solo de posición** (`setPositionTarget`). Deja libre la orientación de la muñeca para que OMPL tenga más margen para rodear el obstáculo y alejarse de las singularidades.
3. Planifica, ejecuta y **comprueba el resultado de `execute()`**. Si no hay controlador de trayectoria activo, el proceso termina con código 1.

### Cambiar de controlador en caliente

```bash
# Un controlador definido en el YAML no existe hasta que se carga
ros2 control load_controller --set-state inactive forward_velocity_controller

ros2 control switch_controllers \
     --deactivate scaled_joint_trajectory_controller \
     --activate forward_velocity_controller
```

---

## Tests

```bash
colcon test --packages-select my_ur_control
colcon test-result --verbose

# Validadores independientes (no necesitan ROS en ejecución)
cd src/my_ur_control
python3 test/test_jacobian.py
python3 test/test_trajectory_singularity.py
```

| Test | Qué comprueba |
|---|---|
| `test_trajectory_utils.py` | Perfil trapezoidal: sincronización, límites de velocidad y aceleración, posición final |
| `test_jacobian.py` | El jacobiano analítico frente a la derivada numérica de la FK (error ~1e-10) |
| `test_trajectory_singularity.py` | `σ_min` del jacobiano escalado a lo largo de una trayectoria |
| `test_go_home_service.py` | Integración del servicio con `launch_testing` |

La ruta de `default_kinematics.yaml` se resuelve en este orden:
1. La variable `UR_KINEMATICS_FILE`.
2. El paquete `ur_description` instalado, vía `ament_index`.
3. `~/ur_ws/src/...` como último recurso.

---

## Problemas resueltos

| Síntoma | Causa | Solución |
|---|---|---|
| `Package '...' not found` | La terminal no tiene cargado el workspace | `source install/setup.bash` |
| RViz o el parseo del URDF fallan con un locale en español | Se usa la coma como separador decimal | `SetEnvironmentVariable("LC_NUMERIC", "C")` en el launch |
| Error de Xacro al cargar el modelo en el nodo C++ | Faltan argumentos obligatorios | `MoveItConfigsBuilder` con `mappings={"name": "ur", "ur_type": "ur3"}` |
| `execute()` devuelve `-4` (CONTROL_FAILED) | `scaled_joint_trajectory_controller` está inactivo | Activarlo; el nodo ahora lo detecta y lo informa |
| `no controller with this name exists` | El controlador no estaba cargado en el `controller_manager` | `ros2 control load_controller --set-state inactive ...` |
| Un test de `launch_testing` falla de forma intermitente | `time.sleep()` no ejecuta *callbacks* | `rclpy.spin_once()` en bucle mientras se espera |

**Notas sobre el UR3** (sacadas de los paquetes del fabricante):
- Los *frames* `base` y `base_link` están girados π en Z. `base` coincide con las coordenadas de la consola del robot real.
- El *planning frame* de MoveIt es `world`, unido de forma fija a `base_link` sin desplazamiento. El efector del grupo `ur_manipulator` es `tool0`.
- El límite de `elbow_joint` es ±π. `wrist_3_joint` no tiene límite de posición.

---

## Hoja de ruta

- [x] Control articular por velocidad con perfiles polinómicos
- [x] Servicio *Go Home* con perfil trapezoidal sincronizado
- [x] Modelo cinemático propio y detección de singularidades
- [x] Planificación con MoveIt 2 y evasión de obstáculos
- [ ] **3A:** *Pick & place* con tramos cartesianos lineales, comprobación de `fraction`, parametrización temporal con TOTG y métricas de error
- [ ] **3B:** Cámara RGB/RGB-D en el efector final (Gazebo + `ros_gz_bridge`)
- [ ] **3C:** *Pick & place* guiado por visión

---

## Créditos

- Modelo, simulación y configuración de MoveIt del UR3: [Universal Robots](https://github.com/UniversalRobots), bajo sus respectivas licencias.
- Código de `my_ur_control` y `ur_moveit_cpp`: Francisco Campayo Jiménez.

## Autor

**Francisco Campayo Jiménez** – Ingeniero robótico
<!-- TODO: LinkedIn / email -->
