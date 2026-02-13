# Sistema de Navegación Autónoma con ArUco Markers

Sistema ROS2 para navegación autónoma de un robot hacia marcadores ArUco utilizando visión cenital.

##  Descripción

El robot se mueve automáticamente hacia los marcadores ArUco 20, 21, 22 o 23, seleccionándolos de forma aleatoria. Una vez alcanzado un objetivo, selecciona otro y continúa indefinidamente.

##  Componentes

1. **subcriptor.py** - Monitor de tópicos que muestra el estado de todos los ArUcos
2. **aruco_navigator.py** - Controlador de navegación autónoma
3. **aruco_nav_params.yaml** - Parámetros de configuración

##  Calibración de la Cámara

**¡MUY IMPORTANTE!** Antes de usar el sistema, debes calibrar la conversión de píxeles a metros:

### Paso 1: Medir el área real

1. Coloca dos marcadores ArUco en posiciones conocidas
2. Mide la distancia física entre ellos en metros
3. Anota las coordenadas en píxeles de ambos marcadores (usando subcriptor.py)

### Paso 2: Calcular la escala

Ejemplo:
```python
# Si dos marcadores están separados:
# - En píxeles: ArUco A (100, 200) y ArUco B (600, 200)
# - En metros: 1.5 metros de distancia horizontal

distancia_pixeles = 600 - 100 = 500 px
distancia_real = 1.5 m
escala = 1.5 / 500 = 0.003 m/px

# Para una imagen de 1280x720:
ancho_real = 1280 * 0.003 = 3.84 metros
alto_real = 720 * 0.003 = 2.16 metros
```

### Paso 3: Actualizar parámetros

Edita `aruco_nav_params.yaml`:
```yaml
aruco_navigator:
  ros__parameters:
    camera_resolution_x: 1280  # Resolución de tu cámara
    camera_resolution_y: 720
    real_world_width: 3.84     # Tus mediciones
    real_world_height: 2.16
```

##  Instalación

### 1. Copiar archivos al workspace

```bash
# Asumiendo que tu workspace es ~/ros2_ws
cd ~/ros2_ws/src/tu_paquete

# Copiar los archivos Python
cp aruco_navigator.py scripts/
cp subcriptor.py scripts/
chmod +x scripts/*.py

# Copiar configuración
cp aruco_nav_params.yaml config/

# Copiar launch file (opcional)
cp aruco_navigation_launch.py launch/
```

### 2. Configurar package.xml

Añade estas dependencias:
```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>sensor_msgs</depend>
```

### 3. Configurar setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'tu_paquete'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu@email.com',
    description='Sistema de navegación con ArUco',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subcriptor = tu_paquete.subcriptor:main',
            'aruco_navigator = tu_paquete.aruco_navigator:main',
        ],
    },
)
```

### 4. Compilar

```bash
cd ~/ros2_ws
colcon build --packages-select tu_paquete
source install/setup.bash
```

##  Uso

### Opción 1: Ejecución individual

**Terminal 1 - Monitor (opcional):**
```bash
ros2 run tu_paquete subcriptor
```

**Terminal 2 - Navegador:**
```bash
ros2 run tu_paquete aruco_navigator --ros-args --params-file config/aruco_nav_params.yaml
```

### Opción 2: Con launch file

```bash
ros2 launch tu_paquete aruco_navigation_launch.py
```

##  Configuración del Robot

### Identificar el ArUco del robot

Tienes dos ArUcos: 3 y 8. Identifica cuál está en tu robot:

```bash
# Ejecuta el monitor y observa cuál se mueve con tu robot
ros2 run tu_paquete subcriptor
```

Luego edita `aruco_navigator.py` línea 52:
```python
self.robot_aruco_id = 3  # Cambia a 8 si es necesario
```

##  Parámetros Ajustables

En `aruco_nav_params.yaml`:

- **goal_tolerance**: Distancia mínima al objetivo (metros)
  - Valor pequeño (0.05): Más preciso, puede oscilar
  - Valor grande (0.20): Menos preciso, más estable

- **linear_speed_max**: Velocidad máxima del robot
  - Ajustar según las capacidades de tu robot

- **angular_speed_max**: Velocidad de rotación máxima
  - Reducir si el robot gira demasiado rápido

##  Solución de Problemas

### El robot no se mueve

1. **Verificar tópicos:**
   ```bash
   ros2 topic list
   ros2 topic echo /cmd_vel
   ```

2. **Verificar visibilidad de ArUcos:**
   ```bash
   ros2 run tu_paquete subcriptor
   ```
   - Verifica que el AGE sea < 0.5s para todos los ArUcos

3. **Verificar que el ID del robot es correcto:**
   - Observa cuál ArUco (3 u 8) se mueve cuando mueves el robot

### El robot va muy rápido/lento

Ajusta en `aruco_nav_params.yaml`:
```yaml
linear_speed_max: 0.3  # Reducir velocidad
angular_speed_max: 0.8
```

### Conversión de coordenadas incorrecta

Recalibra siguiendo la sección "Calibración de la Cámara"

### El robot oscila cerca del objetivo

Aumenta la tolerancia:
```yaml
goal_tolerance: 0.20  # Aumentar de 0.15 a 0.20
```

## Logs y Monitoreo

El navegador muestra información útil:

```
[INFO] [aruco_navigator]:  Nuevo objetivo seleccionado: ArUco 21
[INFO] [aruco_navigator]: Navegando a ArUco 21 | Distancia: 0.45m | Vel: lin=0.36 ang=0.12
[INFO] [aruco_navigator]:  Objetivo ArUco 21 alcanzado!
```

##  Flujo del Programa

1. **SELECTING_TARGET**: Selecciona aleatoriamente un ArUco (20-23)
2. **NAVIGATING**: 
   - Calcula vector hacia el objetivo
   - Ajusta orientación del robot
   - Avanza hacia el objetivo
3. **REACHED**: 
   - Detiene el robot
   - Espera 2 segundos
   - Vuelve a paso 1
