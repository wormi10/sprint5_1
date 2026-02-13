# 🚀 INICIO RÁPIDO - Sistema de Navegación ArUco

## 📦 Archivos Incluidos

1. **aruco_navigator.py** - Controlador principal de navegación
2. **subcriptor.py** - Monitor de tópicos y ArUcos (tu archivo original)
3. **calibration_helper.py** - Asistente para calibrar la cámara
4. **system_tester.py** - Script de prueba del sistema
5. **aruco_nav_params.yaml** - Archivo de configuración
6. **aruco_navigation_launch.py** - Launch file
7. **install.sh** - Script de instalación automática
8. **README.md** - Documentación completa

## ⚡ Instalación Rápida (3 pasos)

### Opción A: Instalación Automática

```bash
# 1. Hacer ejecutable el instalador
chmod +x install.sh

# 2. Ejecutar instalador (reemplaza 'mi_paquete' con el nombre de tu paquete)
./install.sh mi_paquete

# 3. Seguir las instrucciones en pantalla
```

### Opción B: Instalación Manual

```bash
# 1. Copiar archivos a tu paquete ROS2
cd ~/ros2_ws/src/tu_paquete
mkdir -p scripts config launch

cp aruco_navigator.py scripts/
cp calibration_helper.py scripts/
cp system_tester.py scripts/
cp subcriptor.py scripts/
cp aruco_nav_params.yaml config/
cp aruco_navigation_launch.py launch/
chmod +x scripts/*.py

# 2. Editar setup.py y añadir entry_points:
#    'aruco_navigator = tu_paquete.aruco_navigator:main',
#    'calibration_helper = tu_paquete.calibration_helper:main',
#    'system_tester = tu_paquete.system_tester:main',
#    'subcriptor = tu_paquete.subcriptor:main',

# 3. Compilar
cd ~/ros2_ws
colcon build --packages-select tu_paquete
source install/setup.bash
```

## 🎯 Uso Rápido (3 pasos)

### 1️⃣ Calibrar la cámara

```bash
ros2 run tu_paquete calibration_helper
```

- Coloca dos ArUcos visibles
- Mide la distancia real entre ellos
- El script calculará las conversiones automáticamente
- Actualiza `config/aruco_nav_params.yaml` con los valores

### 2️⃣ Probar el sistema

```bash
ros2 run tu_paquete system_tester
```

Verificará:
- ✓ Parámetros de conversión
- ✓ ArUcos detectados
- ✓ Robot visible
- ✓ Objetivos disponibles
- ✓ Simulación de control

### 3️⃣ ¡Navegar!

```bash
ros2 run tu_paquete aruco_navigator
```

El robot se moverá automáticamente entre los ArUcos 20, 21, 22 y 23.

## ⚙️ Configuración Importante

### Identificar el ArUco del robot

Edita `scripts/aruco_navigator.py` línea 52:

```python
self.robot_aruco_id = 3  # Cambia a 8 si tu robot tiene el ArUco 8
```

Para saber cuál es, ejecuta `subcriptor` y observa cuál se mueve con tu robot.

### Ajustar parámetros de calibración

Edita `config/aruco_nav_params.yaml`:

```yaml
aruco_navigator:
  ros__parameters:
    # Resolución de tu cámara
    camera_resolution_x: 1280
    camera_resolution_y: 720
    
    # IMPORTANTE: Calibrar estos valores con calibration_helper
    real_world_width: 2.0   # Ancho del área en metros
    real_world_height: 1.5  # Alto del área en metros
    
    # Ajustes de navegación
    goal_tolerance: 0.15        # Precisión (metros)
    linear_speed_max: 0.5       # Velocidad máxima
    angular_speed_max: 1.0      # Velocidad de giro
```

## 🔧 Solución Rápida de Problemas

| Problema | Solución |
|----------|----------|
| Robot no se mueve | Verifica que el ID del robot (3 u 8) sea correcto en aruco_navigator.py |
| "No hay objetivos visibles" | Asegúrate de que los ArUcos 20-23 estén en el campo de visión |
| Robot va muy rápido | Reduce `linear_speed_max` y `angular_speed_max` |
| Conversión incorrecta | Re-calibra con `calibration_helper` |
| El robot oscila | Aumenta `goal_tolerance` a 0.20 |

## 📊 Flujo del Sistema

```
1. SELECTING_TARGET
   ↓
   Selecciona aleatoriamente ArUco (20, 21, 22 o 23)
   ↓
2. NAVIGATING
   ↓
   → Calcula distancia y ángulo al objetivo
   → Ajusta orientación del robot
   → Avanza hacia el objetivo
   ↓
3. REACHED
   ↓
   Espera 2 segundos
   ↓
   Vuelve a paso 1
```

## 📝 Logs de Ejemplo

```
[INFO] [aruco_navigator]: 🎯 Nuevo objetivo seleccionado: ArUco 21
[INFO] [aruco_navigator]: Navegando a ArUco 21 | Distancia: 0.45m | Vel: lin=0.36 ang=0.12
[INFO] [aruco_navigator]: ✅ Objetivo ArUco 21 alcanzado!
```

## 🆘 ¿Necesitas ayuda?

1. Lee el **README.md** completo para información detallada
2. Ejecuta **system_tester** para diagnóstico
3. Verifica que todos los ArUcos tengan AGE < 0.5s
4. Revisa los logs en busca de errores

## ✅ Checklist Pre-ejecución

- [ ] Cámara configurada y publicando datos
- [ ] ArUcos 20, 21, 22, 23 visibles en la cámara
- [ ] Robot (ArUco 3 u 8) visible
- [ ] Parámetros de calibración actualizados
- [ ] ID del robot configurado correctamente
- [ ] Sistema probado con `system_tester`

---

**¡Listo para navegar! 🤖**

Consulta README.md para documentación completa.
