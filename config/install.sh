#!/bin/bash

# Script de instalación rápida para el sistema de navegación ArUco
# Uso: ./install.sh nombre_de_tu_paquete

set -e  # Salir si hay algún error

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # Sin color

echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  INSTALADOR - Sistema de Navegación ArUco${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
echo ""

# Verificar argumento
if [ -z "$1" ]; then
    echo -e "${RED}Error: Debes proporcionar el nombre de tu paquete ROS2${NC}"
    echo -e "Uso: ${YELLOW}./install.sh nombre_de_tu_paquete${NC}"
    echo ""
    echo "Ejemplo:"
    echo "  ./install.sh my_robot_pkg"
    exit 1
fi

PACKAGE_NAME=$1

# Detectar workspace ROS2
if [ -z "$ROS_WORKSPACE" ]; then
    if [ -d "$HOME/ros2_ws" ]; then
        ROS_WORKSPACE="$HOME/ros2_ws"
    elif [ -d "$HOME/colcon_ws" ]; then
        ROS_WORKSPACE="$HOME/colcon_ws"
    else
        echo -e "${RED}No se detectó workspace de ROS2${NC}"
        read -p "Introduce la ruta a tu workspace (ej: ~/ros2_ws): " ROS_WORKSPACE
        ROS_WORKSPACE="${ROS_WORKSPACE/#\~/$HOME}"  # Expandir ~
    fi
fi

PACKAGE_PATH="$ROS_WORKSPACE/src/$PACKAGE_NAME"

echo -e "Workspace: ${GREEN}$ROS_WORKSPACE${NC}"
echo -e "Paquete: ${GREEN}$PACKAGE_NAME${NC}"
echo -e "Ruta: ${GREEN}$PACKAGE_PATH${NC}"
echo ""

# Verificar si el paquete existe
if [ ! -d "$PACKAGE_PATH" ]; then
    echo -e "${RED}Error: El paquete '$PACKAGE_NAME' no existe en $ROS_WORKSPACE/src${NC}"
    read -p "¿Deseas crear el paquete? (s/n): " CREATE_PKG
    
    if [ "$CREATE_PKG" = "s" ] || [ "$CREATE_PKG" = "S" ]; then
        echo -e "${YELLOW}Creando paquete...${NC}"
        cd "$ROS_WORKSPACE/src"
        ros2 pkg create --build-type ament_python "$PACKAGE_NAME" \
            --dependencies rclpy std_msgs geometry_msgs nav_msgs sensor_msgs
        echo -e "${GREEN}✓ Paquete creado${NC}"
    else
        exit 1
    fi
fi

# Crear directorios necesarios
echo -e "${YELLOW}Creando estructura de directorios...${NC}"
mkdir -p "$PACKAGE_PATH/scripts"
mkdir -p "$PACKAGE_PATH/config"
mkdir -p "$PACKAGE_PATH/launch"
echo -e "${GREEN}✓ Directorios creados${NC}"

# Copiar archivos
echo -e "${YELLOW}Copiando archivos...${NC}"

# Scripts Python
echo "  → aruco_navigator.py"
cp aruco_navigator.py "$PACKAGE_PATH/scripts/"
chmod +x "$PACKAGE_PATH/scripts/aruco_navigator.py"

echo "  → calibration_helper.py"
cp calibration_helper.py "$PACKAGE_PATH/scripts/"
chmod +x "$PACKAGE_PATH/scripts/calibration_helper.py"

echo "  → system_tester.py"
cp system_tester.py "$PACKAGE_PATH/scripts/"
chmod +x "$PACKAGE_PATH/scripts/system_tester.py"

# Si existe subcriptor.py original, preguntar si sobrescribir
if [ -f "$PACKAGE_PATH/scripts/subcriptor.py" ]; then
    read -p "  ⚠️  subcriptor.py ya existe. ¿Sobrescribir? (s/n): " OVERWRITE
    if [ "$OVERWRITE" = "s" ] || [ "$OVERWRITE" = "S" ]; then
        echo "  → subcriptor.py (sobrescrito)"
        cp subcriptor.py "$PACKAGE_PATH/scripts/"
        chmod +x "$PACKAGE_PATH/scripts/subcriptor.py"
    fi
else
    echo "  → subcriptor.py"
    cp subcriptor.py "$PACKAGE_PATH/scripts/"
    chmod +x "$PACKAGE_PATH/scripts/subcriptor.py"
fi

# Configuración
echo "  → aruco_nav_params.yaml"
cp aruco_nav_params.yaml "$PACKAGE_PATH/config/"

# Launch file
echo "  → aruco_navigation_launch.py"
cp aruco_navigation_launch.py "$PACKAGE_PATH/launch/"

# README
echo "  → README.md"
cp README.md "$PACKAGE_PATH/"

echo -e "${GREEN}✓ Archivos copiados${NC}"

# Actualizar setup.py
echo ""
echo -e "${YELLOW}Configurando setup.py...${NC}"

SETUP_FILE="$PACKAGE_PATH/setup.py"

# Crear backup
cp "$SETUP_FILE" "$SETUP_FILE.backup"
echo "  ✓ Backup creado: setup.py.backup"

# Añadir entry points si no existen
if ! grep -q "aruco_navigator" "$SETUP_FILE"; then
    echo "  Añadiendo entry points..."
    
    # Esto es un poco delicado, mejor mostrar instrucciones
    echo ""
    echo -e "${YELLOW}⚠️  Debes añadir manualmente estas líneas a setup.py:${NC}"
    echo ""
    echo -e "${BLUE}En entry_points = {...}, añade en 'console_scripts':${NC}"
    echo ""
    echo "    'aruco_navigator = $PACKAGE_NAME.aruco_navigator:main',"
    echo "    'calibration_helper = $PACKAGE_NAME.calibration_helper:main',"
    echo "    'system_tester = $PACKAGE_NAME.system_tester:main',"
    echo "    'subcriptor = $PACKAGE_NAME.subcriptor:main',"
    echo ""
else
    echo "  ✓ Entry points ya configurados"
fi

# Instrucciones finales
echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✓ INSTALACIÓN COMPLETADA${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}PRÓXIMOS PASOS:${NC}"
echo ""
echo "1. ${BLUE}Revisa y completa setup.py${NC}"
echo "   Verifica que los entry_points estén configurados correctamente"
echo ""
echo "2. ${BLUE}Compila el paquete${NC}"
echo "   cd $ROS_WORKSPACE"
echo "   colcon build --packages-select $PACKAGE_NAME"
echo "   source install/setup.bash"
echo ""
echo "3. ${BLUE}Calibra la cámara${NC}"
echo "   ros2 run $PACKAGE_NAME calibration_helper"
echo ""
echo "4. ${BLUE}Prueba el sistema${NC}"
echo "   ros2 run $PACKAGE_NAME system_tester"
echo ""
echo "5. ${BLUE}Ejecuta la navegación${NC}"
echo "   ros2 run $PACKAGE_NAME aruco_navigator"
echo ""
echo -e "${GREEN}✓ ¡Consulta README.md para más información!${NC}"
echo ""

# Preguntar si compilar ahora
read -p "¿Deseas compilar el paquete ahora? (s/n): " COMPILE_NOW

if [ "$COMPILE_NOW" = "s" ] || [ "$COMPILE_NOW" = "S" ]; then
    echo ""
    echo -e "${YELLOW}Compilando...${NC}"
    cd "$ROS_WORKSPACE"
    colcon build --packages-select "$PACKAGE_NAME"
    
    if [ $? -eq 0 ]; then
        echo ""
        echo -e "${GREEN}✓ Compilación exitosa${NC}"
        echo ""
        echo -e "${YELLOW}No olvides ejecutar:${NC}"
        echo "  source $ROS_WORKSPACE/install/setup.bash"
    else
        echo ""
        echo -e "${RED}⚠️  Error en la compilación${NC}"
        echo "Revisa setup.py y vuelve a intentar"
    fi
fi

echo ""
