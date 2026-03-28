#!/bin/bash

# Configuración
OUTPUT_DIR=~/ros2_ws/src/robotica/outputs
mkdir -p $OUTPUT_DIR
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# Archivos de salida
CMD_VEL_FILE="$OUTPUT_DIR/cmd_vel_$TIMESTAMP.txt"
ODOM_FILE="$OUTPUT_DIR/odom_$TIMESTAMP.txt"
GT_FILE="$OUTPUT_DIR/gt_$TIMESTAMP.txt"

# Flag para evitar doble limpieza
CLEANUP_DONE=false

# Función de limpieza
cleanup() {
    if [ "$CLEANUP_DONE" = true ]; then
        return
    fi
    CLEANUP_DONE=true
    
    echo ""
    echo "Deteniendo procesos..."
    
    # Matar procesos específicos por PID
    kill $PID_VEL $PID_CMD_VEL $PID_ODOM $PID_GT $PID_OMNI 2>/dev/null
    sleep 2
    
    # FUERZA BRUTA - Matar todo lo relacionado
    echo " - Limpieza forzada de procesos ROS..."
    pkill -f "ros2 topic echo" 2>/dev/null
    pkill -f "generador_de_velocidades.py" 2>/dev/null
    pkill -f "omni_odometry_node" 2>/dev/null
    pkill -f "cmd_vel" 2>/dev/null
    pkill -f "odometry" 2>/dev/null
    
    # Matar cualquier proceso Python relacionado con nuestro script
    pkill -f "python.*generador_de_velocidades" 2>/dev/null
    
    sleep 1
    
    # Verificar que no queden procesos
    if pgrep -f "ros2|generador|omni_odometry" > /dev/null; then
        echo " - Aún hay procesos, usando SIGKILL..."
        pkill -9 -f "ros2 topic echo" 2>/dev/null
        pkill -9 -f "generador_de_velocidades.py" 2>/dev/null
        pkill -9 -f "omni_odometry_node" 2>/dev/null
    fi
    
    echo "Procesos detenidos. Archivos en: $OUTPUT_DIR"
    exit 0
}

# Configurar trap al inicio
trap cleanup SIGINT SIGTERM EXIT

echo "Iniciando sistema..."

# 1. Nodo de odometría
echo " - Iniciando nodo omni_odometry..."
ros2 run modelo_omnidireccional omni_odometry_node --ros-args -p use_sim_time:=True &
PID_OMNI=$!
sleep 2

# 2. Captura de topics
echo " - Capturando topics..."
# Usar timeout para que no queden colgados si algo falla
timeout 3600 ros2 topic echo /cmd_vel > $CMD_VEL_FILE 2>/dev/null &
PID_CMD_VEL=$!
timeout 3600 ros2 topic echo /robot/odometry > $ODOM_FILE 2>/dev/null &
PID_ODOM=$!
timeout 3600 ros2 topic echo /robot/ground_truth > $GT_FILE 2>/dev/null &
PID_GT=$!
sleep 1

# 3. Generador de velocidades
echo " - Ejecutando secuencia de velocidades..."
python3 ~/ros2_ws/src/robotica/generador_de_velocidades.py --ros-args -p use_sim_time:=True &
PID_VEL=$!

echo "Sistema funcionando. Presiona Ctrl+C para detener"
echo "Outputs: $OUTPUT_DIR"

# Esperar a que el generador termine
wait $PID_VEL 2>/dev/null
EXIT_CODE=$?

if [ $EXIT_CODE -eq 0 ]; then
    echo "La secuencia terminó naturalmente."
else
    echo "El generador terminó con código: $EXIT_CODE"
fi

# DAR TIEMPO PARA QUE TERMINE DE ESCRIBIR ARCHIVOS
sleep 2

# Llamar cleanup explícitamente
cleanup
