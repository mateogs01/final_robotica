#!/bin/bash

# Configuración
OUTPUT_DIR=~/ros2_ws/src/robotica/outputs/lazo_cerrado
mkdir -p $OUTPUT_DIR

if [[ -z $1 ]];
then 
    EXPERIMENTO=TEST
    echo "No parameter passed."
else
    EXPERIMENTO=$1
fi

# Archivos de salida
GT_FILE="$OUTPUT_DIR/gt_$EXPERIMENTO.txt"

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
    kill $PID_GT $PID_OMNI 2>/dev/null
    sleep 2
    
    # FUERZA BRUTA - Matar todo lo relacionado
#    echo " - Limpieza forzada de procesos ROS..."
#    pkill -f "ros2 topic echo" 2>/dev/null
#    pkill -f "generador_de_velocidades.py" 2>/dev/null
#    pkill -f "omni_odometry_node" 2>/dev/null
#    pkill -f "cmd_vel" 2>/dev/null
#    pkill -f "odometry" 2>/dev/null
    
    # Matar cualquier proceso Python relacionado con nuestro script
#    pkill -f "python.*generador_de_velocidades" 2>/dev/null
    
#    sleep 1
    
#    # Verificar que no queden procesos
#    if pgrep -f "ros2|generador|omni_odometry" > /dev/null; then
#        echo " - Aún hay procesos, usando SIGKILL..."
#        pkill -9 -f "ros2 topic echo" 2>/dev/null
#        pkill -9 -f "generador_de_velocidades.py" 2>/dev/null
#        pkill -9 -f "omni_odometry_node" 2>/dev/null
#    fi
    
    echo "Procesos detenidos. Archivos en: $OUTPUT_DIR"
    exit 0
}

# Configurar trap al inicio
trap cleanup SIGINT SIGTERM EXIT

echo "Iniciando sistema..."

# 2. Captura de topics
echo " - Capturando topics..."
ros2 topic echo /robot/ground_truth > $GT_FILE 2>/dev/null &
PID_GT=$!
sleep 2

# 1. Nodo de odometría
echo " - Iniciando nodo lazo_cerrado..."
ros2 launch lazo_cerrado lazo_cerrado.launch.py
PID_OMNI=$!
sleep 2


echo "Sistema funcionando. Presiona Ctrl+C para detener"
echo "Outputs: $OUTPUT_DIR"


# Llamar cleanup explícitamente
cleanup
