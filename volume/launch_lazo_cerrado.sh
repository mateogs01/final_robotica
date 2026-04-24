#!/bin/bash

# Configuración
OUTPUT_DIR=~/ros2_ws/src/robotica/outputs/lazo_cerrado
mkdir -p $OUTPUT_DIR

if [[ -z $1 ]];
then 
    EXPERIMENTO=TEST
    GOAL_FILE="$OUTPUT_DIR/goal_TEST.txt"
    echo "No parameter passed."
else
    EXPERIMENTO=$1
    GOAL_FILE="$OUTPUT_DIR/goal_path.txt"
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
    kill $PID_GT $PID_OMNI $PID_GOAL 2>/dev/null
    sleep 2
    
    echo "Procesos detenidos. Archivos en: $OUTPUT_DIR"
    exit 0
}

# Configurar trap al inicio
trap cleanup SIGINT SIGTERM EXIT

echo "Iniciando sistema..."

# 2. Captura de topics
echo " - Capturando topics..."

#ros2 topic pub  /ground_truth/target_path nav_msgs/msg/Path &
#PID_PUB=$!
#sleep 2
#ros2 topic echo /ground_truth/target_path > $GOAL_FILE 2>/dev/null &
#sleep 2
#PID_GOAL=$!
#kill $PID_PUB  >/dev/null
echo "¡INCIAR COPPELIA!"
sleep 10

# 1. Nodo de odometría
ros2 topic echo /robot/ground_truth > $GT_FILE 2>/dev/null &
PID_GT=$!
echo " - Iniciando nodo lazo_cerrado..."
ros2 launch lazo_cerrado lazo_cerrado.launch.py
PID_OMNI=$!
sleep 2


echo "Sistema funcionando. Presiona Ctrl+C para detener"
echo "Outputs: $OUTPUT_DIR"


# Llamar cleanup explícitamente
cleanup
