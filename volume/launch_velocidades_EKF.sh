#!/bin/bash

# Configuración
OUTPUT_DIR=~/ros2_ws/src/robotica/outputs/EKF
mkdir -p $OUTPUT_DIR

if [[ -z $1 ]];
then 
    EXPERIMENTO=TEST
    echo "No parameter passed."
else
    EXPERIMENTO=$1
fi

# Archivos de salida
CMD_VEL_FILE="$OUTPUT_DIR/cmd_vel_$EXPERIMENTO.txt"
ODOM_FILE="$OUTPUT_DIR/odom_$EXPERIMENTO.txt"
GT_FILE="$OUTPUT_DIR/gt_$EXPERIMENTO.txt"
EKF_FILE="$OUTPUT_DIR/ekf_$EXPERIMENTO.txt"

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
    kill $PID_VEL $PID_CMD_VEL $PID_ODOM $PID_GT $PID_OMNI $PID_EKF $PID_LOC $PID_LASER 2>/dev/null
    sleep 2
    
    echo "Procesos detenidos. Archivos en: $OUTPUT_DIR"
    exit 0
}

# Configurar trap al inicio
trap cleanup SIGINT SIGTERM EXIT

echo "Iniciando sistema..."

# 1. Nodo de odometría
echo " - Iniciando nodo landmark_detector..."
ros2 run imu_laser landmark_detector --ros-args -p use_sim_time:=True -p publish_robot_frame:=base_link_ekf --log-level error &
PID_LASER=$!
echo " - Iniciando nodo omni_odometry..."
ros2 run modelo_omnidireccional omni_odometry_node --ros-args -p use_sim_time:=True --log-level error &
PID_OMNI=$!
echo " - Iniciando nodo localizer..."
ros2 run robmovil_ekf localizer --ros-args -p use_sim_time:=True --log-level error &
PID_LOC=$!
echo "¡INCIAR COPPELIA!"
sleep 10

# 2. Captura de topics
echo " - Capturando topics..."
# Usar timeout para que no queden colgados si algo falla
timeout 3600 ros2 topic echo /cmd_vel > $CMD_VEL_FILE 2>/dev/null &
PID_CMD_VEL=$!
timeout 3600 ros2 topic echo /robot/odometry > $ODOM_FILE 2>/dev/null &
PID_ODOM=$!
timeout 3600 ros2 topic echo /robot/ground_truth > $GT_FILE 2>/dev/null &
PID_GT=$!
timeout 3600 ros2 topic echo /pose > $EKF_FILE 2>/dev/null &
PID_EKF=$!
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
