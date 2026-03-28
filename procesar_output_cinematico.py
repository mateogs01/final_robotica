#ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true

import numpy as np
import quaternion
import pandas as pd
import yaml
import matplotlib.pyplot as plt

def procesar_cmd(ruta_archivo):
    """
    Procesa un archivo TXT con formato YAML separado por '---'
    y devuelve un DataFrame con timestamp, x linear, y linear, z angular
    """
    # Leer el archivo
    with open(ruta_archivo, 'r') as file:
        contenido = file.read()
    
    # Separar por los '---' y filtrar documentos vacíos
    documentos = [doc.strip() for doc in contenido.split('---') if doc.strip()]
    
    # Procesar cada documento y crear lista de diccionarios
    datos = []
    for doc in documentos:
        data = yaml.safe_load(doc)
        
        # Calcular timestamp en segundos
        timestamp = data['header']['stamp']['sec'] + data['header']['stamp']['nanosec'] / 1e9
        
        datos.append({
            'timestamp': timestamp,
            'vx': data['twist']['linear']['x'],
            'vy': data['twist']['linear']['y'],
            'w':  data['twist']['angular']['z']
        })
    
    # Crear DataFrame
    df = pd.DataFrame(datos)
    
    return df


def procesar_odom(ruta_archivo):
    """
    Procesa un archivo TXT con formato YAML separado por '---'
    y devuelve un DataFrame con timestamp, x linear, y linear, z angular
    """
    # Leer el archivo
    with open(ruta_archivo, 'r') as file:
        contenido = file.read()
    
    # Separar por los '---' y filtrar documentos vacíos
    documentos = [doc.strip() for doc in contenido.split('---') if doc.strip()]
    
    # Procesar cada documento y crear lista de diccionarios
    datos = []
    for doc in documentos:
        data = yaml.safe_load(doc)
        
        # Calcular timestamp en segundos
        timestamp = data['header']['stamp']['sec'] + data['header']['stamp']['nanosec'] / 1e9
        orientation = data['pose']['pose']['orientation']
        q = np.quaternion(orientation['w'], orientation['x'], orientation['y'], orientation['z'])
        theta = quaternion.as_euler_angles(q)[2]
        
        datos.append({
            'timestamp': timestamp,
            'vx': data['twist']['twist']['linear']['x'],
            'vy': data['twist']['twist']['linear']['y'],
            'w':  data['twist']['twist']['angular']['z'],
            'theta': theta
        })
    
    # Crear DataFrame
    df = pd.DataFrame(datos)
    
    return df


def procesar_pos(ruta_archivo, gt=False, eulerindex=2):
    """
    Procesa un archivo TXT con formato YAML separado por '---'
    y devuelve un DataFrame con timestamp, x, y, theta
    """
    # Leer el archivo
    with open(ruta_archivo, 'r') as file:
        contenido = file.read()
    
    # Separar por los '---' y filtrar documentos vacíos
    documentos = [doc.strip() for doc in contenido.split('---') if doc.strip()]
    
    # Procesar cada documento y crear lista de diccionarios
    datos = []
    for doc in documentos:
        data = yaml.safe_load(doc)
        
        # Calcular timestamp en segundos
        timestamp = data['header']['stamp']['sec'] + data['header']['stamp']['nanosec'] / 1e9
        orientation = data['pose']['pose']['orientation']
        def yaw_from_ros_quat(o):
            x, y, z, w = o['x'], o['y'], o['z'], o['w']
            return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        # if gt:
        #     q = np.quaternion(orientation['w'], orientation['x'], orientation['y'], orientation['z'])
        # else: 
        #     q = np.quaternion(orientation['x'], orientation['y'], orientation['z'],orientation['w'])
        # theta = quaternion.as_euler_angles(q)[eulerindex]
        
        datos.append({
            'timestamp': timestamp,
            'x': data['pose']['pose']['position']['x'],
            'y': data['pose']['pose']['position']['y'],
            # 'q': quaternion.as_euler_angles(q),
            'theta': yaw_from_ros_quat(orientation)
        })
    
    # Crear DataFrame
    df = pd.DataFrame(datos)
    
    return df


# %%

df_cmd_vel = procesar_cmd('volume/outputs/cmd_vel_1.txt')
print("DataFrame cmd:")
print(df_cmd_vel)

df_odom = procesar_odom('volume/outputs/odom_1.txt')
print("DataFrame odom:")
print(df_odom)

change_vel = df_cmd_vel[(df_cmd_vel.vx!=0) | (df_cmd_vel.vy!=0) | (df_cmd_vel.w!=0)]
# %%
i = 0
n = 10

objetivo = 'vx'
t0 = df_cmd_vel.iloc[i].timestamp
t1 = df_cmd_vel.iloc[i+n].timestamp

odom_filtered = df_odom[(df_odom.timestamp >= t0) & (df_odom.timestamp < t1)]


plt.plot(odom_filtered.timestamp-t0, odom_filtered.vx, label='vx')
plt.plot(odom_filtered.timestamp-t0, odom_filtered.vy, label='vy')
for j in range(n):
    ti = df_cmd_vel.iloc[i+j].timestamp - t0
    tf = df_cmd_vel.iloc[i+j+1].timestamp - t0
    h = df_cmd_vel.iloc[i+j][objetivo]
    plt.hlines(h, ti, tf, colors='red')
plt.hlines(0,0,0, colors='red', label=f'{objetivo} objetivo')
plt.legend()
plt.ylabel('Velocidad (m/seg)')
plt.xlabel('Tiempo (seg)')
plt.show()
# %%
i = 11
n = 10

objetivo = 'vy'
t0 = df_cmd_vel.iloc[i].timestamp
t1 = df_cmd_vel.iloc[i+n].timestamp

odom_filtered = df_odom[(df_odom.timestamp >= t0) & (df_odom.timestamp < t1)]


plt.plot(odom_filtered.timestamp-t0, odom_filtered.vx, label='vx')
plt.plot(odom_filtered.timestamp-t0, odom_filtered.vy, label='vy')
for j in range(n):
    ti = df_cmd_vel.iloc[i+j].timestamp - t0
    tf = df_cmd_vel.iloc[i+j+1].timestamp - t0
    h = df_cmd_vel.iloc[i+j][objetivo]
    plt.hlines(h, ti, tf, colors='red')
plt.hlines(0,0,0, colors='red', label=f'{objetivo} objetivo')
plt.legend()
plt.xlabel('Tiempo (seg)')
plt.ylabel('Velocidad (m/seg)')
plt.show()
# %%

i = 30
n = 11

t0 = df_cmd_vel.iloc[i].timestamp
t1 = df_cmd_vel.iloc[i+n].timestamp

odom_filtered = df_odom[(df_odom.timestamp >= t0) & (df_odom.timestamp < t1)]


plt.plot(odom_filtered.timestamp-t0, odom_filtered.vx, label='vx')
plt.plot(odom_filtered.timestamp-t0, odom_filtered.vy, label='vy')
objetivo = 'vx'
for j in range(n):
    ti = df_cmd_vel.iloc[i+j].timestamp - t0
    tf = df_cmd_vel.iloc[i+j+1].timestamp - t0
    h = df_cmd_vel.iloc[i+j][objetivo]
    plt.hlines(h, ti, tf, colors='green')
plt.hlines(0,0,0, colors='green', label=f'{objetivo} objetivo')

objetivo = 'vy'
for j in range(n):
    ti = df_cmd_vel.iloc[i+j].timestamp - t0
    tf = df_cmd_vel.iloc[i+j+1].timestamp - t0
    h = df_cmd_vel.iloc[i+j][objetivo]
    plt.hlines(h, ti, tf, colors='red')

plt.hlines(0,0,0, colors='red', label=f'{objetivo} objetivo')

plt.legend()
plt.xlabel('Tiempo (seg)')
plt.ylabel('Velocidad (m/seg)')
plt.show()


# %%
i = 22
n = 7

objetivo = 'w'
t0 = df_cmd_vel.iloc[i].timestamp
t1 = df_cmd_vel.iloc[i+n].timestamp

odom_filtered = df_odom[(df_odom.timestamp >= t0) & (df_odom.timestamp < t1)]


plt.plot(odom_filtered.timestamp-t0, odom_filtered.vx, label='vx')
plt.plot(odom_filtered.timestamp-t0, odom_filtered.vy, label='vy')
plt.plot(odom_filtered.timestamp-t0, odom_filtered.w, label='w')
for j in range(n):
    ti = df_cmd_vel.iloc[i+j].timestamp - t0
    tf = df_cmd_vel.iloc[i+j+1].timestamp - t0
    h = df_cmd_vel.iloc[i+j][objetivo]
    plt.hlines(h, ti, tf, colors='red')
plt.hlines(0,0,0, colors='red', label=f'{objetivo} objetivo')
plt.legend()
plt.xlabel('Tiempo (seg)')
plt.ylabel('Velocidad (rad/seg)')
plt.show()


# %%
df_odom = procesar_pos('volume/outputs/odom_2.txt', eulerindex=2)
print("DataFrame odom:")
print(df_odom)

df_gt = procesar_pos('volume/outputs/gt_2.txt', gt=True, eulerindex=2)
print("DataFrame gt:")
print(df_gt)

# %%

ti = df_gt.iloc[0].timestamp
tf = df_gt.iloc[-1].timestamp

plt.scatter(df_gt.x, df_gt.y, c=df_gt.timestamp-ti, s=5)
plt.plot(df_odom.x, df_odom.y, c='r')
plt.colorbar()

points = []

for t in range(t1,tf,2):
    points.append()

# %%
plt.plot(df_odom.timestamp-ti, df_odom.x, label='x')
plt.plot(df_odom.timestamp-ti, df_odom.y, label='y')
plt.plot(df_odom.timestamp-ti, df_gt.x, label='x truth')
plt.plot(df_odom.timestamp-ti, df_gt.y, label='y truth')

plt.legend()
plt.ylabel('Velocidad (m/seg)')
plt.xlabel('Tiempo (seg)')
plt.show()

# %%

plt.plot(df_odom.timestamp-ti, np.unwrap(df_odom.theta), label='theta')
plt.plot(df_odom.timestamp-ti, np.unwrap(df_gt.theta), label='theta truth')

plt.legend()
plt.ylabel('Angulo (theta)')
plt.xlabel('Tiempo (seg)')
plt.show()
