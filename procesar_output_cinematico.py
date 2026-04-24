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
carpeta = "volume/outputs/"
exp = "1"

df_cmd_vel = procesar_cmd(f'{carpeta}/cmd_vel_{exp}.txt')
print("DataFrame cmd:")
print(df_cmd_vel)

df_odom = procesar_odom(f'{carpeta}/odom_{exp}.txt')
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
carpeta = "volume/outputs/EKF"
exp = "seq3"

df_odom = procesar_pos(f'{carpeta}/odom_{exp}.txt', eulerindex=2)
print("DataFrame odom:")
print(df_odom)

df_gt = procesar_pos(f'{carpeta}/gt_{exp}.txt', gt=True, eulerindex=2)
print("DataFrame gt:")
print(df_gt)

df_ekf = procesar_pos(f'{carpeta}/ekf_{exp}.txt')
print("DataFrame ekf:")
print(df_ekf)

figheight = 5
linewidth = 4
linewidth_ekf = 3

# %%
ti = df_gt.iloc[0].timestamp
tf = df_gt.iloc[-1].timestamp

fig, ax = plt.subplots(figsize=(figheight,figheight))

ax.plot(df_odom.x, df_odom.y, c='r', lw=linewidth, zorder=-1, label="Odometría")
gt = ax.scatter(df_gt.x, df_gt.y, c=df_gt.timestamp-ti, cmap="winter", s=20, label="Ground Truth")
# ax.plot(df_ekf.x, df_ekf.y, c='k', ls=(0,(4,2,1,2)), lw=linewidth_ekf, label="Odometría + EKF")

ax.set_aspect(1)
ax.legend()

plt.colorbar(gt, label="Tiempo (seg)")
ax.set_ylabel('Y (m)')
ax.set_xlabel('X (m)')
plt.tight_layout()

plt.show()

# %%
fig, ax = plt.subplots(figsize=(figheight*1.5,figheight))

ax.plot(df_odom.timestamp-ti, df_odom.x, lw=linewidth, label='X Odometría')
ax.plot(df_odom.timestamp-ti, df_odom.y, lw=linewidth, label='Y Odometría')
ax.plot(df_gt.timestamp-ti, df_gt.x, lw=linewidth, label='X Ground Truth')
ax.plot(df_gt.timestamp-ti, df_gt.y, lw=linewidth, label='Y Ground Truth')
# ax.plot(df_ekf.timestamp-ti, df_ekf.x, c='k', ls=(0,(8,8)), lw=linewidth_ekf, label='X Odometría + EKF')
# ax.plot(df_ekf.timestamp-ti, df_ekf.y, c='k', ls=(0,(8,8)), lw=linewidth_ekf, label='Y Odometría + EKF')

plt.legend()
plt.ylabel('Distancia (m)')
plt.xlabel('Tiempo (seg)')
plt.tight_layout()
plt.show()

# %%
fig, ax = plt.subplots(figsize=(figheight,figheight), subplot_kw={'projection': 'polar'})

ax.plot(np.unwrap(df_odom.theta), df_odom.timestamp-ti, lw=linewidth, label='Theta Odometría')
ax.plot(np.unwrap(df_gt.theta), df_gt.timestamp-ti, lw=linewidth, label='Theta Ground Truth')
# ax.plot(np.unwrap(df_ekf.theta), df_ekf.timestamp-ti,'k', ls=(0,(4,2,1,2)), lw=linewidth_ekf, label='Theta Odometría + EKF')

plt.legend()
plt.tight_layout()
plt.show()

# %%
fig, ax = plt.subplots(figsize=(figheight*1.5,figheight))

ax.plot(df_odom.timestamp-ti, np.unwrap(df_odom.theta), lw=linewidth, label='Theta Odometría')
ax.plot(df_gt.timestamp-ti, np.unwrap(df_gt.theta), lw=linewidth, label='Theta Ground Truth')
# plt.plot(df_ekf.timestamp-ti, np.unwrap(df_ekf.theta), 'k', ls=(0,(4,2,1,2)), lw=linewidth_ekf, label='Theta Odometría + EKF')

plt.legend()
plt.ylabel('Angulo (theta)')
plt.xlabel('Tiempo (seg)')
plt.tight_layout()
plt.show()
