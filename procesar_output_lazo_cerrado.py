import numpy as np
import quaternion
import pandas as pd
import yaml
import matplotlib.pyplot as plt

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


def procesar_path(ruta_archivo):
    """
    """
    
    # ruta_archivo = "volume/outputs/lazo_cerrado/goal_path.txt"
    # Leer el archivo
    with open(ruta_archivo, 'r') as file:
        contenido = file.read()
    
    # Separar por los '---' y filtrar documentos vacíos
    documentos = [doc.strip() for doc in contenido.split('---') if doc.strip()]
    poses = yaml.safe_load(documentos[0])['poses']
    
    # Procesar cada documento y crear lista de diccionarios
    datos = []
    for pose in poses[:-1]:
        # Calcular timestamp en segundos
        # timestamp = data['header']['stamp']['sec'] + data['header']['stamp']['nanosec'] / 1e9
        orientation = pose['pose']['orientation']
        def yaw_from_ros_quat(o):
            x, y, z, w = o['x'], o['y'], o['z'], o['w']
            return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        
        datos.append({
            'x': pose['pose']['position']['x'],
            'y': pose['pose']['position']['y'],
            # 'q': quaternion.as_euler_angles(q),
            'theta': yaw_from_ros_quat(orientation)
        })
    
    # Crear DataFrame
    df = pd.DataFrame(datos)

    return df


# %%
carpeta = "volume/outputs/lazo_cerrado"

df_gt = []

for i in range(1,8):
    df_gt.append(procesar_pos(f'{carpeta}/gt_exp{i}.txt', gt=True, eulerindex=2))
    print("DataFrame gt:")
    print(df_gt[-1])

df_goal = procesar_path(f'{carpeta}/goal_path.txt')
print("DataFrame ekf:")
print(df_goal)


figheight = 5
linewidth = 4
linewidth_ekf = 3

# %%

from matplotlib.gridspec import GridSpec

arrows = True

fig = plt.figure(constrained_layout=True,figsize=(1.5*figheight,1*figheight))

gs = GridSpec(2, 4, figure=fig)
ax1 = fig.add_subplot(gs[0, 0:2])
ax2 = fig.add_subplot(gs[0, 2:4])
ax3 = fig.add_subplot(gs[1, 1:3])

ax = [ax1, ax2, ax3]

# fig, ax = plt.subplots(1, 3,figsize=(3*figheight,figheight))

# colors = [(0.3,0.3,.9), (0.2,0.6,.6), (0.3,0.8,.4)]
colors = plt.colormaps["tab10"]
arrow_len = 0.5
arrow_head_w = .1
arrow_head_l = .2

ax[0].plot(df_goal.x, df_goal.y, c='k', lw=1, zorder=-1, label="Goal Path")
if arrows:
    for j in range(0,len(df_goal),10):
        p = df_goal.iloc[j]
        ax[0].arrow(p.x, p.y, arrow_len*np.cos(p.theta), arrow_len*np.sin(p.theta),
                    shape="full", length_includes_head=True,
                    head_length=arrow_head_l, head_width=arrow_head_w,
                    color=(.3,.3,.3))


for n in range(3):
    i = [1,0,2][n]
    k = [.5,.2,.9][n]
    tf = df_gt[i].timestamp.iloc[-1]
    ax[0].plot(df_gt[i].x, df_gt[i].y, lw=1,
               c=colors(n),
               label=f"K_Rho={k}\nTiempo={tf:.0f}seg")
    # gt = ax.scatter(df_gt[i].x, df_gt[i].y, c=df_gt[i].timestamp-ti, cmap="winter", s=20, label=f"GT{i}")
    
    if arrows:
        for j in range(0,len(df_gt[i]),100):
            p = df_gt[i].iloc[j]
            ax[0].arrow(p.x, p.y, arrow_len*np.cos(p.theta), arrow_len*np.sin(p.theta),
                        shape="full", length_includes_head=True,
                        head_length=arrow_head_l, head_width=arrow_head_w, color=colors(n))

ax[0].set_aspect(1)
ax[0].legend(loc='center left', bbox_to_anchor=(1, 0.5))
ax[0].set_title("K_Alpha = 1; Lookahead = 1m")
ax[0].set_ylabel('Y (m)')
ax[0].set_xlabel('X (m)')


ax[1].plot(df_goal.x, df_goal.y, c='k', lw=1, zorder=-1, label="Goal Path")
if arrows:
    for j in range(0,len(df_goal),10):
        p = df_goal.iloc[j]
        ax[0].arrow(p.x, p.y, arrow_len*np.cos(p.theta), arrow_len*np.sin(p.theta),
                    shape="full", length_includes_head=True,
                    head_length=arrow_head_l, head_width=arrow_head_w,
                    color=(.3,.3,.3))

for n in range(3):
    i = [3,0,4][n]
    k = [.6,1,1.4][n]
    tf = df_gt[i].timestamp.iloc[-1]
    ax[1].plot(df_gt[i].x, df_gt[i].y, lw=1,
               c=colors(n),
               label=f"K_Alpha={k}\nTiempo={tf:.0f}seg")

    if arrows:
        for j in range(0,len(df_gt[i]),100):
            p = df_gt[i].iloc[j]
            ax[1].arrow(p.x, p.y, arrow_len*np.cos(p.theta), arrow_len*np.sin(p.theta),
                        shape="full", length_includes_head=True,
                        head_length=arrow_head_l, head_width=arrow_head_w, color=colors(n))


ax[1].set_aspect(1)
ax[1].legend(loc='center left', bbox_to_anchor=(1, 0.5))
ax[1].set_title("K_Rho = .5; Lookahead = 1m")
# ax[1].set_ylabel('Y (m)')
ax[1].set_xlabel('X (m)')


ax[2].plot(df_goal.x, df_goal.y, c='k', lw=1, zorder=-1, label="Goal Path")
if arrows:
    for j in range(0,len(df_goal),10):
        p = df_goal.iloc[j]
        ax[0].arrow(p.x, p.y, arrow_len*np.cos(p.theta), arrow_len*np.sin(p.theta),
                    shape="full", length_includes_head=True,
                    head_length=arrow_head_l, head_width=arrow_head_w,
                    color=(.3,.3,.3))

for n in range(3):
    i = [6,0,5][n]
    k = [.2,.5,1][n]
    tf = df_gt[i].timestamp.iloc[-1]
    ax[2].plot(df_gt[i].x, df_gt[i].y, lw=1,
               c=colors(n),
               label=f"Lookahead={k}\nTiempo={tf:.0f}seg")

    if arrows:
        for j in range(0,len(df_gt[i]),100):
            p = df_gt[i].iloc[j]
            ax[2].arrow(p.x, p.y, arrow_len*np.cos(p.theta), arrow_len*np.sin(p.theta),
                        shape="full", length_includes_head=True,
                        head_length=arrow_head_l, head_width=arrow_head_w, color=colors(n))

ax[2].set_aspect(1)
ax[2].legend(loc='center left', bbox_to_anchor=(1, 0.5))
ax[2].set_title("K_Alpha = 1; K_Rho = .5")
# ax[2].set_ylabel('Y (m)')
ax[2].set_xlabel('X (m)')


plt.tight_layout()
plt.show()
