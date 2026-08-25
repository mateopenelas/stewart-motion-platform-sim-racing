import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from scipy.spatial.transform import Rotation
import Funciones_Telemetria as t
import time

g = 9.81

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

radio_b = 350
radio_p = 250

delta_b = np.radians(10) # Diferencia entre anclajes
delta_p = np.radians(10) # Diferencia entre anclajes

def gen_anclajes_b() -> np.ndarray:
    anclajes_b = np.zeros((6,3))
    for k in range(6):
        b_phi_k = (2*np.pi/3) * ((k+1)//2) + (-1)**k * (delta_b/2)
        x = radio_b * np.cos(b_phi_k)
        y = radio_b * np.sin(b_phi_k)
        anclajes_b[k,0] = x
        anclajes_b[k,1] = y
    return anclajes_b

def gen_anclajes_p() -> np.ndarray:
    anclajes_p = np.zeros((6,3))
    for k in range(6):
        p_phi_k = (2*np.pi/3) * (k//2) - (-1)**k * (delta_p/2) + np.pi/3
        x = radio_p * np.cos(p_phi_k)
        y = radio_p * np.sin(p_phi_k)
        anclajes_p[k,0] = x
        anclajes_p[k,1] = y
    return anclajes_p

def rotar_plataforma(anclajes_p: np.ndarray, roll: float, pitch: float, yaw: float) -> np.ndarray:
    rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
    plataforma_rotada = rot.apply(anclajes_p)
    return plataforma_rotada

def trasladar_plataforma(plataforma_rotada: np.ndarray, traslacion: np.ndarray):
    return plataforma_rotada + traslacion

def dibujar_base(anclajes_b: np.ndarray):
    xb = anclajes_b[:,0]
    yb = anclajes_b[:,1]
    zb = anclajes_b[:,2]
    ax.scatter(xb, yb, zb, color='black')
    ax.plot(np.append(xb, xb[0]), np.append(yb, yb[0]), np.append(zb, zb[0]), color='black')

def dibujar_plataforma(plataforma_final: np.ndarray):
    xp = plataforma_final[:,0]
    yp = plataforma_final[:,1]
    zp = plataforma_final[:,2]

    ax.scatter(xp, yp, zp, color='green')
    ax.plot(np.append(xp, xp[0]), np.append(yp, yp[0]), np.append(zp, zp[0]), color='green')

def dibujar_patas(anclajes_b: np.ndarray, plataforma_final: np.ndarray):
    for k in range(6):
        pata_x = [anclajes_b[k,0], plataforma_final[k,0]]
        pata_y = [anclajes_b[k,1], plataforma_final[k,1]]
        pata_z = [anclajes_b[k,2], plataforma_final[k,2]]
        ax.plot(pata_x, pata_y, pata_z, color='red')

anclajes_b = gen_anclajes_b()
anclajes_p = gen_anclajes_p()

traslacion = np.array([0,0,500])

fps = 144

mm = t.abrir_fisicas()
ultimo_packet_id = -1

a_0 = 3 # Frecuencia de corte del filtro

dt = 0
t1 = time.perf_counter()

fL_lat = 0
fL_long = 0

g_max = 5

archivo = open('log.txt', 'w')
archivo.write('fL_lat,fL_long,accG[0],accG[2],dt\n')

def low_pass_filter(y, x_viejo, a_0, dt):
    x_nuevo = x_viejo + dt * a_0 * (y - x_viejo)
    return x_nuevo

def actualizar(frame):
    global ultimo_packet_id, fL_lat, fL_long, dt, t1
    fis = t.leer_fisicas(mm)
    if ultimo_packet_id != fis.packetId:
        t_now = time.perf_counter()
        dt = min(t_now - t1, 0.1)
        t1 = t_now
        ultimo_packet_id = fis.packetId
        ax.clear()
        ax.set_xlim(-400, 400)
        ax.set_ylim(-400, 400)
        ax.set_zlim(0, 800)

        fL_lat = low_pass_filter(np.clip(fis.accG[0], -g_max, g_max), fL_lat, a_0, dt)
        fL_long = low_pass_filter(np.clip(fis.accG[2], -g_max, g_max), fL_long, a_0, dt)

        print(fL_lat, fL_long)
        archivo.write(f'{fL_lat},{fL_long},{fis.accG[0]},{fis.accG[2]},{dt}\n')
        roll = -fL_lat/g
        pitch = fL_long/g
        yaw = 0
        plataforma_rotada = rotar_plataforma(anclajes_p, roll, pitch, yaw)
        plataforma_final = trasladar_plataforma(plataforma_rotada, traslacion)
        dibujar_base(anclajes_b)
        dibujar_plataforma(plataforma_final)
        dibujar_patas(anclajes_b, plataforma_final)



try:
    anim = FuncAnimation(fig, actualizar, frames=None, interval=1000/fps)
    plt.show()
finally:
    mm.close()