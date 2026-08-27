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

lav_roll = 0
lav_pitch = 0
lav_yaw = 0

lav_roll_l1 = 0
lav_pitch_l1 = 0
lav_yaw_l1 = 0

lav_roll_l2 = 0
lav_pitch_l2 = 0
lav_yaw_l2 = 0

g_max = 5
lav_max = 2.5

roll = 0
pitch = 0

archivo = open('log.txt', 'w')
archivo.write('steerAngle,gas,brake,speedKmh,localAngularVelocity[0],localAngularVelocity[1],localAngularVelocity[2]\n')

def integrar(valor_viejo, tasa, dt):
    valor_nuevo = valor_viejo + dt * tasa
    return valor_nuevo

def low_pass_filter(y, x_viejo, a_0, dt):
    x_nuevo = integrar(x_viejo, a_0 * (y - x_viejo), dt) 
    return x_nuevo

def high_pass_filter(y, x_lp_viejo, a_0, dt):
    lp_nuevo = low_pass_filter(y, x_lp_viejo, a_0, dt)
    x_nuevo = y - lp_nuevo
    return x_nuevo, lp_nuevo

def tsx(pH, qH, rH, roll, pitch):
    phi_H = pH + qH*np.sin(roll)*np.tan(pitch) + rH*np.cos(roll)*np.tan(pitch)
    theta_H = qH*np.cos(roll) - rH*np.sin(roll)
    psi_H = qH*np.sin(roll)*(1/np.cos(pitch)) + rH*np.cos(roll)*(1/np.cos(pitch))
    return phi_H, theta_H, psi_H

def actualizar(frame):
    global ultimo_packet_id, fL_lat, fL_long, dt, t1, lav_roll, lav_pitch, lav_yaw, roll, pitch, lav_roll_l1, lav_pitch_l1, lav_yaw_l1, lav_roll_l2, lav_pitch_l2, lav_yaw_l2
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

        lav_roll_h1, lav_roll_l1 = high_pass_filter(np.clip(fis.localAngularVelocity[2], -lav_max, lav_max), lav_roll_l1, a_0, dt)
        lav_pitch_h1, lav_pitch_l1 = high_pass_filter(np.clip(fis.localAngularVelocity[0], -lav_max, lav_max), lav_pitch_l1, a_0, dt)
        lav_yaw_h1, lav_yaw_l1 = high_pass_filter(np.clip(fis.localAngularVelocity[1], -lav_max, lav_max), lav_yaw_l1, a_0, dt)

        roll_tsx, pitch_tsx, yaw_tsx = tsx(lav_roll_h1, lav_pitch_h1, lav_yaw_h1, roll, pitch)

        lav_roll_h2, lav_roll_l2 = high_pass_filter(roll_tsx, lav_roll_l2, a_0, dt)
        lav_pitch_h2, lav_pitch_l2 = high_pass_filter(pitch_tsx, lav_pitch_l2, a_0, dt)
        lav_yaw_h2, lav_yaw_l2 = high_pass_filter(yaw_tsx, lav_yaw_l2, a_0, dt)

        lav_roll = integrar(lav_roll, lav_roll_h2, dt)
        lav_pitch = integrar(lav_pitch, lav_pitch_h2, dt)
        lav_yaw = integrar(lav_yaw, lav_yaw_h2, dt)

        archivo.write(f'{fis.steerAngle},{fis.gas},{fis.brake},{fis.speedKmh},{fis.localAngularVelocity[0]},{fis.localAngularVelocity[1]},{fis.localAngularVelocity[2]}\n')
        roll = -fL_lat/g + lav_roll
        pitch = fL_long/g + lav_pitch
        yaw = lav_yaw
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
    archivo.close()