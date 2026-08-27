import numpy as np
from scipy.spatial.transform import Rotation

def gen_anclajes_b(radio_b, delta_b) -> np.ndarray:
    anclajes_b = np.zeros((6,3))
    for k in range(6):
        b_phi_k = (2*np.pi/3) * ((k+1)//2) + (-1)**k * (delta_b/2)
        x = radio_b * np.cos(b_phi_k)
        y = radio_b * np.sin(b_phi_k)
        anclajes_b[k,0] = x
        anclajes_b[k,1] = y
    return anclajes_b

def gen_anclajes_p(radio_p, delta_p) -> np.ndarray:
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

def dibujar_base(anclajes_b: np.ndarray, ax):
    xb = anclajes_b[:,0]
    yb = anclajes_b[:,1]
    zb = anclajes_b[:,2]
    ax.scatter(xb, yb, zb, color='black')
    ax.plot(np.append(xb, xb[0]), np.append(yb, yb[0]), np.append(zb, zb[0]), color='black')

def dibujar_plataforma(plataforma_final: np.ndarray, ax):
    xp = plataforma_final[:,0]
    yp = plataforma_final[:,1]
    zp = plataforma_final[:,2]

    ax.scatter(xp, yp, zp, color='green')
    ax.plot(np.append(xp, xp[0]), np.append(yp, yp[0]), np.append(zp, zp[0]), color='green')

def dibujar_patas(anclajes_b: np.ndarray, plataforma_final: np.ndarray, ax):
    for k in range(6):
        pata_x = [anclajes_b[k,0], plataforma_final[k,0]]
        pata_y = [anclajes_b[k,1], plataforma_final[k,1]]
        pata_z = [anclajes_b[k,2], plataforma_final[k,2]]
        ax.plot(pata_x, pata_y, pata_z, color='red')

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