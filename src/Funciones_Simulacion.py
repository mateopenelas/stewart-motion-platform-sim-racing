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

def dibujar_nariz(plataforma_final: np.ndarray, roll, pitch, yaw, ax, largo=200.0):
    """Flecha azul que marca el 'adelante' de la plataforma.
    Sin esto el hexagono es casi simetrico y no se sabe de que lado se lo mira."""
    centro = plataforma_final.mean(axis=0)
    punta = centro + rotar_plataforma(np.array([[largo, 0.0, 0.0]]), roll, pitch, yaw)[0]
    ax.plot([centro[0], punta[0]],
            [centro[1], punta[1]],
            [centro[2], punta[2]], color='blue', linewidth=3)

def integrar(valor_viejo, tasa, dt):
    valor_nuevo = valor_viejo + dt * tasa
    return valor_nuevo

def hp_orden1(y, v, w, dt):
    """Pasaaltos de 1er orden: s/(s+w).
    v = la salida integrada una vez (el estado)."""
    hp = y - w*v
    v = integrar(v, hp, dt)
    return hp, v

def lp_orden2(y, x, xd, wn, zeta, dt, rate_limit=None):
    """Pasabajos de 2do orden: wn^2/(s^2 + 2*zeta*wn*s + wn^2)."""
    xdd = wn**2 * (y - x) - 2.0*zeta*wn*xd
    xd = integrar(xd, xdd, dt)
    if rate_limit is not None:
        xd = np.clip(xd, -rate_limit, rate_limit)   # R&N Sec. 5.5.2
    x = integrar(x, xd, dt)
    return x, xd

def hp_orden2(y, v, p, wn, zeta, dt):
    """Pasaaltos de 2do orden: s^2/(s^2 + 2*zeta*wn*s + wn^2).
    v = salida integrada 1 vez, p = integrada 2 veces (R&N Ec. 5.50-5.53).
    Son estados: hay que guardarlos entre llamadas."""
    x = y - 2.0*zeta*wn*v - wn**2 * p
    v = integrar(v, x, dt)
    p = integrar(p, v, dt)
    return x, v, p

def ts(pH, qH, rH, roll, pitch):
    phi_H = pH + qH*np.sin(roll)*np.tan(pitch) + rH*np.cos(roll)*np.tan(pitch)
    theta_H = qH*np.cos(roll) - rH*np.sin(roll)
    psi_H = qH*np.sin(roll)*(1/np.cos(pitch)) + rH*np.cos(roll)*(1/np.cos(pitch))
    return phi_H, theta_H, psi_H

def gravedad(roll: float, pitch: float) -> np.ndarray:
    longitudinal = -np.sin(pitch)
    lateral = np.cos(pitch)*np.sin(roll)
    vertical = np.cos(pitch)*np.cos(roll)
    return np.array([lateral, vertical, longitudinal])

def ac_a_plataforma(v):
    """AC [lat, vert, long] -> plataforma [long, lat, vert].
    Signos VERIFICADOS: AC es diestro (x=izquierda, y=arriba, z=adelante),
    asi que es un reordenamiento puro, sin cambios de signo."""
    return np.array([v[2], v[0], v[1]])