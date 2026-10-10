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

def ts(roll, pitch):
    """Matriz T_S de la Ec. 2.12. Convierte velocidad angular del cuerpo
    en derivada de angulos de Euler: beta_punto = T_S @ omega (Ec. 2.11 / 5.16).

    Entra omega = [p, q, r] en ejes plataforma (roll, pitch, yaw rate).
    Sale [phi_punto, theta_punto, psi_punto].

    Singular en theta = +-90 grados (gimbal lock). No es problema practico
    para una plataforma, pero es la razon por la que se usan cuaterniones."""
    sin_roll  = np.sin(roll)
    cos_roll  = np.cos(roll)
    tan_pitch = np.tan(pitch)
    sec_pitch = 1.0 / np.cos(pitch)
    return np.array([
        [1.0, sin_roll*tan_pitch,  cos_roll*tan_pitch],
        [0.0, cos_roll,           -sin_roll          ],
        [0.0, sin_roll*sec_pitch,  cos_roll*sec_pitch],
    ])

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

def escalar_y_limitar(x, K, L):
    """Fig. B.1: recta de pendiente K que satura en +-L."""
    return np.clip(K * x, -L, L)

def hp_scale(x, K, L):
    """HP SCALE, Fig. 5.1 y 5.2 (Ec. B.1 y B.2). Los tres ejes."""
    return escalar_y_limitar(x, K, L)

def lp_scale(f_AA, K_lat, K_long, L_lat, L_long):
    """LP SCALE, Fig. 5.2 (Ec. B.4-B.5). El VERTICAL pasa intacto:
    K_3 = 1 fijo, sin limite. Es estructural, no tuning — lo que define
    el tilt es el cociente x/z e y/z."""
    K = np.array([K_lat, 1.0,    K_long])
    L = np.array([L_lat, np.inf, L_long])
    return escalar_y_limitar(f_AA, K, L)