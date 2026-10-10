if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    import Funciones_Telemetria as t
    import time
    from Funciones_Simulacion import *

    g = 9.81

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=20, azim=160)

    radio_b = 350
    radio_p = 250

    delta_b = np.radians(10) # Diferencia entre anclajes
    delta_p = np.radians(10) # Diferencia entre anclajes

    anclajes_b = gen_anclajes_b(radio_b, delta_b)
    anclajes_p = gen_anclajes_p(radio_p, delta_p)

    traslacion = np.array([0,0,500])

    fps = 30

    mm = t.abrir_fisicas()
    ultimo_packet_id = -1

    dt = 0
    t1 = time.perf_counter()

    primer_frame = True

    fL   = np.zeros(3)
    fL_d = np.zeros(3)
    tilt = np.zeros(3)

    lav    = np.zeros(3)
    lav_v  = np.zeros(3)
    lav_p  = np.zeros(3)
    lav_v2 = np.zeros(3)

    a_v  = np.zeros(3)
    a_p  = np.zeros(3)
    a_v2 = np.zeros(3)

    roll = 0
    pitch = 0

    a_int1 = np.zeros(3)
    a_int2 = np.zeros(3)

    tilt_rate_max = np.radians(5.8)
    tilt_max = np.radians(20)

    K_tras = np.array([1.0, 1.0, 1.0])
    L_tras = np.array([5.0, 5.0, 5.0])
    K_rot = np.array([1.0, 1.0, 1.0])
    L_rot = np.array([2.5, 2.5, 2.5])
    K_lat = 1.0
    K_long = 1.0
    L_lat = 5.0
    L_long = 5.0

    # archivo = open('log.txt', 'w')
    # archivo.write('accG[0],accG[1],accG[2]\n')

    def actualizar(frame):
        global ultimo_packet_id, dt, t1, roll, pitch
        global fL, fL_d, tilt, lav, lav_v, lav_p, lav_v2
        global a_v, a_p, a_v2, a_int1, a_int2
        fis = t.leer_fisicas(mm)
        if ultimo_packet_id != fis.packetId:
            t_now = time.perf_counter()
            dt = min(t_now - t1, 0.1)
            t1 = t_now
            if primer_frame:
                dt = 0.0
            ultimo_packet_id = fis.packetId
            ax.clear()
            ax.set_xlim(-400, 400)
            ax.set_ylim(-400, 400)
            ax.set_zlim(0, 800)

            w_auto = gravedad(fis.roll, fis.pitch)

            # ------------------ROTATIONAL CHANNEL------------------

            f2 = lp_scale(fis.accG + w_auto, K_lat, K_long, L_lat, L_long)

            if primer_frame:
                fL = f2.copy()

            fL, fL_d = lp_orden2(f2, fL, fL_d, 3.1, 1.4, dt)

            fL_lat, fL_vert, fL_long = fL
            denom_arctan = max(fL_vert, 0.2)

            tilt_roll_obj  =  np.arctan2(fL_lat,  denom_arctan)
            tilt_pitch_obj = -np.arctan2(fL_long * np.cos(tilt_roll_obj), denom_arctan)
            tilt_obj = np.array([tilt_roll_obj, tilt_pitch_obj, 0.0])

            paso = tilt_rate_max * dt
            tilt = tilt + np.clip(tilt_obj - tilt, -paso, paso)
            tilt = np.clip(tilt, -tilt_max, tilt_max)

            w1 = hp_scale(ac_a_plataforma(fis.localAngularVelocity), K_rot, L_rot)

            wH, lav_v, lav_p = hp_orden2(w1, lav_v, lav_p, 1.02, 1.0, dt)

            beta_d = ts(roll, pitch) @ wH   

            beta_dH, lav_v2 = hp_orden1(beta_d, lav_v2, 0.2, dt)

            lav = integrar(lav, beta_dH, dt)

            roll, pitch, yaw = tilt + lav

            plataforma_rotada = rotar_plataforma(anclajes_p, roll, pitch, yaw)

            # ------------------TRASLATIONAL CHANNEL------------------

            f_AA_tras = hp_scale(fis.accG, K_tras, L_tras) + w_auto
            f1 = ac_a_plataforma(f_AA_tras)

            w_sim = ac_a_plataforma(gravedad(roll, pitch))
            a1 = f1 - w_sim

            if primer_frame:
                a_p = a1 / 3.1**2
                primer_frame = False

            aH, a_v, a_p = hp_orden2(a1, a_v, a_p, 3.1, 1.4, dt)

            a2 = rotar_plataforma(aH, roll, pitch, yaw)

            a_SI, a_v2 = hp_orden1(a2, a_v2, 0.2, dt)

            a_SI_mm = a_SI * g * 1000  # de G a mm/s^2

            a_int1 = integrar(a_int1, a_SI_mm, dt)
            a_int2 = integrar(a_int2, a_int1, dt)

            plataforma_final = trasladar_plataforma(plataforma_rotada, traslacion + a_int2)
            # print(f'{fis.heading},{fis.pitch}\n')
            # archivo.write(f'{fis.accG[0]},{fis.accG[1]},{fis.accG[2]}\n')
            dibujar_base(anclajes_b, ax)
            dibujar_plataforma(plataforma_final, ax)
            dibujar_patas(anclajes_b, plataforma_final, ax)
            dibujar_nariz(plataforma_final, roll, pitch, yaw, ax)

    try:
        anim = FuncAnimation(fig, actualizar, frames=None, interval=1000/fps)
        plt.show()
    finally:
        mm.close()
        # archivo.close()