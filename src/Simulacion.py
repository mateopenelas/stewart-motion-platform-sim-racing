if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    import Funciones_Telemetria as t
    import time
    from Funciones_Simulacion import *

    g = 9.81

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

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

    g_max = 5
    lav_max = 2.5

    dt = 0
    t1 = time.perf_counter()
    
    fL_lat = fL_lat_d = 0.0
    fL_long = fL_long_d = 0.0

    lav_roll = 0
    lav_pitch = 0
    lav_yaw = 0

    lav_roll_v  = lav_roll_p  = 0.0
    lav_pitch_v = lav_pitch_p = 0.0
    lav_yaw_v   = lav_yaw_p   = 0.0

    lav_roll_v2 = lav_pitch_v2 = lav_yaw_v2 = 0.0

    a_v  = np.zeros(3)
    a_p  = np.zeros(3)
    a_v2 = np.zeros(3)

    roll = 0
    pitch = 0

    a_int1 = np.zeros(3)
    a_int2 = np.zeros(3)


    # archivo = open('log.txt', 'w')
    # archivo.write('accG[0],accG[1],accG[2]\n')

    def actualizar(frame):
        global ultimo_packet_id, dt, t1, roll, pitch
        global fL_lat, fL_lat_d, fL_long, fL_long_d
        global lav_roll, lav_pitch, lav_yaw
        global lav_roll_v, lav_roll_p, lav_pitch_v, lav_pitch_p, lav_yaw_v, lav_yaw_p
        global lav_roll_v2, lav_pitch_v2, lav_yaw_v2
        global a_v, a_p, a_v2, a_int1, a_int2
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

            accG_clip = np.clip(fis.accG, -g_max, g_max)
            f_AA = accG_clip + gravedad(fis.roll, fis.pitch) # Fuerza especifica

            # ------------------ROTATIONAL CHANNEL------------------

            fL_lat, fL_lat_d = lp_orden2(f_AA[0], fL_lat, fL_lat_d, 3.1, 1.4, dt, 0.102)
            fL_long, fL_long_d = lp_orden2(f_AA[2], fL_long, fL_long_d, 3.1, 1.4, dt, 0.102)

            lav_roll_h1, lav_roll_v, lav_roll_p = hp_orden2(np.clip(fis.localAngularVelocity[2], -lav_max, lav_max), lav_roll_v, lav_roll_p, 1.02, 1.0, dt)
            lav_pitch_h1, lav_pitch_v, lav_pitch_p = hp_orden2(np.clip(fis.localAngularVelocity[0], -lav_max, lav_max), lav_pitch_v, lav_pitch_p, 1.02, 1.0, dt)
            lav_yaw_h1, lav_yaw_v, lav_yaw_p = hp_orden2(np.clip(fis.localAngularVelocity[1], -lav_max, lav_max), lav_yaw_v, lav_yaw_p, 1.02, 1.0, dt)

            roll_ts, pitch_ts, yaw_ts = ts(lav_roll_h1, lav_pitch_h1, lav_yaw_h1, roll, pitch)

            lav_roll_h2, lav_roll_v2 = hp_orden1(roll_ts, lav_roll_v2, 0.2, dt)
            lav_pitch_h2, lav_pitch_v2 = hp_orden1(pitch_ts, lav_pitch_v2, 0.2, dt)
            lav_yaw_h2, lav_yaw_v2 = hp_orden1(yaw_ts, lav_yaw_v2, 0.2, dt)

            lav_roll = integrar(lav_roll, lav_roll_h2, dt)
            lav_pitch = integrar(lav_pitch, lav_pitch_h2, dt)
            lav_yaw = integrar(lav_yaw, lav_yaw_h2, dt)

            roll = -fL_lat/g + lav_roll
            pitch = fL_long/g + lav_pitch
            yaw = lav_yaw

            plataforma_rotada = rotar_plataforma(anclajes_p, roll, pitch, yaw)

            # ------------------TRASLATIONAL CHANNEL------------------

            f1 = ac_a_plataforma(f_AA)
            w_sim = ac_a_plataforma(gravedad(roll, pitch))
            a1 = f1 - w_sim

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



    try:
        anim = FuncAnimation(fig, actualizar, frames=None, interval=1000/fps)
        plt.show()
    finally:
        mm.close()
        # archivo.close()