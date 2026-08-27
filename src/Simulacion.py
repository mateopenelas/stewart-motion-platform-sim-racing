if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    import numpy as np
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

    # archivo = open('log.txt', 'w')
    # archivo.write('steerAngle,gas,brake,speedKmh,localAngularVelocity[0],localAngularVelocity[1],localAngularVelocity[2]\n')

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

            # archivo.write(f'{fis.steerAngle},{fis.gas},{fis.brake},{fis.speedKmh},{fis.localAngularVelocity[0]},{fis.localAngularVelocity[1]},{fis.localAngularVelocity[2]}\n')
            roll = -fL_lat/g + lav_roll
            pitch = fL_long/g + lav_pitch
            yaw = lav_yaw
            plataforma_rotada = rotar_plataforma(anclajes_p, roll, pitch, yaw)
            plataforma_final = trasladar_plataforma(plataforma_rotada, traslacion)
            dibujar_base(anclajes_b, ax)
            dibujar_plataforma(plataforma_final, ax)
            dibujar_patas(anclajes_b, plataforma_final, ax)



    try:
        anim = FuncAnimation(fig, actualizar, frames=None, interval=1000/fps)
        plt.show()
    finally:
        mm.close()
        # archivo.close()