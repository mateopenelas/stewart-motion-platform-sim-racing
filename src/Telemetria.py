if __name__ == "__main__":
    import Funciones_Telemetria as f
    import time
    
    mm = f.abrir_fisicas()
    
    ultimo_packet_id = -1
    
    try:
        while True:
            fis = f.leer_fisicas(mm)
            
            if fis.packetId != ultimo_packet_id: # Solo procesar si el frame es nuevo
                ultimo_packet_id = fis.packetId
                
                print(
                    # f"Speed: {fis.speedKmh:6.1f} km/h | "
                    # f"Pitch: {fis.pitch:7.4f} rad | "
                    # f"Roll:  {fis.roll:7.4f} rad | "
                    # f"G-lat: {fis.accG[0]:6.3f} G | "
                    # f"G-lon: {fis.accG[2]:6.3f} G",
                    '\n\n\n\n\n\n\n\n'
                    # f'Gas: {fis.gas:.2f}',
                    # f'Brake: {fis.brake:.2f}',
                    f'Pitch: {fis.pitch:.2f}',
                    f'Roll: {fis.roll:.2f}',
                    f'Yaw: {fis.heading:.2f}',
                    end='\r'
                )
            
            time.sleep(0.01)  # polling a 100Hz
    
    except KeyboardInterrupt:
        print("\nSaliendo...")
    
    finally:
        mm.close()