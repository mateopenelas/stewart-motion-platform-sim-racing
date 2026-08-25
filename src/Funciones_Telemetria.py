import mmap
import ctypes

i = ctypes.c_int32
f = ctypes.c_float

class fisicas(ctypes.Structure):
    _pack_ = 4  # Alinea los bytes de a 4 (pragma #4)
    
    _fields_ = [
        # ── Identificación ──────────────────────────────────────────────────
        ('packetId',              i),      # contador que sube en cada update
        
        # ── Controles ───────────────────────────────────────────────────────
        ('gas',                   f),      # acelerador 0.0 a 1.0
        ('brake',                 f),      # freno 0.0 a 1.0
        ('fuel',                  f),      # combustible en kg
        ('gear',                  i),      # marcha (0=reversa, 1=neutro, 2+=)
        ('rpms',                  i),      # RPM del motor
        ('steerAngle',            f),      # ángulo del volante -1.0 a 1.0
        ('speedKmh',              f),      # velocidad en km/h
        
        # ── Vectores de movimiento ───────────────────────────────────────────
        ('velocity',              f * 3), # velocidad global [x, y, z] m/s
        ('accG',                  f * 3), # fuerzas G [lat, vert, long] en G
        
        # ── Neumáticos (siempre en orden FL, FR, RL, RR) ────────────────────
        ('wheelSlip',             f * 4), # deslizamiento de cada rueda
        ('wheelLoad',             f * 4), # carga vertical en cada rueda
        ('wheelsPressure',        f * 4), # presión de cada neumático
        ('wheelAngularSpeed',     f * 4), # velocidad angular rad/s
        ('tyreWear',              f * 4), # desgaste 0.0 a 1.0
        ('tyreDirtyLevel',        f * 4), # suciedad acumulada
        ('tyreCoreTemperature',   f * 4), # temperatura núcleo en °C
        ('camberRAD',             f * 4), # camber en radianes
        ('suspensionTravel',      f * 4), # recorrido de suspensión en m
        
        # ── Sistemas del auto ────────────────────────────────────────────────
        ('drs',                   f),     # DRS activo
        ('tc',                    f),     # control de tracción activo
        
        # ── Orientación del chasis ───────────────────────────────────────────
        # Estos tres son los más importantes para la plataforma
        ('heading',               f),     # yaw  (rotación horizontal) en rad
        ('pitch',                 f),     # pitch (adelante/atrás)     en rad
        ('roll',                  f),     # roll  (inclinación lateral) en rad
        ('cgHeight',              f),     # altura del centro de gravedad en m
        
        # ── Daños ───────────────────────────────────────────────────────────
        ('carDamage',             f * 5), # frente, atrás, izq, der, centro
        ('numberOfTyresOut',      i),     # ruedas fuera de pista
        ('pitLimiterOn',          i),     # limitador de velocidad activo
        ('abs',                   f),     # ABS activo
        
        # ── Sistemas KERS/ERS (irrelevantes para la mayoría de autos) ────────
        ('kersCharge',            f),
        ('kersInput',             f),
        ('autoShifterOn',         i),
        ('rideHeight',            f * 2), # altura de carrocería [delantera, trasera]
        ('turboBoost',            f),
        ('ballast',               f),
        ('airDensity',            f),
        ('airTemp',               f),
        ('roadTemp',              f),
        
        # ── Velocidad angular local ──────────────────────────────────────────
        # Importantísimo para la plataforma: roll rate, pitch rate, yaw rate
        ('localAngularVelocity',  f * 3),
        
        ('finalFF',               f),     # señal final de force feedback
        ('performanceMeter',      f),
        
        # ── Más sistemas ERS ────────────────────────────────────────────────
        ('engineBrake',           i),
        ('ersRecoveryLevel',      i),
        ('ersPowerLevel',         i),
        ('ersHeatCharging',       i),
        ('ersIsCharging',         i),
        ('kersCurrentKJ',         f),
        ('drsAvailable',          i),
        ('drsEnabled',            i),
        
        # ── Temperaturas de frenos y embrague ────────────────────────────────
        ('brakeTemp',             f * 4),
        ('clutch',                f),
        ('tyreTempI',             f * 4), # temperatura interior del neumático
        ('tyreTempM',             f * 4), # temperatura media
        ('tyreTempO',             f * 4), # temperatura exterior
        
        # ── Control IA ──────────────────────────────────────────────────────
        ('isAIControlled',        i),
        
        # ── Contacto de neumáticos ───────────────────────────────────────────
        # Array 2D: 4 ruedas × 3 coordenadas [x, y, z]
        ('tyreContactPoint',      (f * 3) * 4),
        ('tyreContactNormal',     (f * 3) * 4),
        ('tyreContactHeading',    (f * 3) * 4),
        
        # ── Balance de frenos y velocidad local ─────────────────────────────
        ('brakeBias',             f),
        ('localVelocity',         f * 3), # velocidad en ejes del auto [x, y, z]
    ]

def abrir_fisicas():
    size = ctypes.sizeof(fisicas)
    try:
        m = mmap.mmap(-1, size, tagname="Local\\acpmf_physics", access=mmap.ACCESS_READ)
        return m
    except Exception as e:
        raise FileNotFoundError(
            "No se pudo abrir la shared memory de AC.\n"
            "¿Está corriendo Assetto Corsa con una sesión activa?\n"
            f"Error original: {e}"
        )

def leer_fisicas(m):
    size = ctypes.sizeof(fisicas)
    m.seek(0)
    buffer = m.read(size)
    return fisicas.from_buffer_copy(buffer)