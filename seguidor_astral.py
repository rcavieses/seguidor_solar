import time
from datetime import datetime, timedelta
from astral import LocationInfo
from astral.sun import sun

# ----------------------------
# CONFIGURACIÓN DEL LUGAR
# ----------------------------
# Ejemplo: La Paz, BCS
ciudad = LocationInfo("La Paz", "México", "America/Mazatlan", 24.1426, -110.3128)

# ----------------------------
# CONFIGURACIÓN DEL SEGUIDOR
# ----------------------------
angulo_x = 0   # Este-Oeste
angulo_y = 45  # Inclinación inicial

# Si usarás Raspberry Pi, descomenta:
# from gpiozero import Servo
# servo_horizontal = Servo(17)
# servo_vertical = Servo(18)

# Abrimos registro
log = open("movimientos.txt", "w")
log.write("Registro del seguidor solar con astronomía real\n")
log.write(f"Inicio ejecución: {datetime.now()}\n\n")

print("Calculando horarios solares reales...")

# Obtener horarios solares del día actual
hoy = datetime.now()
datos_solares = sun(ciudad.observer, date=hoy.date(), tzinfo=ciudad.timezone)

amanecer = datos_solares["sunrise"]
atardecer = datos_solares["sunset"]
mediodia = datos_solares["noon"]

print(f"Amanecer: {amanecer}")
print(f"Mediodía solar: {mediodia}")
print(f"Atardecer: {atardecer}\n")

log.write(f"Amanecer: {amanecer}\n")
log.write(f"Mediodía solar: {mediodia}\n")
log.write(f"Atardecer: {atardecer}\n\n")

# Tiempo total de luz
duracion = atardecer - amanecer
horas_luz = int(duracion.total_seconds() // 3600)

incremento_x = 180 / horas_luz  # 0° a 180° de Este a Oeste

print("Iniciando seguimiento solar basado en astronomía 🌞\n")

# ----------------------------
# BUCLE PRINCIPAL
# ----------------------------
hora_actual = amanecer

for i in range(horas_luz + 1):
    # Movimiento horizontal
    angulo_x = i * incremento_x

    # Movimiento vertical (sube hasta el mediodía y luego baja)
    if hora_actual <= mediodia:
        angulo_y += 7
    else:
        angulo_y -= 7

    angulo_y = max(20, min(70, angulo_y))

    # Si usas servos reales, descomenta:
    # servo_horizontal.value = (angulo_x / 90) - 1
    # servo_vertical.value = (angulo_y - 45) / 45

    print(f"{hora_actual.time()} → X:{angulo_x:.1f}°  Y:{angulo_y:.1f}°")
    log.write(f"{hora_actual.time()} → X:{angulo_x:.1f}°  Y:{angulo_y:.1f}°\n")

    # PASA UNA HORA (simulado)
    time.sleep(1)  # usar time.sleep(3600) para tiempo real

    hora_actual += timedelta(hours=1)

# ----------------------------
# FIN DEL DÍA
# ----------------------------
print("\nOcultándose el sol. Regresando a posición inicial.")
log.write("\nAtardecer. Regresando a posición inicial.\n")

# servo_horizontal.value = -1
# servo_vertical.value = 0

log.write(f"Fin: {datetime.now()}\n")
log.close()

print("Listo 🌙 Panel en posición inicial.")
