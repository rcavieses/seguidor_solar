import time
from datetime import datetime

# Si estás en Raspberry Pi, descomenta estas líneas:
# from gpiozero import Servo
# servo_horizontal = Servo(17)   # Pin GPIO 17
# servo_vertical = Servo(18)     # Pin GPIO 18

# Configuración
inicio_dia = 8        # 8:00 a.m.
fin_dia = 18          # 6:00 p.m.
pasos = fin_dia - inicio_dia
angulo_max = 180
incremento = angulo_max / pasos

angulo_x = 0  # Horizontal
angulo_y = 45 # Inclinación inicial (mañana)

# Abrimos archivo de registro
with open("movimientos.txt", "w") as log:
    log.write("Registro de movimientos del seguidor solar\n")
    log.write(f"Inicio: {datetime.now()}\n\n")

    print("Iniciando seguidor solar de dos ejes 🌞")

    for hora in range(inicio_dia, fin_dia + 1):
        # Calculamos ángulos
        angulo_x = (hora - inicio_dia) * incremento
        # La inclinación sube hasta el mediodía y luego baja
        if hora <= (inicio_dia + fin_dia) / 2:
            angulo_y += 10
        else:
            angulo_y -= 10

        # Limitamos los ángulos
        angulo_y = max(20, min(70, angulo_y))

        # Si usas servos reales, descomenta:
        # servo_horizontal.value = (angulo_x / 90) - 1
        # servo_vertical.value = (angulo_y - 45) / 45

        # Mostrar en consola
        print(f"{hora}:00 → X:{angulo_x:.1f}°  Y:{angulo_y:.1f}°")

        # Guardar en archivo
        log.write(f"{hora}:00 → X:{angulo_x:.1f}°  Y:{angulo_y:.1f}°\n")

        # Esperar una hora (en prueba, usar 1 segundo)
        time.sleep(1)  # Reemplazar por 3600 para tiempo real

    print("\nFin del día ☀. Regresando a posición inicial...")
    # servo_horizontal.value = -1
    # servo_vertical.value = 0
    log.write("\nFin del día. Panel regresó a posición inicial.\n")
    log.write(f"Fin: {datetime.now()}\n")

print("Panel en posición 0° 🌙")
