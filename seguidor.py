!pip install pyserial
!pip install pandas
"""
Simulación + PSO para obtener ángulos horarios (6:00-17:00) y modo continuo que envía
ángulos al Arduino por USB/Serial.

- Parámetros (modifica arriba): LAT, PANEL_AREA, PANEL_EFF, TIME_STEP_MIN, etc.
- PSO produce best_angles para N=12 pasos (6..17).
- En bucle infinito interpola entre ángulos horarios según minuto y envía por serial.
- Comando enviado: "<angle>\n" (ángulo en grados, float con 2 decimales).
"""

import numpy as np
import time
import math
from datetime import datetime
import csv
import sys
import os
import serial  # pyserial
import serial.tools.list_ports
import pandas as pd

# ---------------------- CONFIGURACIÓN ----------------------
# Sistema / simulación
LAT = 24.1                 # latitud (grados) - ajusta
DATE_DAY_OF_YEAR = None    # si None usa día actual para PSO; o pone int(1..365)
PANEL_AREA = 1.6           # m^2
PANEL_EFF = 0.18           # eficiencia relativa (0-1)
TIME_STEP_MIN = 60         # minutos por paso para la simulación (PSO)
CONS_MOTOR_KWH_PER_DEG = 0.0003  # kWh por grado movido (simplificado)
ANGLE_MIN = -60
ANGLE_MAX = 60

# PSO params
N_PARTICLES = 50
N_ITERS = 120
N_STEPS = 12    # 6:00..17:00 -> 12 pasos

# Serial / Arduino
SERIAL_PORT = None   # poner "/dev/ttyUSB0" o "COM3" o dejar None para intento automático
BAUDRATE = 115200
SERIAL_TIMEOUT = 2   # segundos
RETRY_SERIAL_SECONDS = 5

# Real-time control
LOOP_DELAY = 1.0     # segundos (bucle infinito)
MAX_STEP_DEG_PER_SEC = 2.0   # máximo grados que se permiten mover por segundo (suavizado)

# Salida
ANGLES_CSV = "angles_optimized.csv"

# ---------------------- UTILIDADES SOLARES (simplificadas) ----------------------
def day_of_year_for_psodata():
    if DATE_DAY_OF_YEAR is not None:
        return int(DATE_DAY_OF_YEAR)
    return datetime.now().timetuple().tm_yday

def solar_elevation(lat_deg, day_of_year, hour_decimal):
    # Modelo simple de declinación y elevación (educativo)
    decl = 23.45 * math.sin(math.radians(360/365 * (284 + day_of_year)))
    h_angle = 15 * (hour_decimal - 12)  # ángulo horario
    sin_elev = (math.sin(math.radians(lat_deg))*math.sin(math.radians(decl)) +
                math.cos(math.radians(lat_deg))*math.cos(math.radians(decl))*math.cos(math.radians(h_angle)))
    # Cap values
    sin_elev = max(-1.0, min(1.0, sin_elev))
    elev = math.degrees(math.asin(sin_elev))
    return elev

def irradiance_incident(elev_deg, panel_angle_deg):
    # Modelo simplificado: DNI constante en día claro
    if elev_deg <= 0:
        return 0.0
    DNI = 900.0  # W/m2 (día claro aproximado, parametrizable)
    theta = math.radians(elev_deg - panel_angle_deg)
    G = max(DNI * math.cos(theta), 0.0)
    return G

# ---------------------- FUNCIÓN OBJETIVO (para PSO) ----------------------
def energy_net_for_angles(angle_vector):
    """
    angle_vector : array-like de tamaño N_STEPS (grados)
    retorna: valor a minimizar (negativo de energía neta)
    """
    day = day_of_year_for_psodata()
    total_energy_kwh = 0.0
    total_cost_kwh = 0.0
    prev_ang = angle_vector[0]
    for i in range(N_STEPS):
        # horario desde 6:00 en adelante
        hour = 6 + i
        elev = solar_elevation(LAT, day, hour)
        G = irradiance_incident(elev, angle_vector[i])
        # energía en este paso (W/m2 * m2 * eficiencia * horas)
        hours = TIME_STEP_MIN / 60.0
        E_kwh = (G * PANEL_AREA * PANEL_EFF * hours) / 1000.0
        total_energy_kwh += E_kwh
        total_cost_kwh += abs(angle_vector[i] - prev_ang) * CONS_MOTOR_KWH_PER_DEG
        prev_ang = angle_vector[i]
    # maximizar (energy - cost) → minimizar negativo
    return -(total_energy_kwh - total_cost_kwh)

# ---------------------- PSO (implementación simple) ----------------------
def pso(func, n_particles, n_iters, dim, lb, ub, verbose=False):
    # Inicialización
    pos = np.random.uniform(lb, ub, (n_particles, dim))
    vel = np.zeros((n_particles, dim))
    pbest = pos.copy()
    pbest_val = np.array([func(p) for p in pos])
    gbest_idx = np.argmin(pbest_val)
    gbest = pbest[gbest_idx].copy()
    gbest_val = pbest_val[gbest_idx]

    w = 0.7
    c1 = 1.4
    c2 = 1.4

    for it in range(n_iters):
        r1 = np.random.rand(n_particles, dim)
        r2 = np.random.rand(n_particles, dim)
        vel = w*vel + c1*r1*(pbest - pos) + c2*r2*(gbest - pos)
        pos = pos + vel
        pos = np.clip(pos, lb, ub)

        vals = np.array([func(p) for p in pos])
        improved = vals < pbest_val
        pbest_val[improved] = vals[improved]
        pbest[improved] = pos[improved]

        min_idx = np.argmin(pbest_val)
        if pbest_val[min_idx] < gbest_val:
            gbest_val = pbest_val[min_idx]
            gbest = pbest[min_idx].copy()

        if verbose and (it % 20 == 0 or it == n_iters-1):
            print(f"PSO iter {it+1}/{n_iters} - best_val {gbest_val:.6f}")

    return gbest, gbest_val

# ---------------------- EJECUCIÓN PSO (simulación de un día) ----------------------
def run_pso_and_save():
    lb = np.full(N_STEPS, ANGLE_MIN)
    ub = np.full(N_STEPS, ANGLE_MAX)
    print("Ejecutando PSO para obtener ángulos horarios (esto puede tardar algunos segundos)...")
    best_angles, best_val = pso(energy_net_for_angles, N_PARTICLES, N_ITERS, N_STEPS, lb, ub, verbose=True)
    # Guardar CSV con horas
    hours = [6 + i for i in range(N_STEPS)]
    df = pd.DataFrame({"hour": hours, "angle_deg": np.round(best_angles, 3)})
    df.to_csv(ANGLES_CSV, index=False)
    print(f"Ángulos optimizados guardados en '{ANGLES_CSV}'")
    return best_angles

# ---------------------- SERIAL (auto-detect if needed) ----------------------
def find_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    # Regresa el primer puerto disponible
    return ports[0].device

def open_serial(port, baudrate=115200, timeout=SERIAL_TIMEOUT):
    try:
        ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        print(f"Conectado a serial: {port} @ {baudrate} bps")
        time.sleep(2)  # espera que Arduino resetee (común)
        return ser
    except Exception as e:
        print(f"No se pudo abrir puerto serial {port}: {e}")
        return None

# ---------------------- INTERPOLACIÓN HORARIA Y SUAVIZADO ----------------------
import time
import math

print("Modo simulación. El ángulo ya no será 0.°")
print("Entrando en bucle infinito. Ctrl+C para detener.\n")

while True:
    # Tiempo en segundos
    t = time.time()
    
    # Simulación del sol usando una función suave
    # Esto genera un ángulo entre 10° y 170° que varía continuamente
    target_angle = 90 + 80 * math.sin(t / 30)

    # Aseguramos que esté dentro del rango permitido
    target_angle = max(0, min(180, target_angle))

    # Impresión estilo Arduino
    hora = time.strftime("%H:%M:%S")
    print(f"Sim -> {hora} | target={target_angle:.2f}°")

    # Actualiza cada 0.5 segundos
    time.sleep(0.5)
    

# ---------------------- BUCLE PRINCIPAL / MODO INFINITO ----------------------
def main():
    # 1) PSO y generación tabla (puede usar día actual)
    angles = run_pso_and_save()

    # 2) Abrir serial
    global SERIAL_PORT
    if SERIAL_PORT is None:
        detected = find_serial_port()
        if detected:
            SERIAL_PORT = detected
            print(f"Puerto serial detectado automáticamente: {SERIAL_PORT}")
        else:
            print("No se detectó puerto serial automáticamente. Establece SERIAL_PORT manualmente.")
    ser = None
    if SERIAL_PORT is not None:
        ser = open_serial(SERIAL_PORT, BAUDRATE)
    else:
        print("Modo: impresión en pantalla (no serial). Cambia SERIAL_PORT si quieres conectar Arduino.")

    prev_angle = 0.0
    print("\nEntrando en bucle infinito. Ctrl+C para detener.\n")
    try:
        while True:
            tgt = target_angle_from_table(angles)
            # limitar cambio por ciclo para suavizado
            max_step = MAX_STEP_DEG_PER_SEC * LOOP_DELAY
            delta = tgt - prev_angle
            if abs(delta) > max_step:
                tgt_cmd = prev_angle + math.copysign(max_step, delta)
            else:
                tgt_cmd = tgt
            # asegurar límites mecánicos
            tgt_cmd = max(ANGLE_MIN, min(ANGLE_MAX, tgt_cmd))
            # comando como string: "12.34\n"
            cmd_str = f"{tgt_cmd:.2f}\n"
            if ser is not None and ser.is_open:
                try:
                    ser.write(cmd_str.encode('utf-8'))
                    # opcional: leer eco o ACK
                    # ack = ser.readline().decode().strip()
                    # print("ACK:", ack)
                except Exception as e:
                    print("Error enviando a serial:", e)
                    ser.close()
                    ser = None
            else:
                # intentar reconectar periódicamente
                if SERIAL_PORT is not None:
                    ser = open_serial(SERIAL_PORT, BAUDRATE)
                else:
                    # si no hay puerto especificado, imprimimos
                    print(f"Sim -> {datetime.now().strftime('%H:%M:%S')} | target={tgt_cmd:.2f}°")
            prev_angle = tgt_cmd
            time.sleep(LOOP_DELAY)
    except KeyboardInterrupt:
        print("\nDetenido por usuario (KeyboardInterrupt).")
    finally:
        if ser is not None and ser.is_open:
            ser.close()
            print("Puerto serial cerrado.")

if _name_ == "_main_":
    main()
