import re
import serial
import struct
import time
import numpy as np

# =========================================================================
# 1. CONFIGURACIÓN DE PUERTOS SERIALES Y PROTOCOLO HIWONDER HTD-35H
# =========================================================================
PUERTO_SERVO = '/dev/ttyTHS1'  # Pines UART GPIO hacia el servo HTD-35H
PUERTO_ESP32 = '/dev/ttyUSB0'  # USB serial conectado al ESP32 (Keypad + LCD)
BAUDIOS      = 115200
SERVO_ID     = 4               # Verifica si tu servo tiene ID 4 o ID 1 (fábrica)

# Geometría del brazo para convertir gramos -> inercia (kg*m^2)
L_BRAZO      = 0.15   # Longitud del eje a la punta en metros (ajusta a tu brazo real)
F_EN_NEWTONS = False  # False: F entra en N*m | True: F entra en Newtons (tau = F * L_BRAZO)

CMD_SERVO_MOVE_TIME_WRITE     = 1
CMD_SERVO_VIN_READ            = 27
CMD_SERVO_POS_READ            = 28
CMD_SERVO_OR_MOTOR_MODE_WRITE = 29

# Parámetros físicos del HTD-35H (Datasheet) y eslabón
m, lc, g, I = 0.35, 0.12, 9.81, 0.0055
V_max       = 11.1
I_stall     = 3.0
I_noload    = 0.10
tau_stall   = 35.0 * 0.0980665
w_noload    = np.deg2rad(60) / 0.18

Ra            = V_max / I_stall                      # 3.70 Ohmios
Kt            = tau_stall / I_stall                  # 1.1441 N*m/A
Ke            = (V_max - I_noload * Ra) / w_noload   # 1.8444 V*s/rad
b             = (Kt * I_noload) / w_noload           # 0.0197 N*m*s/rad
res_encoder   = np.deg2rad(240.0) / 1000.0           # 0.24°/cuenta
CUENTA_CENTRO = 250

# Parámetros de respaldo (solo si en el ESP32 se deja M=0 o K=0 por error)
M_GRAMOS_RESPALDO = 250.0
Kd_RESPALDO       = 0.60

Kp_v = 20.0   # Ganancia proporcional de seguimiento (V/rad)
Kd_v = 2.4    # Ganancia derivativa interna (V*s/rad)
dt   = 0.0065 # Periodo de muestreo (~154 Hz)


def checksum_hiwonder(datos):
    return (~sum(datos)) & 0xFF


def comando_pwm_cmd29(ser, servo_id, modo_motor, pwm):
    pwm = int(np.clip(round(pwm), -1000, 1000))
    pwm_l, pwm_h = struct.pack('<h', pwm)
    cuerpo = [servo_id, 7, CMD_SERVO_OR_MOTOR_MODE_WRITE, modo_motor, 0, pwm_l, pwm_h]
    ser.write(bytearray([0x55, 0x55] + cuerpo + [checksum_hiwonder(cuerpo)]))


def mover_servo_cmd1(ser, servo_id, cuentas, tiempo_ms):
    cuentas = int(np.clip(round(cuentas), 50, 950))
    pos_l, pos_h   = cuentas & 0xFF, (cuentas >> 8) & 0xFF
    time_l, time_h = tiempo_ms & 0xFF, (tiempo_ms >> 8) & 0xFF
    cuerpo = [servo_id, 7, CMD_SERVO_MOVE_TIME_WRITE, pos_l, pos_h, time_l, time_h]
    ser.write(bytearray([0x55, 0x55] + cuerpo + [checksum_hiwonder(cuerpo)]))


def solicitar_lectura(ser, servo_id, cmd, signed=False):
    ser.reset_input_buffer()
    cuerpo = [servo_id, 3, cmd]
    ser.write(bytearray([0x55, 0x55] + cuerpo + [checksum_hiwonder(cuerpo)]))
    raw = ser.read(14)
    if len(raw) < 8:
        return None
    for i in range(len(raw) - 7):
        if (raw[i] == 0x55 and raw[i+1] == 0x55 and
                raw[i+2] == servo_id and raw[i+3] == 5 and raw[i+4] == cmd):
            if checksum_hiwonder(raw[i+2:i+7]) == raw[i+7]:
                return struct.unpack('<h' if signed else '<H', raw[i+5:i+7])[0]
    return None


def expm_exacta(M, orden=18):
    norma = np.linalg.norm(M, ord=np.inf)
    s = max(0, int(np.ceil(np.log2(norma)))) if norma > 0 else 0
    M_s = M / (2.0 ** s)
    res = np.eye(M.shape[0], dtype=float)
    termino = np.eye(M.shape[0], dtype=float)
    for k in range(1, orden + 1):
        termino = (termino @ M_s) / float(k)
        res += termino
    for _ in range(s):
        res = res @ res
    return res


def construir_matrices_impedancia(Md_val, Kd_val, Bd_val, dt_val):
    """Discretiza exactamente (ZOH) el sistema virtual Masa-Resorte-Amortiguador."""
    M_e = np.zeros((3, 3))
    M_e[0:2, 0:2] = np.array([
        [0.0,              1.0],
        [-Kd_val / Md_val, -Bd_val / Md_val]
    ])
    M_e[0:2, 2:3] = np.array([[0.0], [1.0 / Md_val]])
    E_e = expm_exacta(M_e * dt_val)
    return E_e[0:2, 0:2], E_e[0:2, 2:3]


def esperar_confirmacion_esp32(ser_esp, estado_esp):
    """
    Espera a que el ESP32 envíe K, B, M (en gramos), F y el mensaje de confirmación.
    Convierte M de gramos a momento de inercia rotacional (kg*m^2).
    """
    patron_var = re.compile(r'^([KBMF])\s+guardado:\s*([-+]?\d*\.?\d+)', re.IGNORECASE)
    unidad_f = "N" if F_EN_NEWTONS else "N*m"

    print("\n" + "=" * 96)
    print(f"  [ESPERANDO ESP32 EN {PUERTO_ESP32}]")
    print(f"  Ingresa K (N*m/rad), B (N*m*s/rad), M (GRAMOS) y F ({unidad_f}) y confirma con 'D'.")
    print("=" * 96)

    while True:
        linea_bytes = ser_esp.readline()
        if not linea_bytes:
            continue

        linea = linea_bytes.decode('utf-8', errors='ignore').strip()
        if not linea:
            continue

        print(f"  [ESP32 RX] {linea}")

        match = patron_var.match(linea)
        if match:
            clave = match.group(1).upper()
            valor = float(match.group(2))
            estado_esp[clave] = valor
            sufijo = " g" if clave == 'M' else ""
            print(f"     -> Registrado {clave} = {valor:.4f}{sufijo}")
            continue

        if "confirmados" in linea.lower():
            kd_raw   = estado_esp['K']
            bd_raw   = estado_esp['B']
            m_gramos = estado_esp['M']
            f_raw    = estado_esp['F']

            # Conversión de M: gramos -> kg -> inercia rotacional Md = m * L^2
            if m_gramos <= 0.0:
                print(f"  [AVISO] M={m_gramos}g debe ser > 0. Usando respaldo M={M_GRAMOS_RESPALDO:.1f}g.")
                m_gramos = M_GRAMOS_RESPALDO

            m_kg   = m_gramos / 1000.0
            md_raw = m_kg * (L_BRAZO ** 2)
            md_val = float(np.clip(md_raw, 0.002, 2.0))
            if md_val != md_raw:
                print(f"  [AVISO] Inercia calculada ({md_raw:.5f} kg*m^2) ajustada al rango seguro [0.002, 2.0].")

            if kd_raw <= 0.0:
                print(f"  [AVISO] K={kd_raw} debe ser > 0. Usando K={Kd_RESPALDO} N*m/rad.")
                kd_val = Kd_RESPALDO
            else:
                kd_val = float(np.clip(kd_raw, 0.05, 5.0))

            bd_val = float(np.clip(bd_raw, 0.0, 2.0))

            # Conversión opcional de F si está activado F_EN_NEWTONS
            tau_raw = (f_raw * L_BRAZO) if F_EN_NEWTONS else f_raw
            tau_val = float(np.clip(tau_raw, -1.8, 1.8))

            return tau_val, kd_val, bd_val, md_val, m_gramos, f_raw


def imprimir_resumen_parametros(tau_virtual, Kd, Bd, Md, m_gramos, f_raw, q_deseada_deg, cuentas_ini):
    desv_teorica_deg = np.rad2deg(tau_virtual / Kd)
    zeta = Bd / (2.0 * np.sqrt(Md * Kd))
    wn   = np.sqrt(Kd / Md)

    if zeta < 0.98:
        wd = wn * np.sqrt(max(1e-6, 1.0 - zeta ** 2))
        periodo_s = (2.0 * np.pi) / wd
        tipo_amort = f"Subamortiguado (oscila ~1 ciclo cada {periodo_s:.2f} s)"
    elif zeta <= 1.02:
        tipo_amort = "Críticamente amortiguado (retorno limpio sin rebote)"
    else:
        tipo_amort = "Sobreamortiguado (retorno lento y firme)"

    linea_f = (
        f"{f_raw:+.2f} N  -> tau = {tau_virtual:+.4f} N*m (L={L_BRAZO:.2f} m)"
        if F_EN_NEWTONS else f"{tau_virtual:+.4f} N*m"
    )

    print("\n" + "=" * 96)
    print("  PARÁMETROS RECIBIDOS DESDE EL ESP32 — CONVERSIÓN APLICADA")
    print("=" * 96)
    print(f"  1. Constante del Resorte (K / Kd)   : {Kd:.4f} N*m/rad")
    print(f"  2. Amortiguamiento (B / Bd)         : {Bd:.4f} N*m*s/rad")
    print(f"  3. Masa Virtual (M)                 : {m_gramos:.1f} g  -> Md = {Md:.5f} kg*m^2 (L={L_BRAZO:.2f} m)")
    print(f"  4. Fuerza / Torque Virtual (F / tau): {linea_f}")
    print("-" * 96)
    print(f"  -> Factor de amortiguamiento (zeta) : {zeta:.3f} ({tipo_amort})")
    print(f"  -> Frecuencia natural (wn)          : {wn:.2f} rad/s")
    print(f"  -> Deflexión esperada (Hooke F/K)   : {desv_teorica_deg:+.2f}° (Meta bajo carga: {q_deseada_deg - desv_teorica_deg:.2f}°)")
    print("=" * 96 + "\n")


def ejecutar_experimento(ser_servo, tau_virtual, Kd, Bd, Md, m_gramos, f_raw, A_kf, B_kf, Q_q, H_kf, R_q):
    A_imp, B_imp = construir_matrices_impedancia(Md, Kd, Bd, dt)

    q_deseada_deg = 45.0
    q_deseada     = np.deg2rad(q_deseada_deg)
    cuentas_ini   = int(round(CUENTA_CENTRO + q_deseada / res_encoder))

    imprimir_resumen_parametros(tau_virtual, Kd, Bd, Md, m_gramos, f_raw, q_deseada_deg, cuentas_ini)

    print(f"[FASE 0] Posicionando Servo ID {SERVO_ID} en {q_deseada_deg}° ({cuentas_ini} cuentas)...")
    comando_pwm_cmd29(ser_servo, SERVO_ID, 0, 0)
    time.sleep(0.05)
    mover_servo_cmd1(ser_servo, SERVO_ID, cuentas_ini, 800)
    time.sleep(1.0)

    v_mv = solicitar_lectura(ser_servo, SERVO_ID, CMD_SERVO_VIN_READ, signed=False)
    if v_mv is not None and 5000 <= v_mv <= 16000:
        print(f"  -> Voltaje en servo (CMD 27): {v_mv / 1000.0:.2f} V (OK)")
    else:
        print(f"  -> [ALERTA SERVO] Sin respuesta de CMD 27 en {PUERTO_SERVO}. Revisa ID={SERVO_ID}, alimentación 11.1V y cableado.")

    P_q      = np.diag([(res_encoder ** 2) / 4.0, 1e-5, 1e-3])
    x_hat_q  = np.array([[q_deseada], [0.0], [0.0]])
    x_imp    = np.array([[0.0], [0.0]])

    G_q_ini       = m * g * lc * np.cos(q_deseada)
    V_ctrl        = (G_q_ini / Kt) * Ra
    ultimo_q      = q_deseada
    ultima_cuenta = cuentas_ini

    # 1s reposo + 5s fuerza virtual + 4s retorno = 10s
    T_INI_FUERZA = 1.0
    T_FIN_FUERZA = 6.0
    T_TOTAL      = 10.0

    encabezado = (
        f"{'t(s)':>6} | {'Fase / Evento':<13} | {'Cuentas':>7} | {'PosRX(°)':>8} | "
        f"{'Kalm_q(°)':>9} | {'TauESP':>7} | {'TauEst':>7} | {'Desv_e(°)':>9} | {'PWM':>5}"
    )
    print("\n" + encabezado)
    print("-" * len(encabezado))

    hist_q_med, hist_q_est, hist_tau_real, hist_tau_est = [], [], [], []
    t0 = time.perf_counter()
    ciclo = 0
    lecturas_fallidas = 0
    fase_anterior = ""

    while True:
        t_ciclo  = time.perf_counter()
        t_actual = t_ciclo - t0
        if t_actual > T_TOTAL:
            break

        # --- A) FASE DE LA PRUEBA ---
        if t_actual < T_INI_FUERZA:
            tau_ext_soft = 0.0
            estado_txt   = "1.REPOSO"
        elif t_actual <= T_FIN_FUERZA:
            tau_ext_soft = tau_virtual
            estado_txt   = "2.FUERZA_VIRT"
        else:
            tau_ext_soft = 0.0
            estado_txt   = "3.RETORNO"

        if estado_txt != fase_anterior:
            print(f">>> [t={t_actual:4.2f}s] ETAPA -> {estado_txt}")
            fase_anterior = estado_txt

        # --- B) LECTURA SERIAL DEL ENCODER (CMD 28) ---
        p_c = solicitar_lectura(ser_servo, SERVO_ID, CMD_SERVO_POS_READ, signed=True)
        if p_c is not None:
            ultima_cuenta = p_c
            ultimo_q = (p_c - CUENTA_CENTRO) * res_encoder
        else:
            lecturas_fallidas += 1
        q_cmd28 = ultimo_q

        # --- C) FILTRO DE KALMAN EXACTO (ZOH) SOLO CON ENCODER ---
        G_q_prev   = m * g * lc * np.cos(x_hat_q[0, 0])
        V_eff_pred = V_ctrl - (G_q_prev * Ra / Kt)

        x_pred_q = A_kf @ x_hat_q + B_kf * V_eff_pred
        P_pred_q = A_kf @ P_q @ A_kf.T + Q_q

        z_med   = np.array([[q_cmd28]])
        y_res_q = z_med - (H_kf @ x_pred_q)
        S_q     = H_kf @ P_pred_q @ H_kf.T + R_q
        K_q     = (P_pred_q @ H_kf.T) / float(S_q[0, 0])
        x_hat_q = x_pred_q + (K_q @ y_res_q)

        I_KH = np.eye(3) - (K_q @ H_kf)
        P_q  = I_KH @ P_pred_q @ I_KH.T + K_q @ R_q @ K_q.T

        q_est       = float(x_hat_q[0, 0])
        qd_est      = float(x_hat_q[1, 0])
        tau_ext_est = float(x_hat_q[2, 0])

        # --- D) DINÁMICA DE IMPEDANCIA (M en gramos -> Md, K, B) ---
        x_imp = A_imp @ x_imp + B_imp * tau_ext_est
        e_est = float(np.clip(x_imp[0, 0], np.deg2rad(-65.0), np.deg2rad(65.0)))
        x_imp[0, 0] = e_est

        # --- E) LEY DE CONTROL + INYECCIÓN DE FUERZA VIRTUAL ---
        G_q_est   = m * g * lc * np.cos(q_est)
        q_comando = q_deseada - e_est
        error_seg = q_comando - q_est

        if abs(error_seg) < res_encoder and abs(e_est) < res_encoder and tau_ext_soft == 0.0:
            V_ctrl = (G_q_est / Kt) * Ra - (0.8 * Kd_v * qd_est)
        else:
            V_ff   = (G_q_est / Kt) * Ra + Ke * qd_est
            V_ctrl = Kp_v * error_seg - Kd_v * qd_est + V_ff

        V_ctrl = float(np.clip(V_ctrl, -V_max, V_max))

        V_perturbacion = -(tau_ext_soft / Kt) * Ra
        V_total        = float(np.clip(V_ctrl + V_perturbacion, -V_max, V_max))

        pwm_val = int(round((V_total / V_max) * 1000.0))
        comando_pwm_cmd29(ser_servo, SERVO_ID, 1, pwm_val)

        hist_q_med.append(q_cmd28)
        hist_q_est.append(q_est)
        hist_tau_real.append(tau_ext_soft)
        hist_tau_est.append(tau_ext_est)

        ciclo += 1
        if ciclo % 15 == 0:
            print(
                f"{t_actual:6.2f} | {estado_txt:<13} | {ultima_cuenta:7d} | {np.rad2deg(q_cmd28):8.2f} | "
                f"{np.rad2deg(q_est):9.2f} | {tau_ext_soft:+7.3f} | {tau_ext_est:+7.3f} | "
                f"{np.rad2deg(e_est):+9.2f} | {pwm_val:+5d}"
            )

        t_restante = dt - (time.perf_counter() - t_ciclo)
        if t_restante > 0:
            time.sleep(t_restante)

    # Restaurar servo al finalizar la prueba
    print("-" * len(encabezado))
    comando_pwm_cmd29(ser_servo, SERVO_ID, 0, 0)
    time.sleep(0.05)
    mover_servo_cmd1(ser_servo, SERVO_ID, cuentas_ini, 600)

    rmse_q     = np.rad2deg(np.sqrt(np.mean((np.array(hist_q_med) - np.array(hist_q_est)) ** 2)))
    rmse_tau   = np.sqrt(np.mean((np.array(hist_tau_real) - np.array(hist_tau_est)) ** 2))
    tasa_exito = 100.0 * (1.0 - lecturas_fallidas / max(1, ciclo))

    print(f"PRUEBA FINALIZADA | Paquetes servo OK: {tasa_exito:.1f}% | RMSE Pos: {rmse_q:.3f}° | RMSE Tau: {rmse_tau:.3f} N*m")


def main():
    ser_servo = None
    ser_esp   = None

    try:
        ser_servo = serial.Serial(PUERTO_SERVO, BAUDIOS, timeout=0.0025)
        print(f"[OK] Servo HTD-35H abierto en {PUERTO_SERVO} ({BAUDIOS} bps).")
    except serial.SerialException as e:
        print(f"[ERROR] No se pudo abrir {PUERTO_SERVO}: {e}")
        return

    try:
        ser_esp = serial.Serial(PUERTO_ESP32, BAUDIOS, timeout=0.2)
        time.sleep(1.5)
        ser_esp.reset_input_buffer()
        print(f"[OK] ESP32 abierto en {PUERTO_ESP32} ({BAUDIOS} bps).")
    except serial.SerialException as e:
        print(f"[ERROR] No se pudo abrir {PUERTO_ESP32}: {e}")
        ser_servo.close()
        return

    # Discretización exacta ZOH del Filtro de Kalman (solo encoder CMD 28)
    b_eq = b + (Kt * Ke) / Ra
    k_v  = Kt / (Ra * I)
    Ac   = np.array([
        [0.0,       1.0,    0.0],
        [0.0, -b_eq / I, -1.0 / I],
        [0.0,       0.0,    0.0]
    ])
    Bc = np.array([[0.0], [k_v], [0.0]])
    Gc = np.array([[0.0], [0.0], [1.0]])

    M_aug = np.zeros((4, 4))
    M_aug[0:3, 0:3], M_aug[0:3, 3:4] = Ac, Bc
    E_aug = expm_exacta(M_aug * dt)
    A_kf, B_kf = E_aug[0:3, 0:3], E_aug[0:3, 3:4]

    q_tau = 0.8
    M_vl  = np.zeros((6, 6))
    M_vl[0:3, 0:3], M_vl[0:3, 3:6], M_vl[3:6, 3:6] = -Ac, Gc @ (q_tau * Gc.T), Ac.T
    Q_q   = A_kf @ expm_exacta(M_vl * dt)[0:3, 3:6]
    Q_q   = 0.5 * (Q_q + Q_q.T)

    H_kf = np.array([[1.0, 0.0, 0.0]])
    R_q  = np.array([[(res_encoder ** 2) / 4.0]])

    # Memoria persistente sincronizada con el ESP32 (M ahora en gramos)
    estado_esp = {'K': Kd_RESPALDO, 'B': 0.165, 'M': M_GRAMOS_RESPALDO, 'F': 0.0}

    try:
        while True:
            tau_val, kd_val, bd_val, md_val, m_gramos, f_raw = esperar_confirmacion_esp32(ser_esp, estado_esp)
            ejecutar_experimento(
                ser_servo, tau_val, kd_val, bd_val, md_val, m_gramos, f_raw,
                A_kf, B_kf, Q_q, H_kf, R_q
            )
            ser_esp.reset_input_buffer()

    except KeyboardInterrupt:
        print("\n[DETENIDO] Finalizado por el usuario (Ctrl+C).")
    finally:
        if ser_servo and ser_servo.is_open:
            comando_pwm_cmd29(ser_servo, SERVO_ID, 0, 0)
            ser_servo.close()
        if ser_esp and ser_esp.is_open:
            ser_esp.close()
        print("Puertos seriales cerrados correctamente.")


if __name__ == '__main__':
    main()
