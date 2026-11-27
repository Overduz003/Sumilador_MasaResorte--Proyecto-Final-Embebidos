#include <Wire.h>
#include <Adafruit_INA219.h>

// --- Pines (sin cambios) ---
const int PIN_SDA   = 21;
const int PIN_SCL   = 22;
const int PIN_RPWM  = 23;
const int PIN_LPWM  = 19;
const int PIN_ENC_A = 2;
const int PIN_ENC_B = 15;

// --- Configuración del motor y encoder (1:34, lectura x2) ---
const float PULSOS_POR_VUELTA = 748.0f;
const bool  INVERTIR_MOTOR    = true;
const bool  INVERTIR_CORRIENTE = true;

// --- 1. Parámetros del Sistema Masa-Resorte-Amortiguador Virtual ---
float thetaRef  = 0.0f;   // Equilibrio (grados)
float M_VIRTUAL = 0.05f;  // Inercia virtual (más grande = oscila más lento)
float K_VIRTUAL = 2.0f;   // Rigidez del resorte
float B_VIRTUAL = 0.0f;   // Amortiguamiento (0.0 = oscilación libre)

// --- Detección de empuje externo (INA219 -> torque virtual) ---
const float I_UMBRAL_mA   = 180.0f; // Corriente mínima para considerar que lo estás empujando
const float GANANCIA_EXT  = 0.15f;  // Qué tan fácil se mueve el sistema al empujarlo
const bool  INVERTIR_EXT  = false;  // Cambia a true si al empujarlo se mueve al revés

// --- 2. Lazo interno de posición (PD) para vencer la fricción 1:34 ---
float Kp_pos = 4.5f;
float Kd_pos = 0.08f;
const int MAX_PWM = 200;

// --- Filtros EMA ---
const float ALPHA_VEL = 0.25f;
const float ALPHA_CUR = 0.30f;

Adafruit_INA219 ina219;
bool inaDisponible = false;

volatile long pulsosEncoder = 0;
float anguloAnterior    = 0.0f;
float velReal           = 0.0f;
float corrienteFiltrada = 0.0f;

// Estados del sistema virtual
float thetaVirtual = 0.0f;
float velVirtual   = 0.0f;

unsigned long tiempoAnterior = 0;

void IRAM_ATTR isrEncoderA() {
  int estadoA = digitalRead(PIN_ENC_A);
  int estadoB = digitalRead(PIN_ENC_B);
  if (estadoA == estadoB) pulsosEncoder++;
  else                    pulsosEncoder--;
}

void moverMotor(int pwm) {
  if (INVERTIR_MOTOR) pwm = -pwm;
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0) {
    analogWrite(PIN_RPWM, pwm);
    analogWrite(PIN_LPWM, 0);
  } else if (pwm < 0) {
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, -pwm);
  } else {
    analogWrite(PIN_RPWM, 0);
    analogWrite(PIN_LPWM, 0);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);

  pinMode(PIN_RPWM, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT);
  moverMotor(0);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncoderA, CHANGE);

  if (ina219.begin()) {
    inaDisponible = true;
  } else {
    Serial.println("ERROR: No se detecto INA219");
  }

  tiempoAnterior = millis();
}

void loop() {
  unsigned long ahora = millis();
  float dt = (ahora - tiempoAnterior) / 1000.0f;
  if (dt < 0.01f) return; // 100 Hz
  tiempoAnterior = ahora;

  if (!inaDisponible) {
    moverMotor(0);
    return;
  }

  // 1. Leer posición y velocidad reales
  noInterrupts();
  long pulsos = pulsosEncoder;
  interrupts();

  float anguloActual = (pulsos * 360.0f) / PULSOS_POR_VUELTA;
  float velCruda     = (anguloActual - anguloAnterior) / dt;
  velReal            = (ALPHA_VEL * velCruda) + ((1.0f - ALPHA_VEL) * velReal);
  anguloAnterior     = anguloActual;

  // 2. Leer corriente y estimar empuje externo (tauExt)
  float corrienteCruda = ina219.getCurrent_mA();
  corrienteFiltrada    = (ALPHA_CUR * corrienteCruda) + ((1.0f - ALPHA_CUR) * corrienteFiltrada);
  float corrienteReal  = INVERTIR_CORRIENTE ? -corrienteFiltrada : corrienteFiltrada;

  float tauExt = 0.0f;
  if (abs(corrienteReal) > I_UMBRAL_mA) {
    // Quitamos el umbral para que el empuje sea suave desde cero
    float exceso = (corrienteReal > 0) ? (corrienteReal - I_UMBRAL_mA) : (corrienteReal + I_UMBRAL_mA);
    // El motor hace fuerza opuesta a tu mano, por eso el signo negativo
    tauExt = (INVERTIR_EXT ? 1.0f : -1.0f) * exceso * GANANCIA_EXT;
  }

  // 3. Dinámica Masa-Resorte-Amortiguador virtual (Euler semi-implícito)
  float fuerzaResorte = K_VIRTUAL * (thetaVirtual - thetaRef);
  float fuerzaAmort   = B_VIRTUAL * velVirtual;
  float accVirtual    = (tauExt - fuerzaResorte - fuerzaAmort) / M_VIRTUAL;

  velVirtual   += accVirtual * dt;
  thetaVirtual += velVirtual * dt;

  // Límites de seguridad del ángulo virtual
  thetaVirtual = constrain(thetaVirtual, -120.0f, 120.0f);

  // 4. Lazo interno PD: el motor sigue al ángulo virtual
  float errorTrack = thetaVirtual - anguloActual;
  int salidaPWM    = (int)((Kp_pos * errorTrack) - (Kd_pos * velReal));
  salidaPWM        = constrain(salidaPWM, -MAX_PWM, MAX_PWM);

  moverMotor(salidaPWM);

  // 5. Telemetría para Serial Plotter
  Serial.print("ThetaVirt:"); Serial.print(thetaVirtual);
  Serial.print(",Angulo:");   Serial.print(anguloActual);
  Serial.print(",I_mA:");     Serial.println(corrienteReal);
}
