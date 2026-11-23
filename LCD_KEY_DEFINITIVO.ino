#include <Keypad.h>
#include <LiquidCrystal.h>

// LCD: (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd(19, 18, 5, 17, 16, 4);

// Teclado 4x4
const byte FILAS = 4;
const byte COLUMNAS = 4;
char teclas[FILAS][COLUMNAS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte pinesFilas[FILAS] = {13, 12, 14, 27};
byte pinesColumnas[COLUMNAS] = {26, 25, 33, 32};
Keypad teclado = Keypad(makeKeymap(teclas), pinesFilas, pinesColumnas, FILAS, COLUMNAS);

// Orden lógico: 0=K, 1=B, 2=M, 3=F
const char nombres[4] = {'K', 'B', 'M', 'F'};
String textos[4]      = {"", "", "", ""};
float valores[4]      = {0.0, 0.0, 0.0, 0.0};

int estadoActual    = 0;     // 0:K, 1:B, 2:M, 3:F
bool sobreescribir  = false;
bool todoConfirmado = false;

String formatearBloque(int idx) {
  String bloque = String(nombres[idx]) + ":" + textos[idx];
  while (bloque.length() < 7) bloque += " ";
  bloque = bloque.substring(0, 7);

  if (estadoActual == idx && !todoConfirmado) {
    bloque += "<";
  } else {
    bloque += " ";
  }
  return bloque;
}

void actualizarPantalla() {
  // Renglón superior: K (índice 0) y M (índice 2)
  lcd.setCursor(0, 0);
  lcd.print(formatearBloque(0) + formatearBloque(2));

  // Renglón inferior: B (índice 1) y F (índice 3)
  lcd.setCursor(0, 1);
  lcd.print(formatearBloque(1) + formatearBloque(3));
}

void cambiarEstado(int nuevoEstado) {
  estadoActual = nuevoEstado;
  todoConfirmado = false;
  sobreescribir = (textos[estadoActual].length() > 0);
  actualizarPantalla();
}

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  actualizarPantalla();
}

void loop() {
  char tecla = teclado.getKey();
  if (!tecla) return;

  // Avanzar entre variables (Tecla A)
  if (tecla == 'A') {
    int siguiente = todoConfirmado ? 0 : (estadoActual + 1) % 4;
    cambiarEstado(siguiente);
  }
  // Retroceder entre variables (Tecla B)
  else if (tecla == 'B') {
    int anterior = todoConfirmado ? 3 : (estadoActual + 3) % 4;
    cambiarEstado(anterior);
  }
  // Punto decimal (Tecla *)
  else if (tecla == '*') {
    todoConfirmado = false;
    String &actual = textos[estadoActual];

    if (sobreescribir) {
      actual = "0.";
      sobreescribir = false;
    } else if (actual.indexOf('.') == -1 && actual.length() < 4) {
      actual = (actual.length() == 0) ? "0." : actual + ".";
    }
    actualizarPantalla();
  }
  // Done / Guardar y avanzar en orden K -> B -> M -> F (Tecla D)
  else if (tecla == 'D') {
    valores[estadoActual] = textos[estadoActual].toFloat();
    Serial.print(nombres[estadoActual]);
    Serial.print(" guardado: ");
    Serial.println(valores[estadoActual], 4);

    if (estadoActual < 3) {
      cambiarEstado(estadoActual + 1);
    } else {
      todoConfirmado = true;
      sobreescribir = true;
      Serial.println("--- K, B, M, F confirmados ---");
      actualizarPantalla();
    }
  }
  // Borrar campo actual (Tecla #)
  else if (tecla == 'C') {
    textos[estadoActual] = "";
    todoConfirmado = false;
    sobreescribir = false;
    actualizarPantalla();
  }
  // Dígitos (0 - 9)
  else if (tecla >= '0' && tecla <= '9') {
    todoConfirmado = false;
    String &actual = textos[estadoActual];

    if (sobreescribir) {
      actual = "";
      sobreescribir = false;
    }
    if (actual.length() < 5) {
      actual += tecla;
    }
    actualizarPantalla();
  }
}