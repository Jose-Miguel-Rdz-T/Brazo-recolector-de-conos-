#include <ESP32Servo.h>

// Pines sensores TCRT5000
#define SENSOR_IZQUIERDO 15
#define SENSOR_DERECHO 25

// Pines puente H
#define ENA 23
#define IN1 19
#define IN2 21
#define IN3 18
#define IN4 5
#define ENB 22

// Pines servos
Servo base, brazo, codo, pinza, muneca;
const int pinBase = 13, pinBrazo = 12, pinCodo = 14, pinPinza = 26, pinMuneca = 27;

// Estados del robot
enum EstadoRobot {AVANZANDO, RETROCEDIENDO, GIRANDO, DETENIDO};
EstadoRobot estadoActual = AVANZANDO;

// Tiempos y umbrales
const unsigned long TIEMPO_EVASION = 2500;  // Tiempo mínimo entre detecciones
const unsigned long TIEMPO_RETROCESO = 1000;
const unsigned long TIEMPO_GIRO = 500;
unsigned long ultimaDeteccion = 0;

void setup() {
  Serial.begin(115200);

  pinMode(SENSOR_IZQUIERDO, INPUT);
  pinMode(SENSOR_DERECHO, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Inicializa motores con velocidad media
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);

  // Configurar servos
  configurarServo(base, pinBase, 500, 2500);
  configurarServo(brazo, pinBrazo, 500, 2500);
  configurarServo(codo, pinCodo, 500, 2500);
  configurarServo(pinza, pinPinza, 500, 2500);
  configurarServo(muneca, pinMuneca, 500, 2500);

  home_position();
  delay(1000);

  avanzar();
}

void loop() {
  int izq = digitalRead(SENSOR_IZQUIERDO);  // 1 = negro, 0 = blanco
  int der = digitalRead(SENSOR_DERECHO);

  // Solo procesar detección si estamos avanzando
  if (estadoActual == AVANZANDO && (izq == 1 || der == 1)) {
    if (millis() - ultimaDeteccion > TIEMPO_EVASION) {
      ultimaDeteccion = millis();
      evitarLineaNegra(izq, der);
    }
  }

  // Procesar comandos UART (desde Raspberry Pi)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "MOVE_FORWARD") {
      avanzar();
      estadoActual = AVANZANDO;
    }
    else if (cmd == "STOP") {
      detener();
      estadoActual = DETENIDO;
    }
    else if (cmd == "HANDLE_YELLOW_CONE") manejar_cono_amarillo();
    else if (cmd == "HANDLE_RED_CONE") manejar_cono_rojo();
  }
  
  delay(10);
}

// Función mejorada para evitar líneas negras
void evitarLineaNegra(int sensorIzq, int sensorDer) {
  Serial.println("Evitando línea negra");
  detener();
  estadoActual = RETROCEDIENDO;
  delay(100);
  
  retroceder();
  delay(TIEMPO_RETROCESO);
  
  // Decide la dirección de giro basado en qué sensor detectó la línea
  if (sensorIzq && sensorDer) {
    // Ambos sensores detectan - girar 180°
    girarDerecha();
    delay(TIEMPO_GIRO * 2);
  } else if (sensorIzq) {
    // Línea detectada a la izquierda - girar derecha
    girarDerecha();
    delay(TIEMPO_GIRO);
  } else {
    // Línea detectada a la derecha - girar izquierda
    girarIzquierda();
    delay(TIEMPO_GIRO);
  }
  
  avanzar();
  estadoActual = AVANZANDO;
}

// Funciones de movimiento mejoradas
void avanzar() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  Serial.println("Avanzando");
}

void retroceder() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  Serial.println("Retrocediendo");
}

void girarDerecha() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  Serial.println("Girando derecha");
}

void girarIzquierda() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  Serial.println("Girando izquierda");
}

void detener() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  Serial.println("Detenido");
}

// Funciones de servo
void configurarServo(Servo &s, int pin, int minUs, int maxUs) {
  s.setPeriodHertz(50);
  s.attach(pin, minUs, maxUs);
}

void moverServo(Servo &s, int angulo) {
  if (s.attached()) {
    s.write(angulo);
    delay(500);
  }
}

// Rutinas para el brazo y conos
void home_position() {
  moverServo(base, 0);
  moverServo(brazo, 100);
  moverServo(codo, 160);
  moverServo(muneca, 150);
  moverServo(pinza, 90);
}

void manejar_cono_amarillo() {
  detener();
  estadoActual = DETENIDO;
  recoger_cono();
  pre_soltar_cono();
  home_position();
  avanzar();
  estadoActual = AVANZANDO;
  Serial.println("COMPLETADA: AMARILLO");
}

void manejar_cono_rojo() {
  detener();
  estadoActual = DETENIDO;
  empujar_cono();
  home_position();
  avanzar();
  estadoActual = AVANZANDO;
  Serial.println("COMPLETADA: ROJO");
}

void recoger_cono() {
  moverServo(brazo, 130);
  moverServo(codo, 60);
  moverServo(muneca, 125);
  moverServo(pinza, 50);
  delay(500);
}

void pre_soltar_cono() {
  moverServo(brazo, 120);
  moverServo(brazo, 30);
  moverServo(base, 180);
  moverServo(brazo, 0);
  moverServo(codo, 0);
  moverServo(muneca, 40);
  moverServo(pinza, 90);
  home_position();
}

void empujar_cono() {
  avanzar();
  delay(1000);
  detener();
}
