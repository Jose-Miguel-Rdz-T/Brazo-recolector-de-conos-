#include <ESP32Servo.h>

// Pines sensores y motores
const int pin_echo = 4, pin_trigger = 2;
const int sen_izq = 35, sen_der = 15;
const int pinena = 23, pina1 = 21, pina2 = 19;
const int pinb1 = 5, pinb2 = 18, pinenb = 22;

// Servos
Servo base, brazo, codo, pinza, muneca;
const int pinBase = 13, pinBrazo = 12, pinCodo = 14, pinpinza = 26, pinmuneca = 27;

const int DISTANCIA_OBJETIVO = 25;

void setup() {
  Serial.begin(115200);
  pinMode(pin_echo, INPUT);
  pinMode(pin_trigger, OUTPUT);
  pinMode(sen_izq, INPUT);
  pinMode(sen_der, INPUT);
  pinMode(pinena, OUTPUT);
  pinMode(pina1, OUTPUT);
  pinMode(pina2, OUTPUT);
  pinMode(pinb1, OUTPUT);
  pinMode(pinb2, OUTPUT);
  pinMode(pinenb, OUTPUT);

  configurarServo(base, pinBase, 500, 2500);
  configurarServo(brazo, pinBrazo, 500, 2500);
  configurarServo(codo, pinCodo, 500, 2500);
  configurarServo(pinza, pinpinza, 500, 2500);
  configurarServo(muneca, pinmuneca, 500, 2500);

  home_position();
  delay(1000);
  avanzar();
}

void loop() {
  static unsigned long t0 = 0;
  if (millis() - t0 >= 200) {
    t0 = millis();

    float d = medirDistancia();
    if ((digitalRead(sen_izq) == LOW) || (digitalRead(sen_der) == LOW)) {
      evitar_linea();
      return;
    }
    if (d > 0 && d < DISTANCIA_OBJETIVO) {
      detener();
      Serial.println("OBJETO_DETECTADO");
    }
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "MOVE_FORWARD") avanzar();
    else if (cmd == "STOP") detener();
    else if (cmd == "HANDLE_YELLOW_CONE") {
      manejar_cono_amarillo();
    } else if (cmd == "HANDLE_RED_CONE") {
      manejar_cono_rojo();
    }
  }
}

// FUNCIONES AUXILIARES

void home_position() {
  moverServo(base, 0); 
  moverServo(brazo, 100); delay(200);
  moverServo(codo, 160); delay(200);
  moverServo(muneca, 150); delay(200);
  moverServo(pinza, 90);
}

void avanzar() {
  digitalWrite(pina1, HIGH); digitalWrite(pina2, LOW);
  digitalWrite(pinb1, HIGH); digitalWrite(pinb2, LOW);
  analogWrite(pinena, 100); analogWrite(pinenb, 100);
}

void detener() {
  analogWrite(pinena, 0); analogWrite(pinenb, 0);
}

void retroceder() {
  digitalWrite(pina1, LOW); digitalWrite(pina2, HIGH);
  digitalWrite(pinb1, LOW); digitalWrite(pinb2, HIGH);
  analogWrite(pinena, 100); analogWrite(pinenb, 100);
}

void girar_izquierda() {
  digitalWrite(pina1, LOW); digitalWrite(pina2, HIGH);
  digitalWrite(pinb1, HIGH); digitalWrite(pinb2, LOW);
  analogWrite(pinena, 100); analogWrite(pinenb, 100);
}

void girar_derecha() {
  digitalWrite(pina1, HIGH); digitalWrite(pina2, LOW);
  digitalWrite(pinb1, LOW); digitalWrite(pinb2, HIGH);
  analogWrite(pinena, 100); analogWrite(pinenb, 100);
}

void evitar_linea() {
  detener(); delay(300);
  retroceder(); delay(500);

  bool izq = digitalRead(sen_izq) == LOW;
  bool der = digitalRead(sen_der) == LOW;

  if (izq && !der) girar_derecha();
  else if (!izq && der) girar_izquierda();
  else retroceder();

  delay(500);
  avanzar();
}

void manejar_cono_amarillo() {
  detener(); recoger_cono(); pre_soltar_cono(); home_position(); avanzar();
  Serial.println("COMPLETADA:AMARILLO");
}

void manejar_cono_rojo() {
  detener(); evitar_linea(); home_position(); avanzar();
  Serial.println("COMPLETADA:ROJO");
}

void recoger_cono() {
  
  moverServo(brazo, 130); delay(500);
  moverServo(codo, 60); delay(500);
  moverServo(muneca, 125);
  moverServo(pinza, 50); delay(500);
  pre_soltar_cono(); delay(500); 
  home_position();
}

void pre_soltar_cono() {
  moverServo(brazo, 120); delay(500);
  moverServo(brazo, 30); delay(500);
  moverServo(base, 180);
  moverServo(brazo, 0); delay(500);
  moverServo(codo, 0); delay(500);
  moverServo(muneca, 40); 
  moverServo(pinza, 90);
  home_position();
}

void empujar_cono() {
  avanzar(); delay(1000); detener();
}

void configurarServo(Servo &s, int pin, int minUs, int maxUs) {
  s.setPeriodHertz(50); s.attach(pin, minUs, maxUs);
}

void moverServo(Servo &s, int angulo) {
  if (s.attached()) {
    s.write(angulo); delay(500);
  }
}

float medirDistancia() {
  digitalWrite(pin_trigger, LOW); delayMicroseconds(2);
  digitalWrite(pin_trigger, HIGH); delayMicroseconds(10);
  digitalWrite(pin_trigger, LOW);
  long duracion = pulseIn(pin_echo, HIGH, 25000);
  return duracion * 0.034 / 2;
}
