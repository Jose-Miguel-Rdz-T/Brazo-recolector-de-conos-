#include <ESP32Servo.h>

//----------SENSOR ULTRASÃ“NICO--------------
const int pin_echo = 4;
const int pin_trigger = 2;

//-----------SENSORES DE LÃNEA--------------
const int sen_izq = 35;
const int sen_der = 15;

//------------PUENTE H-------------------
const int pinena = 23;
const int pina1 = 21;
const int pina2 = 19;
const int pinb1 = 5;
const int pinb2 = 18;
const int pinenb = 22;

//------------BRAZO----------------
Servo base;  
Servo brazo;
Servo codo;
Servo pinza;
Servo muneca;

const int pinBase = 13;
const int pinBrazo = 12;
const int pinCodo = 14;
const int pinpinza = 26;
const int pinmuneca = 27;

const int DISTANCIA_OBJETIVO = 25; // 25 cm

void setup() {
  Serial.begin(115200);

  // Configurar pines
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
}

float medirDistancia() {
  digitalWrite(pin_trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trigger, LOW);
  
  long duracion = pulseIn(pin_echo, HIGH);
  float distancia = duracion * 0.034 / 2;
  return distancia;
}

void loop() {
  float distancia = medirDistancia();

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    Serial.println("Comando recibido: [" + command + "]");
    Serial.println("Distancia actual: " + String(distancia) + " cm");

    if (command == "MOVE_FORWARD") {
      avanzar();
      Serial.println("ACK: MOVE_FORWARD DONE");
    } 
    else if (command == "STOP") {
      detener();
      Serial.println("ACK: STOP DONE");
    }
    else if (command == "COLLECT_CONE") {
      if (distancia <= DISTANCIA_OBJETIVO) {
        detener();
        recoger_cono();
        Serial.println("ACK: COLLECT_CONE DONE");
      } else {
        Serial.println("ERR: DISTANCE TOO FAR FOR COLLECT_CONE");
      }
    }
    else if (command == "PRE_RELEASE_CONE") {
      pre_soltar_cono();
      Serial.println("ACK: PRE_RELEASE_CONE DONE");
    }
    else if (command == "PUSH_CONE") {
      if (distancia <= DISTANCIA_OBJETIVO) {
        detener();
        empujar_cono();
        Serial.println("ACK: PUSH_CONE DONE");
      } else {
        Serial.println("ERR: DISTANCE TOO FAR FOR PUSH_CONE");
      }
    }
    else if (command == "HOME_POSITION") {
      home_position();
      Serial.println("ACK: HOME_POSITION DONE");
    }
    else if (command == "AVOID_LINE") {
      evitar_linea();
      Serial.println("ACK: AVOID_LINE DONE");
    }
    else {
      Serial.println("ERR: UNKNOWN COMMAND");
    }
  }

  if (digitalRead(sen_izq) == LOW || digitalRead(sen_der) == LOW) {
    evitar_linea();
  }

  delay(100);
}

void avanzar() {
  digitalWrite(pina1, HIGH);
  digitalWrite(pina2, LOW);
  digitalWrite(pinb1, HIGH);
  digitalWrite(pinb2, LOW);
  analogWrite(pinena, 150);
  analogWrite(pinenb, 150);
}

void detener() {
  digitalWrite(pina1, LOW);
  digitalWrite(pina2, LOW);
  digitalWrite(pinb1, LOW);
  digitalWrite(pinb2, LOW);
  analogWrite(pinena, 0);
  analogWrite(pinenb, 0);
}

void retroceder() {
  digitalWrite(pina1, LOW);
  digitalWrite(pina2, HIGH);
  digitalWrite(pinb1, LOW);
  digitalWrite(pinb2, HIGH);
  analogWrite(pinena, 150);
  analogWrite(pinenb, 150);
}

void girar_izquierda() {
  digitalWrite(pina1, LOW);
  digitalWrite(pina2, HIGH);
  digitalWrite(pinb1, HIGH);
  digitalWrite(pinb2, LOW);
  analogWrite(pinena, 150);
  analogWrite(pinenb, 150);
  delay(500);
}

void girar_derecha() {
  digitalWrite(pina1, HIGH);
  digitalWrite(pina2, LOW);
  digitalWrite(pinb1, LOW);
  digitalWrite(pinb2, HIGH);
  analogWrite(pinena, 150);
  analogWrite(pinenb, 150);
  delay(500);
}

void evitar_linea() {
  detener();
  retroceder();
  delay(500);
  detener();

  bool linea_izq = (digitalRead(sen_izq) == LOW);
  bool linea_der = (digitalRead(sen_der) == LOW);
  
  if (linea_izq && !linea_der) {
    girar_derecha();
  } else if (!linea_izq && linea_der) {
    girar_izquierda();
  } else {
    retroceder();
  }

  detener();
  delay(100);
  avanzar();
}

void empujar_cono() {
  moverServo(base, 90);
  delay(500);
  moverServo(brazo, 60);
  delay(500);
  moverServo(codo, 90);
  delay(500);
  moverServo(muneca, 90);
  delay(500);

  for (int i = 0; i < 3; i++) {
    moverServo(brazo, 30);
    delay(300);
    moverServo(brazo, 60);
    delay(300);
  }

  home_position();
}

void pre_soltar_cono() {
  moverServo(brazo, 120);
  delay(500);
  moverServo(brazo, 30);
  delay(500);
  moverServo(base, 180);
  delay(500);
  moverServo(brazo, 0);
  delay(500);
  moverServo(codo, 0);
  delay(500);
  moverServo(muneca, 40);
  delay(500);
  moverServo(pinza, 90);
  delay(500);
  home_position();
}

void recoger_cono() {
  moverServo(brazo, 130);
  delay(500);
  moverServo(codo, 60);
  delay(500); 
  moverServo(muneca, 125);
  delay(500);
  moverServo(pinza, 50);
  delay(500);
  pre_soltar_cono();
  delay(500);
  home_position();
}

void home_position() {
  moverServo(base, 0);
  delay(500); 
  moverServo(brazo, 100);
  delay(500);   
  moverServo(codo, 160);
  delay(500); 
  moverServo(muneca, 150);
  delay(500); 
  moverServo(pinza, 90);
  delay(500);
}

void configurarServo(Servo &servo, int pin, int minUs, int maxUs) {
  servo.setPeriodHertz(50);
  servo.attach(pin, minUs, maxUs);
}

void moverServo(Servo &servo, int angulo) {
  if (servo.attached()) {
    servo.write(angulo);
    delay(1500);
  }
}
