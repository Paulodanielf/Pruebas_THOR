#include <AccelStepper.h>

// Definición de pines para cada motor y sus sensores de final de carrera
// E0
#define MOTOR0_STEP_PIN 26
#define MOTOR0_DIR_PIN 28
#define MOTOR0_ENABLE_PIN 24
#define MOTOR0_LIMIT_PIN 31
// Z
#define MOTOR1_STEP_PIN 46
#define MOTOR1_DIR_PIN 48
#define MOTOR1_ENABLE_PIN 62
#define MOTOR1_LIMIT_PIN 31
// E1
#define MOTOR2_STEP_PIN 36
#define MOTOR2_DIR_PIN 34
#define MOTOR2_ENABLE_PIN 30
#define MOTOR2_LIMIT_PIN 31
// X
#define MOTOR3_STEP_PIN 54
#define MOTOR3_DIR_PIN 55
#define MOTOR3_ENABLE_PIN 38
#define MOTOR3_LIMIT_PIN 31
// Y
#define MOTOR4_STEP_PIN 60
#define MOTOR4_DIR_PIN 61
#define MOTOR4_ENABLE_PIN 56
#define MOTOR4_LIMIT_PIN 31
// Y2
#define MOTOR5_STEP_PIN 16
#define MOTOR5_DIR_PIN 17
#define MOTOR5_ENABLE_PIN 56
#define MOTOR5_LIMIT_PIN 31
 
AccelStepper steppers[6] = {
  AccelStepper(AccelStepper::DRIVER, MOTOR0_STEP_PIN, MOTOR0_DIR_PIN),
  AccelStepper(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN),
  AccelStepper(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN),
  AccelStepper(AccelStepper::DRIVER, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN),
  AccelStepper(AccelStepper::DRIVER, MOTOR4_STEP_PIN, MOTOR4_DIR_PIN),
  AccelStepper(AccelStepper::DRIVER, MOTOR5_STEP_PIN, MOTOR5_DIR_PIN)
};

int enablePins[6] = {MOTOR0_ENABLE_PIN, MOTOR1_ENABLE_PIN, MOTOR2_ENABLE_PIN, MOTOR3_ENABLE_PIN, MOTOR4_ENABLE_PIN, MOTOR5_ENABLE_PIN};
int limitPins[6] = {MOTOR0_LIMIT_PIN, MOTOR1_LIMIT_PIN, MOTOR2_LIMIT_PIN, MOTOR3_LIMIT_PIN, MOTOR4_LIMIT_PIN, MOTOR5_LIMIT_PIN};
const float gearRatio[6] = {5, 5.18 * 2 * 6, 5.18 * 6.716480186, 5, 2.0833333, 2.0833333};

int motorId;
float velocidadHz, angulo;

const int STEPS_PER_REVOLUTION = 200 * 4;

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 6; i++) {
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], LOW);
    pinMode(limitPins[i], INPUT_PULLUP); // Configurar pines de sensor como entrada con pull-up

    // Procedimiento de referencia al inicio
    steppers[i].setMaxSpeed(100); // Velocidad baja para referencia
    steppers[i].setAcceleration(100);

    while (digitalRead(limitPins[i]) != HIGH) {
      steppers[i].moveTo(-1); // Mueve hacia el sensor de final de carrera
      steppers[i].run();
    }

    steppers[i].setCurrentPosition(0); // Establece la posición actual como cero
    steppers[i].moveTo(0); // Asegúrate de que el motor esté en cero
  }

  Serial.println("Motores referenciados y listos.");
}

void loop() {
  if (Serial.available()) {
    motorId = Serial.parseInt() - 1;
    velocidadHz = Serial.parseFloat();
    angulo = Serial.parseFloat();

    Serial.print("Recibido: motorId=");
    Serial.print(motorId);
    Serial.print(", velocidadHz=");
    Serial.print(velocidadHz);
    Serial.print(", angulo=");
    Serial.println(angulo);

    if (motorId >= 0 && motorId < 6) {
      float velocidad = velocidadHz * STEPS_PER_REVOLUTION * gearRatio[motorId] / 360;
      long absolutePosition = (long)(STEPS_PER_REVOLUTION * angulo * gearRatio[motorId] / 360.0);

      if (motorId == 4 || motorId == 5) {
        // Sincronos
        if (motorId == 4) {
          Serial.println("Moviendo motores 4 y 5 de manera sincrona.");
          steppers[4].setMaxSpeed(velocidad);
          steppers[4].setAcceleration(8 * velocidad);
          steppers[4].moveTo(absolutePosition);
          
          steppers[5].setMaxSpeed(velocidad);
          steppers[5].setAcceleration(8 * velocidad);
          steppers[5].moveTo(absolutePosition);
        } 
        // Inversos
        else if (motorId == 5) {
          Serial.println("Moviendo motores 4 y 5 de manera inversa.");
          steppers[4].setMaxSpeed(velocidad);
          steppers[4].setAcceleration(8 * velocidad);
          steppers[4].moveTo(absolutePosition);
          
          steppers[5].setMaxSpeed(velocidad);
          steppers[5].setAcceleration(8 * velocidad);
          steppers[5].moveTo(-absolutePosition);
        }

        unsigned long startTime = millis(); // Registrar el tiempo de inicio

        while (steppers[4].distanceToGo() != 0 || steppers[5].distanceToGo() != 0) {
          if (digitalRead(limitPins[4]) == LOW || digitalRead(limitPins[5]) == LOW) { // Supone que el sensor se activa con LOW
            steppers[4].stop(); // Detiene el motor 4
            steppers[5].stop(); // Detiene el motor 5
            Serial.println("Sensor de final de carrera activado. Motores detenidos.");
            break; // Sale del bucle
          }
          steppers[4].run();
          steppers[5].run();
        }

        unsigned long endTime = millis(); // Registrar el tiempo de fin
        unsigned long elapsedTime = endTime - startTime; // Calcular el tiempo transcurrido

        Serial.print("Tiempo de movimiento: ");
        Serial.print(elapsedTime);
        Serial.println(" ms");

        Serial.println("Movimiento completado.");
      } else {
        Serial.print("Configurando motor ");
        Serial.print(motorId);
        Serial.print(" a velocidad=");
        Serial.print(velocidad);
        Serial.print(" y posición=");
        Serial.println(absolutePosition);

        steppers[motorId].setMaxSpeed(velocidad);
        steppers[motorId].setAcceleration(8 * velocidad);
        steppers[motorId].moveTo(absolutePosition);

        unsigned long startTime = millis(); // Registrar el tiempo de inicio

        while (steppers[motorId].distanceToGo() != 0) {
          if (digitalRead(limitPins[motorId]) == LOW) { // Supone que el sensor se activa con LOW
            steppers[motorId].stop(); // Detiene el motor
            Serial.println("Sensor de final de carrera activado. Motor detenido.");
            break; // Sale del bucle
          }
          steppers[motorId].run();
        }

        unsigned long endTime = millis(); // Registrar el tiempo de fin
        unsigned long elapsedTime = endTime - startTime; // Calcular el tiempo transcurrido

        Serial.print("Tiempo de movimiento: ");
        Serial.print(elapsedTime);
        Serial.println(" ms");

        Serial.println("Movimiento completado.");
      }
    } else {
      Serial.println("Error: ID de motor no valido.");
    }
  }
  delay(100);
}