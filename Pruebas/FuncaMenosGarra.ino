#include <AccelStepper.h>
#include <Servo.h>

// Número de motores
#define NUM_MOT 6

// Pasos por revolución
#define PASOS_REV 200

// Definición de pines para cada motor
const int stepPins[NUM_MOT] = {2, 4, 8, 11, 15, 16};
const int dirPins[NUM_MOT] = {3, 5, 9, 12, 14, 17};
const int enablePins[NUM_MOT] = {6, 7, 10, 13, 18, 19};

// Relación de engranajes para cada motor
const float engrMotor[NUM_MOT] = {5, 5, 3, 3, 2, 2};

float velMax[NUM_MOT] = {360, 360, 180, 180, 180, 180};
// Velocidad máxima en grados por segundo para cada motor


// Creamos un array de objetos AccelStepper para controlar los motores
AccelStepper motores[NUM_MOT] = {
  AccelStepper(AccelStepper::DRIVER, stepPins[0], dirPins[0]),
  AccelStepper(AccelStepper::DRIVER, stepPins[1], dirPins[1]),
  AccelStepper(AccelStepper::DRIVER, stepPins[2], dirPins[2]),
  AccelStepper(AccelStepper::DRIVER, stepPins[3], dirPins[3]),
  AccelStepper(AccelStepper::DRIVER, stepPins[4], dirPins[4]),
  AccelStepper(AccelStepper::DRIVER, stepPins[5], dirPins[5])
};

// Declara el objeto para el servomotor
Servo garra;
const int pinServo = 59; // Pin del servomotor

void setup() {
  Serial.begin(9600);

   // Inicializamos el servomotor
  garra.attach(pinServo);
  // Configuramos los pines habilitadores como salida y deshabilitamos los motores inicialmente
  // for (int i = 0; i < NUM_MOT; i++) {
  //   pinMode(enablePins[i], OUTPUT);
  //   digitalWrite(enablePins[i], HIGH); // Asumiendo que activo LOW habilita el motor
  // }
  // Configuramos la velocidad máxima y aceleración de los motores
  for (int i = 0; i < NUM_MOT; i++) {
    float maxSpeed = velMax[i] * engrMotor[i] * PASOS_REV / 360.0; // Convertir grados/s a pasos/s
    motores[i].setMaxSpeed(maxSpeed);
    motores[i].setAcceleration(8 * maxSpeed); // Ajustar según sea necesario
  }

  Serial.println("Setup completo");
}

void loop() {
if (Serial.available()) {
    // Inicializar arrays y variables
    float velMax[NUM_MOT] = {360, 360, 180, 180, 180, 180};
    float posMot[NUM_MOT] = {0};
    float Id = 0, Ang = 0, Vang = 0;
    bool coordinado = true;
    bool salirFor = false; // Bandera para salir del for
    bool mover = true;
    
    // Leer y procesar posiciones desde Serial
    for (int i = 0; i < NUM_MOT && !salirFor; i++) {
        posMot[i] = Serial.parseFloat();

      if (posMot[i] == 3000) {
        mover = false;
        // Si el comando es = 3000, mover la garra
        garra.write(90); // Mover la garra a 90 grados
        Serial.println("Abrir la garra");
        salirFor = true;
      } else if (posMot[i] == 3001){
        mover = false;
        garra.write(0); // Cerrar la garra
        Serial.println("Cerrar la garra");
        salirFor = true;
      }
      
        if (mover) {
       if (posMot[i] > 1500) {

            posMot[i] -= 2000;
            // Manejar casos específicos basados en el índice
            switch (i) {
                case 0:
                    Id = posMot[i] - 1;
                    break;
                case 1:
                    Ang = posMot[i];
                    break;
                case 2:
                    Vang = posMot[i];
                    salirFor = true; // Establecer la bandera para salir del for
                    break;
                default:
                    Serial.println("Valor fuera de rango");
                    salirFor = true; // Establecer la bandera para salir del for
                    break;
            }
        }else if (posMot[i] > 500) {
            posMot[i] -= 1000;
            coordinado = false;
        } 
        }
    }

    // Solo ejecutar el siguiente for si no se ha salido prematuramente
    for (int i = 0; i < NUM_MOT && salirFor; i++) {
        if (i == Id) {
            posMot[i] = Ang;
            velMax[i] = Vang;
        } else {
            posMot[i] = 0;
        }
    }

    // Ajuste de posiciones para palanca y torsión
    float posPal = posMot[4];
    float posTor = posMot[5];
    posMot[4] = posPal - posTor;
    posMot[5] = posPal + posTor;

    // Mostrar posiciones recibidas
    Serial.print("Posiciones recibidas: ");
    for (int i = 0; i < NUM_MOT; i++) {
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print("=");
      Serial.print(posMot[i]);
      if (i < NUM_MOT - 1) Serial.print(", ");
    }
    Serial.println();

    // Variables para cálculo de movimiento
    long pasos[NUM_MOT];
    float tiempos[NUM_MOT];
    float tiempoMax = 0;

    // Calcular pasos y tiempos para cada motor
    for (int i = 0; i < NUM_MOT && salirFor; i++) {
      // Calcular pasos objetivo
      long targetSteps = (long)(posMot[i] * engrMotor[i] * PASOS_REV / 360.0);
      motores[i].moveTo(targetSteps);

      // Pasos necesarios para llegar al objetivo
      pasos[i] = motores[i].distanceToGo();

      // Calcular tiempo necesario en ms
      float velocidadMaxPasos = velMax[i] * engrMotor[i] * PASOS_REV / 360.0;
      if (velocidadMaxPasos != 0) {
        tiempos[i] = (abs(pasos[i]) / velocidadMaxPasos) * 1000.0;
      } else {
        tiempos[i] = 0;
      }
      
      // Determinar el tiempo máximo
      if (tiempos[i] > tiempoMax) {
        tiempoMax = tiempos[i];
      }
    }

    // Evitar división por cero
    if (tiempoMax == 0) tiempoMax = 1;

    // Configurar velocidad y aceleración para movimiento coordinado o individual
    if (mover) {
    for (int i = 0; i < NUM_MOT; i++) {
      float velocidad;
      if (coordinado) {
        velocidad = abs(pasos[i]) * 1000.0 / tiempoMax;
      } else {
        velocidad = velMax[i] * engrMotor[i] * PASOS_REV / 360.0;
      }
      motores[i].setMaxSpeed(velocidad);
      motores[i].setAcceleration(8 * velocidad); // Ajustar según sea necesario
    }

    }
    // Habilitar motores
    for (int i = 0; i < NUM_MOT; i++) {
      digitalWrite(enablePins[i], LOW); // Asumiendo que activo LOW habilita el motor
    }


    if (mover) {
    // Iniciar movimiento
    unsigned long tiempoInicio = millis();
    bool movimientoActivo = true;
    while (movimientoActivo) {
      movimientoActivo = false;
      for (int i = 0; i < NUM_MOT; i++) {
        if (motores[i].distanceToGo() != 0) {
          movimientoActivo = true;
          motores[i].run();
        }
      }
    }
unsigned long tiempoTotal = millis() - tiempoInicio;
        

    // Deshabilitar motores
    for (int i = 0; i < NUM_MOT; i++) {
      digitalWrite(enablePins[i], HIGH); // Deshabilitar motor
    }

    // Mostrar información final
    Serial.print("Tiempo total: ");
    Serial.print(tiempoTotal);
    Serial.println(" ms");
    }
    Serial.print("Posiciones finales: ");
    for (int i = 0; i < NUM_MOT; i++) {
      float anguloActual = motores[i].currentPosition() * 360.0 / (PASOS_REV * engrMotor[i]);
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print("=");
      Serial.print(anguloActual);
      if (i < NUM_MOT - 1) Serial.print(", ");
    }
    Serial.println("\n");
  }
}