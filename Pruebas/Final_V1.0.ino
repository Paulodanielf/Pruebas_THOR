#include <AccelStepper.h>
#include <Servo.h>

// Número de motores
#define NUM_MOT 6

// Pasos por revolución
#define PASOS_REV 200*16

// Definición de pines para cada motor
// const int stepPins[NUM_MOT] = {2, 4, 8, 11, 15, 16};
// const int dirPins[NUM_MOT] = {3, 5, 9, 12, 14, 17};
const int stepPins[NUM_MOT] = {28, 24, 22, 23, 25, 27};
const int dirPins[NUM_MOT] = {36, 32, 30, 31, 33, 35};

#define stepB  26
#define dirB  34
AccelStepper motorB = AccelStepper(AccelStepper::DRIVER, stepB, dirB);
// const int enablePins[NUM_MOT] = {6, 7, 10, 13, 18, 19, 20};
// const int sensorPins[NUM_MOT] = {42, 44, 46, 48, 49, 47};
const int enable = 40;

// Relación de engranajes para cada motor
const float engrMotor[NUM_MOT] = {5, 5.18 * 6.716480186, 5.18*6.71, 2, 2.06042, 2.06042};

float velMax[NUM_MOT] = {1, 1, 1, 1, 1, 1};
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

// Crear objeto Servo
Servo servoGarra;
// Definir constantes
const int PIN_SERVO = 42;        // Pin donde está conectado el servo
const int ANGULO_ABIERTO = 90; // Ángulo para abrir la garra (ajustar según necesidad)
const int ANGULO_CERRADO = 45;   // Ángulo para cerrar la garra (ajustar según necesidad)
const int VELOCIDAD_SERVO = 1; // Velocidad de movimiento del servo (menor = más lento)

void moverGarra(int anguloObjetivo) {
  int anguloActual = servoGarra.read();
  
  // Determinar dirección del movimiento
  if (anguloActual < anguloObjetivo) {
    // Mover hacia la posición objetivo incrementando el ángulo
    for (int angulo = anguloActual; angulo <= anguloObjetivo; angulo += VELOCIDAD_SERVO) {
      servoGarra.write(angulo);
      delay(15); // Pequeña pausa para movimiento suave
    }
  } else {
    // Mover hacia la posición objetivo decrementando el ángulo
    for (int angulo = anguloActual; angulo >= anguloObjetivo; angulo -= VELOCIDAD_SERVO) {
      servoGarra.write(angulo);
      delay(15); // Pequeña pausa para movimiento suave
    }
  }
  
  // Asegurar que llegue exactamente al ángulo objetivo
  servoGarra.write(anguloObjetivo);
}

void setup() {
  Serial.begin(9600);

   // Conectar el servo al pin definido
  servoGarra.attach(PIN_SERVO);
  // Mover el servo a la posición inicial (abierto)
  servoGarra.write(ANGULO_ABIERTO);
  // Configuramos los pines habilitadores como salida y deshabilitamos los motores inicialmente
  pinMode(enable, OUTPUT);
  digitalWrite(enable, LOW);
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
  motorB.setMaxSpeed(velMax[1] * engrMotor[1] * PASOS_REV / 360.0);
  motorB.setAcceleration(8 * velMax[1] * engrMotor[1] * PASOS_REV / 360.0);

  Serial.println("Setup completo");
}

void loop() {
if (Serial.available()) {
  delay(50);
  Serial.println("Entro al serial");    // Inicializar arrays y variables

  // Limpiar cualquier dato residual en el buffer antes de leer
  //  while(Serial.available()) {
    //  Serial.read();
    //}

  // Esperar a que lleguen nuevos datos
    while(!Serial.available()) {
      delay(10);
    }

    float velMax[NUM_MOT] = {100, 10, 90, 10, 90, 90};
    float posMot[NUM_MOT] = {0};
    float Id = 0, Ang = 0, Vang = 0;
    bool coordinado = true;
    bool salirFor = false; // Bandera para salir del for
    bool Ind = false;

    // Leer y procesar posiciones desde Serial
    for (int i = 0; i < NUM_MOT && !salirFor; i++) {
      while(!Serial.available()) {
        delay(1);  // Esperar si los datos no están disponibles
      }
        posMot[i] = Serial.parseFloat();
      if (posMot[i] == 3000) {
        // Si el comando es >= 3000, mover la garra
        moverGarra(ANGULO_ABIERTO);
      Serial.println("Abriendo garra...");
        salirFor = true;
      } else if (posMot[i] == 3001){
        moverGarra(ANGULO_CERRADO);
      Serial.println("Cerrando garra...");
        salirFor = true;
      }

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
                    Ind = true;
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

    // Limpiar cualquier dato residual después de procesar
    while(Serial.available()) {
      Serial.read();
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
    float posPal = posMot[4]*2;
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
    for (int i = 0; i < NUM_MOT && (!salirFor || Ind); i++) {
      // Calcular pasos objetivo
      long targetSteps = (long)(posMot[i] * engrMotor[i] * PASOS_REV / 360.0);

      // Pasos necesarios para llegar al objetivo
      if(Ind){
        if(Id==i){motores[int(Id)].moveTo(targetSteps);}
      }else{
        motores[i].moveTo(targetSteps);
      }
      if (i==1) {
        Serial.println("Entro al motor B");
        motorB.moveTo(targetSteps);
        Serial.println(targetSteps);
      }

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
    for (int i = 0; i < NUM_MOT; i++) {
      float velocidad;
      if (coordinado) {
        velocidad = abs(pasos[i]) * 1000.0 / tiempoMax;
      } else {
        velocidad = velMax[i] * engrMotor[i] * PASOS_REV / 360.0;
      }
      motores[i].setMaxSpeed(velocidad);
      motores[i].setAcceleration(8 * velocidad); // Ajustar según sea necesario
      if (i==1) {
        motorB.setMaxSpeed(velocidad);
        motorB.setAcceleration(8*velocidad);
      }
    }

    // Habilitar motores
    // for (int i = 0; i < NUM_MOT; i++) {
    //   digitalWrite(enablePins[i], LOW); // Asumiendo que activo LOW habilita el motor
    // }

    // Iniciar movimiento
    unsigned long tiempoInicio = millis();
    bool movimientoActivo = true;
    while (movimientoActivo) {
      movimientoActivo = false;
      for (int i = 0; i < NUM_MOT; i++) {
        if (motores[i].distanceToGo() != 0) {
          movimientoActivo = true;
          motores[i].run();
          if (i == 1)
          {
          motorB.run();
          //Serial.print("MotorA: ");
          //Serial.println(motores[i].distanceToGo());
          //Serial.print("MotorB: ");
          //Serial.println(motorB.distanceToGo());
          }
        }
      }
    }
    unsigned long tiempoTotal = millis() - tiempoInicio;

    // Deshabilitar motores
    // for (int i = 0; i < NUM_MOT; i++) {
    //   digitalWrite(enablePins[i], HIGH); // Deshabilitar motor
    // }

    // Mostrar información final
    Serial.print("Tiempo total: ");
    Serial.print(tiempoTotal);
    Serial.println(" ms");
    Serial.print("Posiciones finales: ");
    for (int i = 0; i < NUM_MOT; i++) {
      float anguloActual = motores[i].currentPosition() * 360.0 / (PASOS_REV * engrMotor[i]);
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print("=");
      Serial.print(anguloActual);
      if (i < NUM_MOT - 1) Serial.print(",");
      
    }
    Serial.println(" MotorB:");
      Serial.print(motorB.distanceToGo());
    Serial.println("\n");
  }
}