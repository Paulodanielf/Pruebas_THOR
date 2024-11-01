#include <Servo.h>

// Crear objeto Servo
Servo servoGarra;

// Definir constantes
const int PIN_SERVO = 42;        // Pin donde está conectado el servo
const int ANGULO_ABIERTO = 90; // Ángulo para abrir la garra (ajustar según necesidad)
const int ANGULO_CERRADO = 0;   // Ángulo para cerrar la garra (ajustar según necesidad)
const int VELOCIDAD_SERVO = 1; // Velocidad de movimiento del servo (menor = más lento)

// Variables globales
String comandoSerial = "";      // Almacena el comando recibido
bool comandoCompleto = false;   // Indica si se recibió un comando completo

void setup() {
  // Inicializar comunicación serial a 9600 baudios
  Serial.begin(9600);
  
  // Conectar el servo al pin definido
  servoGarra.attach(PIN_SERVO);
  
  // Mover el servo a la posición inicial (cerrado)
  servoGarra.write(ANGULO_CERRADO);
  
  // Mensaje de inicio
  Serial.println("Sistema de control de garra iniciado");
  Serial.println("Comandos disponibles: 'abrir' o 'cerrar'");
}

void loop() {
  // Leer comando del puerto serial
  while (Serial.available() > 0) {
    char caracterRecibido = Serial.read();
    
    // Si el caracter es un salto de línea, el comando está completo
    if (caracterRecibido == '\n') {
      comandoCompleto = true;
    } else {
      // Agregar el caracter al comando
      comandoSerial += caracterRecibido;
    }
  }
  
  // Procesar el comando si está completo
  if (comandoCompleto) {
    // Eliminar espacios en blanco al inicio y final
    comandoSerial.trim();
    
    // Convertir a minúsculas para hacer la comparación más flexible
    comandoSerial.toLowerCase();
    
    // Ejecutar el comando correspondiente
    if (comandoSerial == "abrir") {
      moverGarra(ANGULO_ABIERTO);
      Serial.println("Abriendo garra...");
    }
    else if (comandoSerial == "cerrar") {
      moverGarra(ANGULO_CERRADO);
      Serial.println("Cerrando garra...");
    }
    else {
      Serial.println("Comando no reconocido. Use 'abrir' o 'cerrar'");
    }
    
    // Limpiar el comando para la siguiente lectura
    comandoSerial = "";
    comandoCompleto = false;
  }
}

// Función para mover la garra de manera suave
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
