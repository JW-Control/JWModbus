#include <JWModbusSlave.h>

// Definir pines para RS-485 (ajusta según tu hardware)
#define RX_PIN 16
#define TX_PIN 17
//#define DRIVER_ENABLE_PIN  4  // Si usas un pin para controlar el transceptor

JWModbusSlave modbusSlave;

void preTransmission() {
  // Por ejemplo, activar el transmisor:
  // digitalWrite(DRIVER_ENABLE_PIN, HIGH);
}

void postTransmission() {
  // Por ejemplo, desactivar el transmisor:
  // digitalWrite(DRIVER_ENABLE_PIN, LOW);
}

void setup() {
  Serial.begin(115200);
  // Inicia el puerto RS485 (usando Serial2, por ejemplo)
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Inicializa el esclavo con ID 1
  modbusSlave.begin(1, Serial2);
  modbusSlave.setPreTransmission(preTransmission);
  modbusSlave.setPostTransmission(postTransmission);
  
  // Pre-cargar áreas de memoria con datos de prueba
  for (uint16_t i = 0; i < 10; i++) {
    modbusSlave.holdingRegisters[i] = i * 100;  // Ejemplo: 0, 100, 200, ...
    modbusSlave.inputRegisters[i] = i * 10;       // Ejemplo: 0, 10, 20, ...
  }
  // Para los coils, alternamos ON y OFF
  for (uint16_t i = 0; i < 16; i++) {
    modbusSlave.coils[i] = (i % 2 == 0);
  }
  // Para discrete inputs, también un patrón simple
  for (uint16_t i = 0; i < 16; i++) {
    modbusSlave.discreteInputs[i] = (i % 2 != 0);
  }
  
  Serial.println("Ejemplo Slave Genérico Iniciado");
}

void loop() {
  // Procesa continuamente las solicitudes Modbus entrantes
  modbusSlave.poll();
  delay(10);
}