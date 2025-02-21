#include <JWModbusMaster.h>

// Definir pines para RS-485 (ajusta según tu hardware)
#define RX_PIN 16
#define TX_PIN 17
//#define DRIVER_ENABLE_PIN  4  // Si usas un pin para controlar el transceptor

// Instanciar el objeto ModbusMaster
JWModbusMaster modbusMaster;

// Callbacks para RS485 (ajusta según tu configuración)
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
  
  // Inicializa el maestro con ID de esclavo 1
  modbusMaster.begin(1, Serial2);
  modbusMaster.preTransmission(preTransmission);
  modbusMaster.postTransmission(postTransmission);
  
  Serial.println("Ejemplo Master Genérico Iniciado");
}

void loop() {
  // Ejemplo 1: Leer Holding Registers usando la función normal
  Serial.println(">> Leyendo Holding Registers (normal)");
  uint8_t res = modbusMaster.readHoldingRegisters(0, 4);
  if (res == JWModbusMaster::ku8MBSuccess) {
    for (uint8_t i = 0; i < 4; i++) {
      Serial.print("Reg[");
      Serial.print(i);
      Serial.print("] = ");
      Serial.println(modbusMaster.getResponseBuffer(i));
    }
  } else {
    Serial.print("Error leyendo Holding Registers: 0x");
    Serial.println(res, HEX);
  }
  delay(500);

  // Ejemplo 2: Usar helper H_readCoils para leer Coils
  Serial.println(">> Leyendo Coils (helper H_readCoils)");
  bool coils[8];
  res = modbusMaster.H_readCoils(10, 8, coils, true);
  if (res != JWModbusMaster::ku8MBSuccess) {
    Serial.print("Error leyendo Coils: 0x");
    Serial.println(res, HEX);
  }
  delay(500);

  // Ejemplo 3: Escribir un único registro (Write Single Register)
  Serial.println(">> Escribiendo un único Holding Register");
  res = modbusMaster.writeSingleRegister(20, 1234);
  if (res == JWModbusMaster::ku8MBSuccess) {
    Serial.println("Escritura OK");
  } else {
    Serial.print("Error en Write Single Register: 0x");
    Serial.println(res, HEX);
  }
  delay(500);

  // Ejemplo 4: Escribir múltiples registros usando el helper H_writeHoldingRegisters
  Serial.println(">> Escribiendo múltiples Holding Registers (helper H_writeHoldingRegisters)");
  uint16_t regsToWrite[3] = { 111, 222, 333 };
  res = modbusMaster.H_writeHoldingRegisters(30, 3, regsToWrite, true);
  if (res != JWModbusMaster::ku8MBSuccess) {
    Serial.print("Error en Write Multiple Registers: 0x");
    Serial.println(res, HEX);
  }
  delay(500);

  // Ejemplo 5: Leer Input Registers usando helper H_readInputRegisters
  Serial.println(">> Leyendo Input Registers (helper H_readInputRegisters)");
  uint16_t inRegs[3];
  res = modbusMaster.H_readInputRegisters(40, 3, inRegs, true);
  if (res != JWModbusMaster::ku8MBSuccess) {
    Serial.print("Error en Read Input Registers: 0x");
    Serial.println(res, HEX);
  }
  delay(1000);
}