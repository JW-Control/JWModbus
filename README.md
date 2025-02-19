# JWModbusMaster

**JWModbusMaster** es una librería para Arduino que permite comunicarte como **Modbus Master** mediante el protocolo RTU sobre RS-485. La librería soporta las funciones básicas de Modbus (lectura y escritura de coils, registros, etc.) y añade funciones helper integradas para facilitar su uso en tus sketches.

## Características

- Soporta lectura de coils, discrete inputs, holding registers e input registers.
- Soporta escritura de coils (individual y múltiples) y registros (individual y múltiples).
- Funciones helper integradas (prefijadas con `H_`) para simplificar el llamado de las funciones, por ejemplo:
  - `H_readHoldingRegisters`
  - `H_readCoils`
  - `H_writeCoils`
- Compatibilidad con callbacks para personalizar la lectura y escritura de datos.
- Funcionalidad básica de Modbus RTU con verificación de CRC y manejo de excepciones.

## Instalación

1. Descarga o clona el repositorio.
2. Copia la carpeta `JWModbusMaster` en la carpeta de librerías de Arduino (por ejemplo, `Documents/Arduino/libraries`).
3. Reinicia el IDE de Arduino.

## Estructura de la Librería
JWModbusMaster/
├── examples/
│ ├── Example_ReadHoldingRegisters/
│ │ └── Example_ReadHoldingRegisters.ino
│ ├── Example_WriteCoils/
│ │ └── Example_WriteCoils.ino
│ └── … (otros ejemplos)
├── src/
│ ├── JWModbusMaster.h
│ ├── JWModbusMaster.cpp
│ └── (otros archivos auxiliares, si es necesario)
├── keywords.txt
├── library.properties
├── README.md
└── LICENSE (opcional, según la licencia que elijas)

## Uso

### Ejemplo: Leer Holding Registers
#include <HardwareSerial.h>
HardwareSerial Serial485(2);

const uint16_t RXD2 = 16;
const uint16_t TXD2 = 17;

#include <JWModbusMaster.h>

JWModbusMaster modbus;
unsigned long lastRead = 0;

void setup() {
  Serial.begin(115200);
  Serial485.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  // Inicializa el maestro con el ID del esclavo (por ejemplo, 1)
  modbus.begin(1, Serial485);
  
  Serial.println("ModbusMaster iniciado. Leyendo Holding Registers cada 100ms usando H_readHoldingRegisters().");
}

void loop() {
  if (millis() - lastRead >= 100) {
    lastRead = millis();
    const uint8_t numRegs = 1;
    uint16_t regs[numRegs];
    
    // Llama al helper integrado
    uint8_t result = modbus.H_readHoldingRegisters(0x0000, numRegs, regs, true);
    
    if(result == modbus.ku8MBSuccess) {
      Serial.print("Holding Register[0] = ");
      Serial.println(regs[0]);
    } else {
      Serial.print("Error en lectura, código: 0x");
      Serial.println(result, HEX);
    }
  }
  delay(10);
}

### Ejemplo: Escribir Coils
#include <HardwareSerial.h>
HardwareSerial Serial485(2);

const uint16_t RXD2 = 16;
const uint16_t TXD2 = 17;

#include <JWModbusMaster.h>

JWModbusMaster modbus;
unsigned long lastWrite = 0;

void setup() {
  Serial.begin(115200);
  Serial485.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  modbus.begin(1, Serial485);
  
  Serial.println("ModbusMaster iniciado. Escribiendo Coils cada 500ms usando H_writeCoils().");
}

void loop() {
  if (millis() - lastWrite >= 500) {
    lastWrite = millis();
    
    // Ejemplo: Escribir un patrón en 4 coils (ON, OFF, ON, OFF)
    bool coilStates[4] = { true, false, true, false };
    
    uint8_t result = modbus.H_writeCoils(0x0000, 4, coilStates, true);
    if(result == modbus.ku8MBSuccess) {
      Serial.println("Coils escritos correctamente.");
    } else {
      Serial.print("Error al escribir Coils, código: 0x");
      Serial.println(result, HEX);
    }
  }
  delay(10);
}

## Contribuciones
¡Las contribuciones son bienvenidas! Si tienes mejoras, nuevas funcionalidades o correcciones, por favor abre un issue o envía un pull request.

## Licencia
Este proyecto está licenciado bajo la MIT License.

## Autor
JW Control - jw.control.peru@gmail.com