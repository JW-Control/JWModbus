# JWModbus

JWModbus es una librería de Arduino para comunicación Modbus RTU, que implementa tanto las funciones de maestro como de esclavo. La librería incluye helpers para simplificar el manejo de datos y la interacción entre el maestro y el esclavo.

## Características

- **Modbus RTU:** Comunicación sobre RS-232/RS-485.
- **Funciones de Maestro y Esclavo:** Soporta lectura y escritura de coils, discrete inputs, holding registers e input registers.
- **Helpers (prefijados con `H_`):** Funciones que facilitan el desempaquetado y empaquetado de datos.
- **Callbacks de Pre y Post Transmisión:** Permiten controlar pines (por ejemplo, en RS-485) para habilitar o deshabilitar el transmisor.
- **Compatibilidad con Arduino:** Funciona en todas las arquitecturas compatibles.

## Instalación

1. Descarga el repositorio desde GitHub o clónalo:
   ```bash
   git clone https://github.com/JW-Control/JWModbus.git

2. Copia la carpeta JWModbus en tu carpeta de librerías de Arduino (usualmente ~/Arduino/libraries).

3. Reinicia el IDE de Arduino.


## Uso
### Maestro
En el ejemplo ExampleMaster.ino se muestra cómo configurar y utilizar el maestro para:
- Leer Holding Registers y Input Registers.
- Escribir en registros y coils.
- Usar las funciones helper para desempaquetar y empaquetar datos.
- Consulta el archivo de ejemplo en examples/ExampleMaster/ExampleMaster.ino para ver un ejemplo completo.

### Esclavo
El ejemplo ExampleSlave.ino muestra cómo:

- Inicializar el esclavo.
- Configurar áreas de memoria (holding registers, input registers, - coils, discrete inputs).
- Procesar solicitudes Modbus entrantes.
- Consulta el archivo de ejemplo en examples/ExampleSlave/ExampleSlave.ino.

## Estructura del Repositorio
El repositorio se organiza de la siguiente manera:

JWModbus/
├── examples/
│   ├── ExampleMaster/
│   │   └── ExampleMaster.ino
│   └── ExampleSlave/
│       └── ExampleSlave.ino
├── src/
│   ├── JWModbusMaster.cpp
│   ├── JWModbusMaster.h
│   ├── JWModbusSlave.cpp
│   ├── JWModbusSlave.h
│   └── util/
│       ├── crc16.h
│       └── word.h
├── keywords.txt
├── library.properties
├── LICENSE
└── README.md

## Contribuciones
¡Las contribuciones son bienvenidas! Si tienes mejoras, nuevas funcionalidades o correcciones, por favor abre un issue o envía un pull request.

## Licencia
Este proyecto está licenciado bajo la MIT License.

## Autor
JW Control - jw.control.peru@gmail.com