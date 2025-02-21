#ifndef JWModbusSlave_h
#define JWModbusSlave_h

#include <Arduino.h>
#include "util/crc16.h"
#include "util/word.h"

// Máximos permitidos para cada área de memoria
#define MODBUS_MAX_HOLDING_REGISTERS 1000
#define MODBUS_MAX_INPUT_REGISTERS 1000
#define MODBUS_MAX_COILS 1000
#define MODBUS_MAX_DISCRETE_INPUTS 1000

// Tamaños por defecto (ejemplo inspirado en PLC Siemens)
#define DEFAULT_HOLDING_REGISTERS_SIZE 100
#define DEFAULT_INPUT_REGISTERS_SIZE 100
#define DEFAULT_COILS_SIZE 200
#define DEFAULT_DISCRETE_INPUTS_SIZE 200

// Códigos de excepción Modbus
static const uint8_t MB_EX_ILLEGAL_FUNCTION = 0x01;
static const uint8_t MB_EX_ILLEGAL_DATA_ADDRESS = 0x02;
static const uint8_t MB_EX_ILLEGAL_DATA_VALUE = 0x03;

// Tipos de callbacks para la lectura/escritura de Holding Registers
typedef uint16_t (*ReadRegisterCallback)(uint16_t address);
typedef void (*WriteRegisterCallback)(uint16_t address, uint16_t value);

// Callbacks para coils
typedef bool (*ReadCoilCallback)(uint16_t address);
typedef void (*WriteCoilCallback)(uint16_t address, bool state);

// Callbacks para discrete inputs (si quieres)
typedef bool (*ReadDiscreteCallback)(uint16_t address);

// Callbacks para input registers (si quieres)
typedef uint16_t (*ReadInputRegisterCallback)(uint16_t address);

class JWModbusSlave
{
public:
  JWModbusSlave();

  /**
   * Inicializa la instancia del esclavo con un ID y el puerto serial a usar.
   *
   * @param slaveID ID del esclavo (1..255)
   * @param serial Referencia al puerto serial (por ejemplo, Serial, Serial2, etc.)
   */
  void begin(uint8_t slaveID, Stream &serial);

  /**
   * Método que se debe llamar en el loop para procesar mensajes entrantes.
   */
  void poll();

  // Setters para configurar el tamaño efectivo del mapeo (dentro de límites)
  void setHoldingRegistersSize(uint16_t size);
  void setInputRegistersSize(uint16_t size);
  void setCoilsSize(uint16_t size);
  void setDiscreteInputsSize(uint16_t size);

  uint16_t getHoldingRegistersSize() const { return _holdingRegistersSize; }
  uint16_t getInputRegistersSize() const { return _inputRegistersSize; }
  uint16_t getCoilsSize() const { return _coilsSize; }
  uint16_t getDiscreteInputsSize() const { return _discreteInputsSize; }

  // Setters para callbacks en holding registers
  void setOnReadHoldingRegister(ReadRegisterCallback callback);
  void setOnWriteHoldingRegister(WriteRegisterCallback callback);
  // Callbacks para coils
  void setOnReadCoilBit(ReadCoilCallback callback);
  void setOnWriteCoilBit(WriteCoilCallback callback);
  // (Opcional) Callbacks para discrete inputs
  void setOnReadDiscreteBit(ReadDiscreteCallback callback);
  // (Opcional) Callbacks para input registers
  void setOnReadInputRegister(ReadInputRegisterCallback callback);

  /**
   * @brief Establece la función de callback que se invocará justo antes de enviar una respuesta.
   *
   * Útil para activar un pin de dirección en módulos RS-485 que requieren habilitar el transmisor.
   * @param callback Función que se llamará antes de transmitir.
   */
  void setPreTransmission(void (*callback)());

  /**
   * @brief Establece la función de callback que se invocará justo después de enviar una respuesta.
   *
   * Útil para desactivar un pin de dirección en módulos RS-485 que requieren deshabilitar el transmisor.
   * @param callback Función que se llamará después de transmitir.
   */
  void setPostTransmission(void (*callback)());

  // Setter para activar/desactivar modo debug
  void setDebug(bool debug);

  // Áreas de memoria para simular el mapeo (se usan arrays con tamaño máximo)
  uint16_t holdingRegisters[MODBUS_MAX_HOLDING_REGISTERS];
  uint16_t inputRegisters[MODBUS_MAX_INPUT_REGISTERS];
  bool coils[MODBUS_MAX_COILS];
  bool discreteInputs[MODBUS_MAX_DISCRETE_INPUTS];

  // Nueva función para habilitar o deshabilitar el polling con FreeRTOS.
  void setFreeRTOSPoll(bool enable, UBaseType_t priority = 1, size_t stackSize = 2048, BaseType_t coreID = 1);

  // También mantenemos startPollTask como función pública
  void startPollTask(UBaseType_t priority = 1, size_t stackSize = 2048, BaseType_t coreID = 1);

private:
  Stream *_serial;
  uint8_t _slaveID;

  // Tamaños efectivos (pueden ser modificados por el usuario)
  uint16_t _holdingRegistersSize;
  uint16_t _inputRegistersSize;
  uint16_t _coilsSize;
  uint16_t _discreteInputsSize;

  // Callbacks para holding registers
  ReadRegisterCallback _onReadHolding;
  WriteRegisterCallback _onWriteHolding;
  // Callbacks para coils
  ReadCoilCallback _onReadCoilBit;
  WriteCoilCallback _onWriteCoilBit;
  // (Opcional) Discrete
  ReadDiscreteCallback _onReadDiscreteBit;
  // (Opcional) Input registers
  ReadInputRegisterCallback _onReadInputReg;

  // Callbacks para manejo de la transmisión (opcional)
  void (*_preTransmission)();
  void (*_postTransmission)();

  // Agregar la variable _debug
  bool _debug;

  // Buffer para el frame recibido (se declara volatile si se usaran interrupciones)
  volatile uint8_t _frame[256];
  volatile uint8_t _frameLength;

  // Agregamos un TaskHandle para la tarea de polling
  TaskHandle_t _pollTaskHandle;

  /**
   * Procesa el mensaje Modbus recibido.
   *
   * @param frame Puntero al frame recibido.
   * @param length Longitud del frame.
   */
  void processRequest(uint8_t *frame, uint8_t length);

  /**
   * Envía la respuesta generada.
   *
   * @param frame Puntero al frame de respuesta.
   * @param length Longitud del frame de respuesta.
   */
  void sendResponse(uint8_t *frame, uint8_t length);

  /**
   * Calcula el CRC para un frame dado.
   *
   * @param frame Puntero al frame.
   * @param length Longitud del frame.
   * @return uint16_t CRC calculado.
   */
  uint16_t calculateCRC(uint8_t *frame, uint8_t length);

  /**
   * Limpia el buffer del frame.
   */
  void clearFrame();

  /**
   * Valida el rango de la solicitud.
   *
   * @param func  Identificador de la función (ej. 0x03 para lectura de holding registers)
   * @param startAddr Dirección de inicio solicitada.
   * @param qty       Cantidad solicitada.
   * @param effectiveSize Tamaño efectivo del área (por ejemplo, _holdingRegistersSize).
   * @param maxQty    Cantidad máxima permitida para esa función.
   * @return true si el rango es válido, false en caso contrario.
   */
  bool validateRange(uint8_t func, uint16_t startAddr, uint16_t qty, uint16_t effectiveSize, uint16_t maxQty);

  void sendException(uint8_t function, uint8_t exceptionCode);

};

#endif // JWModbusSlave_h