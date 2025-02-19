#ifndef JWModbusMaster_h
#define JWModbusMaster_h

/**
@def __MODBUSMASTER_DEBUG__ (0)
Set to 1 to enable debugging features within class:
  - PIN A cycles for each byte read in the Modbus response
  - PIN B cycles for each millisecond timeout during the Modbus response
*/
#define __MODBUSMASTER_DEBUG__ (0)
#define __MODBUSMASTER_DEBUG_PIN_A__ 4
#define __MODBUSMASTER_DEBUG_PIN_B__ 5

#include "Arduino.h"
#include "util/crc16.h"
#include "util/word.h"

class ModbusMaster
{
public:
  ModbusMaster();
  /**
   * Initialize the ModbusMaster instance with a slave ID and a reference to a serial stream.
   * @param slave Modbus slave ID (1..255)
   * @param serial Reference to the serial stream (e.g., Serial, Serial1, etc.)
   */
  void begin(uint8_t slave, Stream &serial);

  void idle(void (*)());
  void preTransmission(void (*)());
  void postTransmission(void (*)());

  // Modbus exception codes
  /**
  Modbus protocol illegal function exception.

  The function code received in the query is not an allowable action for
  the server (or slave). This may be because the function code is only
  applicable to newer devices, and was not implemented in the unit
  selected. It could also indicate that the server (or slave) is in the
  wrong state to process a request of this type, for example because it is
  unconfigured and is being asked to return register values.

  @ingroup constant
  */
  static const uint8_t ku8MBIllegalFunction = 0x01;

  /**
  Modbus protocol illegal data address exception.

  The data address received in the query is not an allowable address for
  the server (or slave). More specifically, the combination of reference
  number and transfer length is invalid. For a controller with 100
  registers, the ADU addresses the first register as 0, and the last one
  as 99. If a request is submitted with a starting register address of 96
  and a quantity of registers of 4, then this request will successfully
  operate (address-wise at least) on registers 96, 97, 98, 99. If a
  request is submitted with a starting register address of 96 and a
  quantity of registers of 5, then this request will fail with Exception
  Code 0x02 "Illegal Data Address" since it attempts to operate on
  registers 96, 97, 98, 99 and 100, and there is no register with address
  100.

  @ingroup constant
  */
  static const uint8_t ku8MBIllegalDataAddress = 0x02;

  /**
  Modbus protocol illegal data value exception.

  A value contained in the query data field is not an allowable value for
  server (or slave). This indicates a fault in the structure of the
  remainder of a complex request, such as that the implied length is
  incorrect. It specifically does NOT mean that a data item submitted for
  storage in a register has a value outside the expectation of the
  application program, since the MODBUS protocol is unaware of the
  significance of any particular value of any particular register.

  @ingroup constant
  */
  static const uint8_t ku8MBIllegalDataValue = 0x03;

  /**
  Modbus protocol slave device failure exception.

  An unrecoverable error occurred while the server (or slave) was
  attempting to perform the requested action.

  @ingroup constant
  */
  static const uint8_t ku8MBSlaveDeviceFailure = 0x04;

  // Class-defined success/exception codes
  /**
  ModbusMaster success.

  Modbus transaction was successful; the following checks were valid:
    - slave ID
    - function code
    - response code
    - data
    - CRC

  @ingroup constant
  */
  static const uint8_t ku8MBSuccess = 0x00;

  /**
  ModbusMaster invalid response slave ID exception.

  The slave ID in the response does not match that of the request.

  @ingroup constant
  */
  static const uint8_t ku8MBInvalidSlaveID = 0xE0;

  /**
  ModbusMaster invalid response function exception.

  The function code in the response does not match that of the request.

  @ingroup constant
  */
  static const uint8_t ku8MBInvalidFunction = 0xE1;

  /**
  ModbusMaster response timed out exception.

  The entire response was not received within the timeout period,
  ModbusMaster::ku8MBResponseTimeout.

  @ingroup constant
  */
  static const uint8_t ku8MBResponseTimedOut = 0xE2;

  /**
  ModbusMaster invalid response CRC exception.

  The CRC in the response does not match the one calculated.

  @ingroup constant
  */
  static const uint8_t ku8MBInvalidCRC = 0xE3;

  // Response buffer management
  uint16_t getResponseBuffer(uint8_t index);
  void clearResponseBuffer();

  // Transmit buffer management
  uint8_t setTransmitBuffer(uint8_t index, uint16_t value);
  void clearTransmitBuffer();

  void beginTransmission(uint16_t writeAddress);
  uint8_t requestFrom(uint16_t address, uint16_t quantity);
  void sendBit(bool data);
  void send(uint8_t data);
  void send(uint16_t data);
  void send(uint32_t data);
  uint8_t available(void);
  uint16_t receive(void);

  // Modbus function commands
  uint8_t readCoils(uint16_t readAddress, uint16_t bitQty);
  uint8_t readDiscreteInputs(uint16_t readAddress, uint16_t bitQty);
  uint8_t readHoldingRegisters(uint16_t readAddress, uint16_t readQty);
  uint8_t readInputRegisters(uint16_t readAddress, uint8_t readQty);
  uint8_t writeSingleCoil(uint16_t writeAddress, uint8_t state);
  uint8_t writeSingleRegister(uint16_t writeAddress, uint16_t writeValue);
  uint8_t writeMultipleCoils(uint16_t writeAddress, uint16_t bitQty);
  uint8_t writeMultipleCoils();
  uint8_t writeMultipleRegisters(uint16_t writeAddress, uint16_t writeQty);
  uint8_t writeMultipleRegisters();
  uint8_t maskWriteRegister(uint16_t writeAddress, uint16_t andMask, uint16_t orMask);
  uint8_t readWriteMultipleRegisters(uint16_t readAddress, uint16_t readQty, uint16_t writeAddress, uint16_t writeQty);
  uint8_t readWriteMultipleRegisters(uint16_t readAddress, uint16_t readQty);

  // Modbus Helpers functions

  /**
   * @brief Helper para leer Coils.
   *
   * Llama a readCoils() y desempaqueta los bits en dataArray.
   *
   * @param readAddress Dirección inicial a leer.
   * @param readQty Cantidad de coils a leer.
   * @param dataArray Array donde se guardarán los estados de las coils.
   * @param debug Si es true, se imprime información de depuración. Valor por defecto: false.
   * @return uint8_t Código de resultado de la operación.
   */
  uint8_t H_readCoils(uint16_t readAddress, uint16_t readQty, bool *dataArray, bool debug = false);

  /**
   * @brief Helper para escribir coils.
   *
   * Dependiendo de la cantidad (coilQty), si es 1 se llama a writeSingleCoil,
   * y si es mayor a 1 se utiliza writeMultipleCoils.
   *
   * @param writeAddress Dirección inicial donde escribir (por ejemplo, 0x0000).
   * @param coilQty Cantidad de coils a escribir.
   * @param dataArray Array de bool que contiene el estado de cada coil (true = ON, false = OFF).
   * @param debug (opcional) Si es true, imprime mensajes de depuración. Valor por defecto: false.
   * @return uint8_t Código de resultado de la operación.
   */
  uint8_t H_writeCoils(uint16_t writeAddress, uint16_t coilQty, const bool *dataArray, bool debug = false);

  /**
   * @brief Lee el estado de discrete inputs (Input Status, función 0x02) desde un dispositivo Modbus.
   *
   * Esta función llama a readDiscreteInputs() de la instancia modbus, desempaqueta los bits recibidos
   * y los copia en un array de booleanos.
   *
   * @param readAddress Dirección inicial a leer (por ejemplo, 0x0000).
   * @param readQty     Cantidad de discrete inputs a leer.
   * @param dataArray   Puntero a un array de bool donde se guardarán los estados de los discrete inputs.
   * @param debug       (opcional) Si es true, se imprime información de depuración en el Monitor Serie. Valor por defecto: false.
   * @return uint8_t    Código de resultado de la operación (0 indica éxito).
   */
  uint8_t H_readDiscreteInputs(uint16_t readAddress, uint16_t readQty, bool *dataArray, bool debug = false);

  /**
   * @brief Helper para leer Holding Registers.
   *
   * Llama a readHoldingRegisters(), copia la respuesta al array dataArray e imprime
   * mensajes de debug si se solicita.
   *
   * @param readAddress Dirección inicial a leer.
   * @param readQty Cantidad de registros a leer.
   * @param dataArray Array donde se guardarán los registros.
   * @param debug Si es true, se imprime información de depuración. Valor por defecto: false.
   * @return uint8_t Código de resultado de la operación (0 indica éxito).
   */
  uint8_t H_readHoldingRegisters(uint16_t readAddress, uint8_t readQty, uint16_t *dataArray, bool debug = false);

  /**
   * @brief Escribe uno o más Holding Registers en un dispositivo Modbus.
   *
   * Si regQty es 1, se llama a writeSingleRegister; si es mayor a 1, se llama a writeMultipleRegisters.
   *
   * @param writeAddress Dirección inicial donde escribir (por ejemplo, 0x1000).
   * @param regQty       Cantidad de registros a escribir.
   * @param dataArray    Puntero a un array de uint16_t con los datos a escribir.
   * @param debug        (opcional) Si es true, se imprime información de depuración. Valor por defecto: false.
   * @return uint8_t     Código de resultado de la operación.
   */
  uint8_t H_writeHoldingRegisters(uint16_t writeAddress, uint8_t regQty, const uint16_t *dataArray, bool debug = false);

  /**
   * @brief Lee registros de entrada (Input Registers, función 0x04) desde un dispositivo Modbus.
   *
   * Esta función encapsula la llamada a readInputRegisters de la instancia modbus, copia los datos al array proporcionado
   * y, opcionalmente, imprime información en el Monitor Serie para depuración.
   *
   * @param readAddress Dirección inicial de lectura (por ejemplo, 0x1000).
   * @param readQty     Cantidad de registros a leer.
   * @param dataArray   Puntero a un array de uint16_t donde se guardarán los registros leídos.
   * @param debug       (opcional) Si es true, se imprimen mensajes de depuración. Valor por defecto: false.
   * @return uint8_t    Código de resultado de la operación (0 indica éxito).
   */
  uint8_t H_readInputRegisters(uint16_t readAddress, uint8_t readQty, uint16_t *dataArray, bool debug = false);

private:
  Stream *_serial;                            ///< Serial stream reference
  uint8_t _u8MBSlave;                         ///< Modbus slave ID
  static const uint8_t ku8MaxBufferSize = 64; ///< Maximum buffer size

  // Variables for reading
  uint16_t _u16ReadAddress;                      ///< Starting address for read operations
  uint16_t _u16ReadQty;                          ///< Quantity of words to read
  uint16_t _u16ResponseBuffer[ku8MaxBufferSize]; ///< Buffer to store response data

  // Variables for writing
  uint16_t _u16WriteAddress;                     ///< Starting address for write operations
  uint16_t _u16WriteQty;                         ///< Quantity of words to write
  uint16_t _u16TransmitBuffer[ku8MaxBufferSize]; ///< Buffer for data to be transmitted

  // Transmit buffer control:
  uint8_t _u8TransmitBufferIndex;    ///< Current index in the transmit buffer array
  uint16_t _u16TransmitBufferLength; ///< Total number of bits stored in the transmit buffer
                                     ///< (Note: For byte transmissions, this is updated as (_u8TransmitBufferIndex << 4)).

  // Response buffer control:
  uint8_t _u8ResponseBufferIndex;  ///< Current index in the response buffer
  uint8_t _u8ResponseBufferLength; ///< Total length (in words) of the response buffer

  // Modbus function codes for bit access
  static const uint8_t ku8MBReadCoils = 0x01;          ///< Modbus function 0x01 Read Coils
  static const uint8_t ku8MBReadDiscreteInputs = 0x02; ///< Modbus function 0x02 Read Discrete Inputs
  static const uint8_t ku8MBWriteSingleCoil = 0x05;    ///< Modbus function 0x05 Write Single Coil
  static const uint8_t ku8MBWriteMultipleCoils = 0x0F; ///< Modbus function 0x0F Write Multiple Coils

  // Modbus function codes for 16 bit access
  static const uint8_t ku8MBReadHoldingRegisters = 0x03;       ///< Modbus function 0x03 Read Holding Registers
  static const uint8_t ku8MBReadInputRegisters = 0x04;         ///< Modbus function 0x04 Read Input Registers
  static const uint8_t ku8MBWriteSingleRegister = 0x06;        ///< Modbus function 0x06 Write Single Register
  static const uint8_t ku8MBWriteMultipleRegisters = 0x10;     ///< Modbus function 0x10 Write Multiple Registers
  static const uint8_t ku8MBMaskWriteRegister = 0x16;          ///< Modbus function 0x16 Mask Write Register
  static const uint8_t ku8MBReadWriteMultipleRegisters = 0x17; ///< Modbus function 0x17 Read Write Multiple Registers

  // Modbus timeout [milliseconds]
  static const uint16_t ku16MBResponseTimeout = 2000; ///< Response timeout in milliseconds

  /**
   * Executes a Modbus transaction with the specified function code.
   * @param u8MBFunction Modbus function code.
   * @return Status of the transaction.
   */
  uint8_t ModbusMasterTransaction(uint8_t u8MBFunction);

  // Callback functions for handling transmission phases and idle periods.
  void (*_idle)();
  void (*_preTransmission)();
  void (*_postTransmission)();
};

#endif
