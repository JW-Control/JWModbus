#include "JWModbusMaster.h"

JWModbusMaster::JWModbusMaster(void)
{
  _idle = 0;
  _preTransmission = 0;
  _postTransmission = 0;
}

void JWModbusMaster::begin(uint8_t slave, Stream &serial)
{
  _u8MBSlave = slave;
  _serial = &serial;
  _u8TransmitBufferIndex = 0;
  _u16TransmitBufferLength = 0;

#if __MODBUSMASTER_DEBUG__
  pinMode(__MODBUSMASTER_DEBUG_PIN_A__, OUTPUT);
  pinMode(__MODBUSMASTER_DEBUG_PIN_B__, OUTPUT);
#endif
}

/**
 * Nota: Esta función actualmente no está implementada de forma completa.
 * Se utiliza para reiniciar los índices del buffer de respuesta.
 */
uint8_t JWModbusMaster::requestFrom(uint16_t address, uint16_t quantity)
{
  // Clamp the requested quantity to the maximum buffer size.
  if (quantity > ku8MaxBufferSize)
  {
    quantity = ku8MaxBufferSize;
  }
  _u8ResponseBufferIndex = 0;
  _u8ResponseBufferLength = 0; // Se evita usar variable no inicializada.
  return 0;
}

void JWModbusMaster::beginTransmission(uint16_t u16Address)
{
  _u16WriteAddress = u16Address;
  _u8TransmitBufferIndex = 0;
  _u16TransmitBufferLength = 0;
}

void JWModbusMaster::sendBit(bool data)
{
  uint8_t txBitIndex = _u16TransmitBufferLength % 16;
  if ((_u16TransmitBufferLength >> 4) < ku8MaxBufferSize)
  {
    if (0 == txBitIndex)
    {
      _u16TransmitBuffer[_u8TransmitBufferIndex] = 0;
    }
    bitWrite(_u16TransmitBuffer[_u8TransmitBufferIndex], txBitIndex, data);
    _u16TransmitBufferLength++;
    _u8TransmitBufferIndex = _u16TransmitBufferLength >> 4;
  }
}

void JWModbusMaster::send(uint16_t data)
{
  if (_u8TransmitBufferIndex < ku8MaxBufferSize)
  {
    _u16TransmitBuffer[_u8TransmitBufferIndex++] = data;
    // Actualiza la longitud del buffer en bits.
    _u16TransmitBufferLength = _u8TransmitBufferIndex << 4;
  }
}

void JWModbusMaster::send(uint32_t data)
{
  send(lowWord(data));
  send(highWord(data));
}

void JWModbusMaster::send(uint8_t data)
{
  send(word(data));
}

uint8_t JWModbusMaster::available(void)
{
  return _u8ResponseBufferLength - _u8ResponseBufferIndex;
}

uint16_t JWModbusMaster::receive(void)
{
  if (_u8ResponseBufferIndex < _u8ResponseBufferLength)
  {
    return _u16ResponseBuffer[_u8ResponseBufferIndex++];
  }
  else
  {
    return 0xFFFF;
  }
}

void JWModbusMaster::idle(void (*idle)())
{
  _idle = idle;
}

void JWModbusMaster::preTransmission(void (*preTransmission)())
{
  _preTransmission = preTransmission;
}

void JWModbusMaster::postTransmission(void (*postTransmission)())
{
  _postTransmission = postTransmission;
}

uint16_t JWModbusMaster::getResponseBuffer(uint8_t u8Index)
{
  if (u8Index < ku8MaxBufferSize)
  {
    return _u16ResponseBuffer[u8Index];
  }
  else
  {
    return 0xFFFF;
  }
}

void JWModbusMaster::clearResponseBuffer()
{
  for (uint8_t i = 0; i < ku8MaxBufferSize; i++)
  {
    _u16ResponseBuffer[i] = 0;
  }
}

uint8_t JWModbusMaster::setTransmitBuffer(uint8_t u8Index, uint16_t u16Value)
{
  if (u8Index < ku8MaxBufferSize)
  {
    _u16TransmitBuffer[u8Index] = u16Value;
    return ku8MBSuccess;
  }
  else
  {
    return ku8MBIllegalDataAddress;
  }
}

void JWModbusMaster::clearTransmitBuffer()
{
  for (uint8_t i = 0; i < ku8MaxBufferSize; i++)
  {
    _u16TransmitBuffer[i] = 0;
  }
}

uint8_t JWModbusMaster::readCoils(uint16_t u16ReadAddress, uint16_t u16BitQty)
{
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16BitQty;
  return JWModbusMasterTransaction(ku8MBReadCoils);
}

uint8_t JWModbusMaster::readDiscreteInputs(uint16_t u16ReadAddress, uint16_t u16BitQty)
{
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16BitQty;
  return JWModbusMasterTransaction(ku8MBReadDiscreteInputs);
}

uint8_t JWModbusMaster::readHoldingRegisters(uint16_t u16ReadAddress, uint16_t u16ReadQty)
{
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16ReadQty;
  return JWModbusMasterTransaction(ku8MBReadHoldingRegisters);
}

/**
Modbus function 0x04 Read Input Registers.

This function code is used to read from 1 to 125 contiguous input
registers in a remote device. The request specifies the starting
register address and the number of registers. Registers are addressed
starting at zero.

The register data in the response buffer is packed as one word per
register.

@param u16ReadAddress address of the first input register (0x0000..0xFFFF)
@param u16ReadQty quantity of input registers to read (1..125, enforced by remote device)
@return 0 on success; exception number on failure
@ingroup register
*/
uint8_t JWModbusMaster::readInputRegisters(uint16_t u16ReadAddress,
                                         uint8_t u16ReadQty)
{
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16ReadQty;
  return JWModbusMasterTransaction(ku8MBReadInputRegisters);
}

/**
Modbus function 0x05 Write Single Coil.

This function code is used to write a single output to either ON or OFF
in a remote device. The requested ON/OFF state is specified by a
constant in the state field. A non-zero value requests the output to be
ON and a value of 0 requests it to be OFF. The request specifies the
address of the coil to be forced. Coils are addressed starting at zero.

@param u16WriteAddress address of the coil (0x0000..0xFFFF)
@param u8State 0=OFF, non-zero=ON (0x00..0xFF)
@return 0 on success; exception number on failure
@ingroup discrete
*/
uint8_t JWModbusMaster::writeSingleCoil(uint16_t u16WriteAddress, uint8_t u8State)
{
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = (u8State ? 0xFF00 : 0x0000);
  return JWModbusMasterTransaction(ku8MBWriteSingleCoil);
}

/**
Modbus function 0x06 Write Single Register.

This function code is used to write a single holding register in a
remote device. The request specifies the address of the register to be
written. Registers are addressed starting at zero.

@param u16WriteAddress address of the holding register (0x0000..0xFFFF)
@param u16WriteValue value to be written to holding register (0x0000..0xFFFF)
@return 0 on success; exception number on failure
@ingroup register
*/
uint8_t JWModbusMaster::writeSingleRegister(uint16_t u16WriteAddress,
                                          uint16_t u16WriteValue)
{
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = 0;
  _u16TransmitBuffer[0] = u16WriteValue;
  return JWModbusMasterTransaction(ku8MBWriteSingleRegister);
}

/**
Modbus function 0x0F Write Multiple Coils.

This function code is used to force each coil in a sequence of coils to
either ON or OFF in a remote device. The request specifies the coil
references to be forced. Coils are addressed starting at zero.

The requested ON/OFF states are specified by contents of the transmit
buffer. A logical '1' in a bit position of the buffer requests the
corresponding output to be ON. A logical '0' requests it to be OFF.

@param u16WriteAddress address of the first coil (0x0000..0xFFFF)
@param u16BitQty quantity of coils to write (1..2000, enforced by remote device)
@return 0 on success; exception number on failure
@ingroup discrete
*/
uint8_t JWModbusMaster::writeMultipleCoils(uint16_t u16WriteAddress, uint16_t u16BitQty)
{
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = u16BitQty;
  return JWModbusMasterTransaction(ku8MBWriteMultipleCoils);
}

uint8_t JWModbusMaster::writeMultipleCoils()
{
  _u16WriteQty = _u16TransmitBufferLength;
  return JWModbusMasterTransaction(ku8MBWriteMultipleCoils);
}

/**
Modbus function 0x10 Write Multiple Registers.

This function code is used to write a block of contiguous registers (1
to 123 registers) in a remote device.

The requested written values are specified in the transmit buffer. Data
is packed as one word per register.

@param u16WriteAddress address of the holding register (0x0000..0xFFFF)
@param u16WriteQty quantity of holding registers to write (1..123, enforced by remote device)
@return 0 on success; exception number on failure
@ingroup register
*/
uint8_t JWModbusMaster::writeMultipleRegisters(uint16_t u16WriteAddress, uint16_t u16WriteQty)
{
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = u16WriteQty;
  return JWModbusMasterTransaction(ku8MBWriteMultipleRegisters);
}

uint8_t JWModbusMaster::writeMultipleRegisters()
{
  _u16WriteQty = _u8TransmitBufferIndex;
  return JWModbusMasterTransaction(ku8MBWriteMultipleRegisters);
}

/**
Modbus function 0x16 Mask Write Register.

This function code is used to modify the contents of a specified holding
register using a combination of an AND mask, an OR mask, and the
register's current contents. The function can be used to set or clear
individual bits in the register.

The request specifies the holding register to be written, the data to be
used as the AND mask, and the data to be used as the OR mask. Registers
are addressed starting at zero.

The function's algorithm is:

Result = (Current Contents && And_Mask) || (Or_Mask && (~And_Mask))

@param u16WriteAddress address of the holding register (0x0000..0xFFFF)
@param u16AndMask AND mask (0x0000..0xFFFF)
@param u16OrMask OR mask (0x0000..0xFFFF)
@return 0 on success; exception number on failure
@ingroup register
*/
uint8_t JWModbusMaster::maskWriteRegister(uint16_t u16WriteAddress, uint16_t u16AndMask, uint16_t u16OrMask)
{
  _u16WriteAddress = u16WriteAddress;
  _u16TransmitBuffer[0] = u16AndMask;
  _u16TransmitBuffer[1] = u16OrMask;
  return JWModbusMasterTransaction(ku8MBMaskWriteRegister);
}

/**
Modbus function 0x17 Read Write Multiple Registers.

This function code performs a combination of one read operation and one
write operation in a single MODBUS transaction. The write operation is
performed before the read. Holding registers are addressed starting at
zero.

The request specifies the starting address and number of holding
registers to be read as well as the starting address, and the number of
holding registers. The data to be written is specified in the transmit
buffer.

@param u16ReadAddress address of the first holding register (0x0000..0xFFFF)
@param u16ReadQty quantity of holding registers to read (1..125, enforced by remote device)
@param u16WriteAddress address of the first holding register (0x0000..0xFFFF)
@param u16WriteQty quantity of holding registers to write (1..121, enforced by remote device)
@return 0 on success; exception number on failure
@ingroup register
*/
uint8_t JWModbusMaster::readWriteMultipleRegisters(uint16_t u16ReadAddress, uint16_t u16ReadQty, uint16_t u16WriteAddress, uint16_t u16WriteQty)
{
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16ReadQty;
  _u16WriteAddress = u16WriteAddress;
  _u16WriteQty = u16WriteQty;
  return JWModbusMasterTransaction(ku8MBReadWriteMultipleRegisters);
}

uint8_t JWModbusMaster::readWriteMultipleRegisters(uint16_t u16ReadAddress, uint16_t u16ReadQty)
{
  _u16ReadAddress = u16ReadAddress;
  _u16ReadQty = u16ReadQty;
  _u16WriteQty = _u8TransmitBufferIndex;
  return JWModbusMasterTransaction(ku8MBReadWriteMultipleRegisters);
}

/**
 * JWModbusMasterTransaction:
 * Esta función ensambla el mensaje Modbus (ADU), lo envía y espera la respuesta.
 *
 * NOTA: Se recomienda en el futuro reestructurar este bucle bloqueante a una máquina de estados
 * o utilizar tareas de FreeRTOS para mejorar el rendimiento en ESP32.
 */
uint8_t JWModbusMaster::JWModbusMasterTransaction(uint8_t u8MBFunction)
{
  uint8_t u8ModbusADU[256];
  uint8_t u8ModbusADUSize = 0;
  uint8_t i, u8Qty;
  uint16_t u16CRC;
  uint32_t u32StartTime;
  uint8_t u8BytesLeft = 8;
  uint8_t u8MBStatus = ku8MBSuccess;

  // Ensamblar el ADU de solicitud
  u8ModbusADU[u8ModbusADUSize++] = _u8MBSlave;
  u8ModbusADU[u8ModbusADUSize++] = u8MBFunction;

  switch (u8MBFunction)
  {
  case ku8MBReadCoils:
  case ku8MBReadDiscreteInputs:
  case ku8MBReadInputRegisters:
  case ku8MBReadHoldingRegisters:
  case ku8MBReadWriteMultipleRegisters:
    u8ModbusADU[u8ModbusADUSize++] = highByte(_u16ReadAddress);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16ReadAddress);
    u8ModbusADU[u8ModbusADUSize++] = highByte(_u16ReadQty);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16ReadQty);
    break;
  }

  switch (u8MBFunction)
  {
  case ku8MBWriteSingleCoil:
  case ku8MBMaskWriteRegister:
  case ku8MBWriteMultipleCoils:
  case ku8MBWriteSingleRegister:
  case ku8MBWriteMultipleRegisters:
  case ku8MBReadWriteMultipleRegisters:
    u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteAddress);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteAddress);
    break;
  }

  switch (u8MBFunction)
  {
  case ku8MBWriteSingleCoil:
    u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteQty);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty);
    break;

  case ku8MBWriteSingleRegister:
    u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[0]);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[0]);
    break;

  case ku8MBWriteMultipleCoils:
    u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteQty);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty);
    u8Qty = (_u16WriteQty % 8) ? ((_u16WriteQty >> 3) + 1) : (_u16WriteQty >> 3);
    u8ModbusADU[u8ModbusADUSize++] = u8Qty;
    for (i = 0; i < u8Qty; i++)
    {
      // Cada byte se forma a partir de 2 palabras (orden bajo/alto)
      if (i % 2 == 0) // índice par
      {
        u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[i >> 1]);
      }
      else // índice impar
      {
        u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[i >> 1]);
      }
    }
    break;

  case ku8MBWriteMultipleRegisters:
  case ku8MBReadWriteMultipleRegisters:
    u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteQty);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty << 1);
    for (i = 0; i < lowByte(_u16WriteQty) && i < _u8TransmitBufferIndex; i++)
    {
      u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[i]);
      u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[i]);
    }
    break;

  case ku8MBMaskWriteRegister:
    u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[0]);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[0]);
    u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[1]);
    u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[1]);
    break;
  }

  // Calcular y agregar el CRC
  u16CRC = 0xFFFF;
  for (i = 0; i < u8ModbusADUSize; i++)
  {
    u16CRC = crc16_update(u16CRC, u8ModbusADU[i]);
  }
  u8ModbusADU[u8ModbusADUSize++] = lowByte(u16CRC);
  u8ModbusADU[u8ModbusADUSize++] = highByte(u16CRC);
  u8ModbusADU[u8ModbusADUSize] = 0;

  // Vaciar el buffer de recepción antes de transmitir
  while (_serial->read() != -1)
    ;

  // Transmitir la solicitud
  if (_preTransmission)
  {
    _preTransmission();
  }
  for (i = 0; i < u8ModbusADUSize; i++)
  {
    _serial->write(u8ModbusADU[i]);
  }

  u8ModbusADUSize = 0;
  _serial->flush(); // Asegura que se envíen todos los datos
  if (_postTransmission)
  {
    _postTransmission();
  }

  // Bucle de espera para la respuesta (bloqueante)
  u32StartTime = millis();
  while (u8BytesLeft && !u8MBStatus)
  {
    if (_serial->available())
    {
#if __MODBUSMASTER_DEBUG__
      digitalWrite(__MODBUSMASTER_DEBUG_PIN_A__, true);
#endif
      u8ModbusADU[u8ModbusADUSize++] = _serial->read();
      u8BytesLeft--;
#if __MODBUSMASTER_DEBUG__
      digitalWrite(__MODBUSMASTER_DEBUG_PIN_A__, false);
#endif
    }
    else
    {
#if __MODBUSMASTER_DEBUG__
      digitalWrite(__MODBUSMASTER_DEBUG_PIN_B__, true);
#endif
      if (_idle)
      {
        _idle();
      }
#if __MODBUSMASTER_DEBUG__
      digitalWrite(__MODBUSMASTER_DEBUG_PIN_B__, false);
#endif
    }

    // Una vez leídos los primeros 5 bytes, se realizan comprobaciones iniciales
    if (u8ModbusADUSize == 5)
    {
      if (u8ModbusADU[0] != _u8MBSlave)
      {
        u8MBStatus = ku8MBInvalidSlaveID;
        break;
      }

      if ((u8ModbusADU[1] & 0x7F) != u8MBFunction)
      {
        u8MBStatus = ku8MBInvalidFunction;
        break;
      }

      if (bitRead(u8ModbusADU[1], 7))
      {
        u8MBStatus = u8ModbusADU[2];
        break;
      }

      switch (u8ModbusADU[1])
      {
      case ku8MBReadCoils:
      case ku8MBReadDiscreteInputs:
      case ku8MBReadInputRegisters:
      case ku8MBReadHoldingRegisters:
      case ku8MBReadWriteMultipleRegisters:
        u8BytesLeft = u8ModbusADU[2];
        break;

      case ku8MBWriteSingleCoil:
      case ku8MBWriteMultipleCoils:
      case ku8MBWriteSingleRegister:
      case ku8MBWriteMultipleRegisters:
        u8BytesLeft = 3;
        break;

      case ku8MBMaskWriteRegister:
        u8BytesLeft = 5;
        break;
      }
    }
    if ((millis() - u32StartTime) > ku16MBResponseTimeout)
    {
      u8MBStatus = ku8MBResponseTimedOut;
    }
  }

  // Verificar que la respuesta tenga el tamaño mínimo esperado
  if (!u8MBStatus && u8ModbusADUSize >= 5)
  {
    u16CRC = 0xFFFF;
    for (i = 0; i < (u8ModbusADUSize - 2); i++)
    {
      u16CRC = crc16_update(u16CRC, u8ModbusADU[i]);
    }

    if ((lowByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 2]) ||
        (highByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 1]))
    {
      u8MBStatus = ku8MBInvalidCRC;
    }
  }

  // Desensamblar el ADU en el buffer de respuesta según la función
  if (!u8MBStatus)
  {
    switch (u8ModbusADU[1])
    {
    case ku8MBReadCoils:
    case ku8MBReadDiscreteInputs:
    {
      uint8_t byteCount = u8ModbusADU[2];
      uint8_t wordCount = byteCount >> 1;
      for (i = 0; i < wordCount && i < ku8MaxBufferSize; i++)
      {
        _u16ResponseBuffer[i] = word(u8ModbusADU[2 * i + 4], u8ModbusADU[2 * i + 3]);
      }
      // Si hay un byte sobrante, se almacena en la siguiente posición
      if (byteCount % 2 && wordCount < ku8MaxBufferSize)
      {
        _u16ResponseBuffer[wordCount] = word(0, u8ModbusADU[2 * wordCount + 3]);
        _u8ResponseBufferLength = wordCount + 1;
      }
      else
      {
        _u8ResponseBufferLength = wordCount;
      }
      break;
    }
    case ku8MBReadInputRegisters:
    case ku8MBReadHoldingRegisters:
    case ku8MBReadWriteMultipleRegisters:
    {
      uint8_t wordCount = u8ModbusADU[2] >> 1;
      for (i = 0; i < wordCount && i < ku8MaxBufferSize; i++)
      {
        _u16ResponseBuffer[i] = word(u8ModbusADU[2 * i + 3], u8ModbusADU[2 * i + 4]);
      }
      _u8ResponseBufferLength = wordCount;
      break;
    }
    }
  }

  // Reiniciar índices para la próxima transacción
  _u8TransmitBufferIndex = 0;
  _u16TransmitBufferLength = 0;
  _u8ResponseBufferIndex = 0;
  return u8MBStatus;
}

// Helper para leer Coils (función 0x01)
uint8_t JWModbusMaster::H_readCoils(uint16_t readAddress, uint16_t readQty, bool *dataArray, bool debug)
{
  uint8_t result = readCoils(readAddress, readQty);
  if (result == ku8MBSuccess)
  {
    uint16_t wordCount = available();
    uint16_t coilIndex = 0;
    for (uint16_t i = 0; i < wordCount; i++)
    {
      uint16_t wordVal = getResponseBuffer(i);
      for (uint8_t bit = 0; bit < 16; bit++)
      {
        if (coilIndex < readQty)
        {
          dataArray[coilIndex] = ((wordVal >> bit) & 0x01) != 0;
          coilIndex++;
        }
        else
        {
          break;
        }
      }
    }
    if (debug)
    {
      Serial.printf("[H_readCoils] Coils leídos (0x%04X .. 0x%04X):\n", readAddress, readAddress + readQty - 1);
      for (uint16_t i = 0; i < readQty; i++)
      {
        Serial.printf("  Índice %u -> %s\n", i, dataArray[i] ? "ON" : "OFF");
      }
    }
  }
  else
  {
    if (debug)
    {
      Serial.printf("[H_readCoils] Error 0x%02X al leer (0x%04X .. 0x%04X)\n", result, readAddress, readAddress + readQty - 1);
    }
  }
  return result;
}

// Helper para escribir Coils (funciones 0x05 y 0x0F)
uint8_t JWModbusMaster::H_writeCoils(uint16_t writeAddress, uint16_t coilQty, const bool *dataArray, bool debug)
{
  uint8_t result;
  if (coilQty == 1)
  {
    // Para un solo coil, usamos writeSingleCoil
    // Se define: 0xFF00 para ON, 0x0000 para OFF.
    uint16_t state = dataArray[0] ? 0xFF00 : 0x0000;
    result = writeSingleCoil(writeAddress, state);
    if (debug)
    {
      if (result == ku8MBSuccess)
        Serial.printf("[H_writeCoils] writeSingleCoil(0x%04X, %s) OK\n", writeAddress, dataArray[0] ? "ON" : "OFF");
      else
        Serial.printf("[H_writeCoils] Error 0x%02X en writeSingleCoil(0x%04X, %s)\n", result, writeAddress, dataArray[0] ? "ON" : "OFF");
    }
  }
  else
  {
    // Para múltiples coils, iniciamos la transmisión, enviamos cada bit y llamamos a writeMultipleCoils.
    beginTransmission(writeAddress);
    for (uint16_t i = 0; i < coilQty; i++)
    {
      sendBit(dataArray[i]);
    }
    result = writeMultipleCoils();
    if (debug)
    {
      if (result == ku8MBSuccess)
        Serial.printf("[H_writeCoils] writeMultipleCoils(0x%04X, qty=%u) OK\n", writeAddress, coilQty);
      else
        Serial.printf("[H_writeCoils] Error 0x%02X en writeMultipleCoils(0x%04X, qty=%u)\n", result, writeAddress, coilQty);
    }
  }
  return result;
}

// Helper para leer Discrete Inputs (función 0x02)
uint8_t JWModbusMaster::H_readDiscreteInputs(uint16_t readAddress, uint16_t readQty, bool *dataArray, bool debug)
{
  uint8_t result = readDiscreteInputs(readAddress, readQty);
  if (result == ku8MBSuccess)
  {
    uint16_t wordCount = available();
    uint16_t inputIndex = 0;
    for (uint16_t i = 0; i < wordCount; i++)
    {
      uint16_t wordVal = getResponseBuffer(i);
      for (uint8_t bit = 0; bit < 16; bit++)
      {
        if (inputIndex < readQty)
        {
          dataArray[inputIndex] = ((wordVal >> bit) & 0x01) != 0;
          inputIndex++;
        }
        else
        {
          break;
        }
      }
    }
    if (debug)
    {
      Serial.printf("[H_readDiscreteInputs] Discrete Inputs leídos (0x%04X .. 0x%04X):\n", readAddress, readAddress + readQty - 1);
      for (uint16_t i = 0; i < readQty; i++)
      {
        Serial.printf("  Índice %u -> %s\n", i, dataArray[i] ? "ON" : "OFF");
      }
    }
  }
  else
  {
    if (debug)
    {
      Serial.printf("[H_readDiscreteInputs] Error 0x%02X al leer (0x%04X .. 0x%04X)\n", result, readAddress, readAddress + readQty - 1);
    }
  }
  return result;
}

// Helper para leer Holding Registers (función 0x03)
uint8_t JWModbusMaster::H_readHoldingRegisters(uint16_t readAddress, uint8_t readQty, uint16_t *dataArray, bool debug)
{
  uint8_t result = readHoldingRegisters(readAddress, readQty);
  if (result == ku8MBSuccess)
  {
    for (uint8_t i = 0; i < readQty; i++)
    {
      dataArray[i] = getResponseBuffer(i);
    }
    if (debug)
    {
      Serial.printf("[H_readHoldingRegisters] Registros (0x%04X .. 0x%04X):\n", readAddress, readAddress + readQty - 1);
      for (uint8_t i = 0; i < readQty; i++)
      {
        Serial.printf("  Índice %u -> 0x%04X\n", i, dataArray[i]);
      }
    }
  }
  else
  {
    if (debug)
    {
      Serial.printf("[H_readHoldingRegisters] Error 0x%02X al leer (0x%04X .. 0x%04X)\n", result, readAddress, readAddress + readQty - 1);
    }
  }
  return result;
}

// Helper para escribir Holding Registers (funciones 0x06 y 0x10)
uint8_t JWModbusMaster::H_writeHoldingRegisters(uint16_t writeAddress, uint8_t regQty, const uint16_t *dataArray, bool debug)
{
  uint8_t result;
  if (regQty == 1)
  {
    result = writeSingleRegister(writeAddress, dataArray[0]);
    if (debug)
    {
      if (result == ku8MBSuccess)
        Serial.printf("[H_writeHoldingRegisters] writeSingleRegister(0x%04X, 0x%04X) OK\n", writeAddress, dataArray[0]);
      else
        Serial.printf("[H_writeHoldingRegisters] Error 0x%02X en writeSingleRegister(0x%04X, 0x%04X)\n", result, writeAddress, dataArray[0]);
    }
  }
  else
  {
    beginTransmission(writeAddress);
    for (uint8_t i = 0; i < regQty; i++)
    {
      send(dataArray[i]);
    }
    result = writeMultipleRegisters();
    if (debug)
    {
      if (result == ku8MBSuccess)
        Serial.printf("[H_writeHoldingRegisters] writeMultipleRegisters(0x%04X, qty=%u) OK\n", writeAddress, regQty);
      else
        Serial.printf("[H_writeHoldingRegisters] Error 0x%02X en writeMultipleRegisters(0x%04X, qty=%u)\n", result, writeAddress, regQty);
    }
  }
  return result;
}

// Helper para leer Input Registers (función 0x04)
uint8_t JWModbusMaster::H_readInputRegisters(uint16_t readAddress, uint8_t readQty, uint16_t *dataArray, bool debug)
{
  uint8_t result = readInputRegisters(readAddress, readQty);
  if (result == ku8MBSuccess)
  {
    for (uint8_t i = 0; i < readQty; i++)
    {
      dataArray[i] = getResponseBuffer(i);
    }
    if (debug)
    {
      Serial.printf("[H_readInputRegisters] Input Registers (0x%04X .. 0x%04X):\n", readAddress, readAddress + readQty - 1);
      for (uint8_t i = 0; i < readQty; i++)
      {
        Serial.printf("  Índice %u -> 0x%04X\n", i, dataArray[i]);
      }
    }
  }
  else
  {
    if (debug)
    {
      Serial.printf("[H_readInputRegisters] Error 0x%02X al leer (0x%04X .. 0x%04X)\n", result, readAddress, readAddress + readQty - 1);
    }
  }
  return result;
}