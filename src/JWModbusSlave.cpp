#include "JWModbusSlave.h"

JWModbusSlave::JWModbusSlave() : _serial(nullptr), _slaveID(0), _frameLength(0),
                             _holdingRegistersSize(DEFAULT_HOLDING_REGISTERS_SIZE),
                             _inputRegistersSize(DEFAULT_INPUT_REGISTERS_SIZE),
                             _coilsSize(DEFAULT_COILS_SIZE),
                             _discreteInputsSize(DEFAULT_DISCRETE_INPUTS_SIZE),
                             _onReadHolding(nullptr), _onWriteHolding(nullptr),
                             _onReadCoilBit(nullptr), _onWriteCoilBit(nullptr),
                             _onReadDiscreteBit(nullptr), _onReadInputReg(nullptr),
                             _debug(false), _pollTaskHandle(NULL),
                             _preTransmission(nullptr), _postTransmission(nullptr)
{
    memset(holdingRegisters, 0, sizeof(holdingRegisters));
    memset(inputRegisters, 0, sizeof(inputRegisters));
    memset(coils, 0, sizeof(coils));
    memset(discreteInputs, 0, sizeof(discreteInputs));
}

void JWModbusSlave::setDebug(bool debug)
{
    _debug = debug;
}

void JWModbusSlave::begin(uint8_t slaveID, Stream &serial)
{
    _slaveID = slaveID;
    _serial = &serial;
}

uint16_t JWModbusSlave::calculateCRC(uint8_t *frame, uint8_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; i++)
    {
        crc = crc16_update(crc, frame[i]);
    }
    return crc;
}

void JWModbusSlave::sendResponse(uint8_t *frame, uint8_t length)
{
    // Agrega el CRC al final del frame
    uint16_t crc = calculateCRC(frame, length);
    frame[length++] = lowByte(crc);
    frame[length++] = highByte(crc);

    
    // Llama al callback pretransmission, si está definido
    if (_preTransmission)
    {
        _preTransmission();
    }

    // Envía la respuesta
    for (uint8_t i = 0; i < length; i++)
    {
        _serial->write(frame[i]);
    }
    _serial->flush();

    // Llama al callback posttransmission, si está definido
    if (_postTransmission)
    {
        _postTransmission();
    }

    if (_debug)
    {
        Serial.printf("[ModbusSlave] Respuesta enviada (%u bytes)\n", length);
    }
}

void JWModbusSlave::clearFrame()
{
    noInterrupts();
    _frameLength = 0;
    interrupts();
}

bool JWModbusSlave::validateRange(uint8_t func, uint16_t startAddr, uint16_t qty, uint16_t effectiveSize, uint16_t maxQty)
{
    // Si qty excede maxQty, lo consideramos "Illegal Data Value" (0x03)
    if (qty < 1 || qty > maxQty)
    {
        if (_debug)
        {
            Serial.printf("[%s] -> Cantidad fuera de rango: %u (debe estar entre 1 y %u)\n", qty, maxQty);
        }
        sendException(func, MB_EX_ILLEGAL_DATA_VALUE);
        return false;
    }
    // Si (startAddr + qty) > effectiveSize, es "Illegal Data Address" (0x02)
    if ((startAddr + qty) > effectiveSize)
    {
        if (_debug)
        {
            Serial.printf("[%s] -> Dirección fuera de rango: startAddr=%u, qty=%u, effectiveSize=%u\n", startAddr, qty, effectiveSize);
        }
        sendException(func, MB_EX_ILLEGAL_DATA_ADDRESS);
        return false;
    }
    return true;
}

void JWModbusSlave::sendException(uint8_t function, uint8_t exceptionCode)
{
    // Frame de excepción: 3 bytes + 2 de CRC
    // [ID][Func|0x80][ExceptionCode]
    // Luego se completa con CRC en sendResponse()
    uint8_t response[3];
    response[0] = _slaveID;
    response[1] = function | 0x80; // bit 7 en 1 indica excepción
    response[2] = exceptionCode;

    sendResponse(response, 3); // 3 bytes + 2 de CRC => 5 bytes totales
}

void JWModbusSlave::processRequest(uint8_t *frame, uint8_t length)
{
    // Verifica longitud mínima
    if (length < 8)
    {
        if (_debug)
            Serial.println("[ModbusSlave] Frame demasiado corto, ignorando");
        return;
    }

    uint16_t crcReceived = word(frame[length - 1], frame[length - 2]);
    uint16_t crcCalc = calculateCRC(frame, length - 2);
    if (crcReceived != crcCalc)
    {
        if (_debug)
            Serial.println("[ModbusSlave] CRC inválido, ignorando frame");
        return;
    }

    // Verifica que el mensaje sea para este esclavo
    if (frame[0] != _slaveID)
    {
        if (_debug)
        {
            Serial.printf("[ModbusSlave] ID de esclavo no coincide (%u != %u), ignorando\n", frame[0], _slaveID);
        }
        return;
    }

    uint8_t func = frame[1];
    uint8_t response[256];
    uint8_t responseLen = 0; // Variable para almacenar la longitud de la respuesta

    if (_debug)
    {
        Serial.printf("[ModbusSlave] Recibido ID=%u, Func=0x%02X\n", frame[0], func);
    }
    switch (func)
    {
    case 0x01: // Read Coils
    {
        uint16_t startAddr = word(frame[2], frame[3]);
        uint16_t qty = word(frame[4], frame[5]);

        if (!validateRange(func, startAddr, qty, _coilsSize, 125)) // Podrías definir otro máximo para coils
            return;

        response[0] = _slaveID;
        response[1] = 0x01;
        uint8_t byteCount = (qty % 8) ? (qty / 8 + 1) : (qty / 8);
        response[2] = byteCount;
        for (uint8_t i = 0; i < byteCount; i++)
        {
            uint8_t b = 0;
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                uint16_t index = startAddr + i * 8 + bit;
                bool state = false;
                if (index < _coilsSize)
                {
                    if (_onReadCoilBit)
                        state = _onReadCoilBit(index);
                    else
                        state = coils[index];
                }
                b |= (state ? 1 : 0) << bit;
            }
            response[3 + i] = b;
        }
        responseLen = 3 + byteCount;
        break;
    }
    case 0x02: // Read Discrete Inputs
    {
        uint16_t startAddr = word(frame[2], frame[3]);
        uint16_t qty = word(frame[4], frame[5]);

        if (!validateRange(func, startAddr, qty, _discreteInputsSize, 125))
            return;

        response[0] = _slaveID;
        response[1] = 0x02;
        uint8_t byteCount = (qty % 8) ? (qty / 8 + 1) : (qty / 8);
        response[2] = byteCount;
        for (uint8_t i = 0; i < byteCount; i++)
        {
            uint8_t b = 0;
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                uint16_t index = startAddr + i * 8 + bit;
                bool state = false;
                if (index < _discreteInputsSize)
                {
                    if (_onReadDiscreteBit)
                        state = _onReadDiscreteBit(index);
                    else
                        state = discreteInputs[index];
                }
                b |= (state ? 1 : 0) << bit;
            }
            response[3 + i] = b;
        }
        responseLen = 3 + byteCount;
        break;
    }
    case 0x03: // Read Holding Registers
    {
        uint16_t startAddr = word(frame[2], frame[3]);
        uint16_t qty = word(frame[4], frame[5]);

        // Por ejemplo, en Modbus RTU se limita a 125 registros en una sola lectura.
        if (!validateRange(func, startAddr, qty, _holdingRegistersSize, 125))
            return;

        // 3) Calcula el byteCount
        uint8_t byteCount = qty * 2;
        // 4) Verifica que la respuesta quepa en 'response'
        if ((3 + byteCount) > (sizeof(response) - 2))
        {
            if (_debug)
            {
                Serial.printf("[ModbusSlave] -> Respuesta muy grande para el buffer: 3 + %u = %u bytes, buffer=%u\n",
                              byteCount, (3 + byteCount), (unsigned)sizeof(response));
            }
            return;
        }

        // Arma la respuesta
        response[0] = _slaveID;
        response[1] = 0x03;
        response[2] = byteCount;
        for (uint16_t i = 0; i < qty; i++)
        {
            uint16_t regVal = (_onReadHolding) ? _onReadHolding(startAddr + i) : holdingRegisters[startAddr + i];
            response[3 + i * 2] = highByte(regVal);
            response[4 + i * 2] = lowByte(regVal);
        }
        responseLen = 3 + byteCount;
        break;
    }
    case 0x04: // Read Input Registers
    {
        uint16_t startAddr = word(frame[2], frame[3]);
        uint16_t qty = word(frame[4], frame[5]);

        if (!validateRange(func, startAddr, qty, _inputRegistersSize, 125))
            return;

        response[0] = _slaveID;
        response[1] = 0x04;
        response[2] = qty * 2;
        for (uint16_t i = 0; i < qty; i++)
        {
            uint16_t addr = startAddr + i;
            uint16_t reg = 0;
            if (addr < _inputRegistersSize)
            {
                if (_onReadInputReg)
                    reg = _onReadInputReg(addr);
                else
                    reg = inputRegisters[addr];
            }
            response[3 + i * 2] = highByte(reg);
            response[3 + i * 2 + 1] = lowByte(reg);
        }
        responseLen = 3 + qty * 2;
        break;
    }
    case 0x05: // Write Single Coil
    {
        uint16_t addr = word(frame[2], frame[3]);
        uint16_t value = word(frame[4], frame[5]); // 0xFF00 = ON, 0x0000 = OFF

        // Validación: la coil se cuenta de a 1, y la “cantidad” aquí es 1
        // maxQty puede ser _coilsSize, p. ej.
        if (!validateRange(func, addr, 1, _coilsSize, _coilsSize))
        {
            return;
        }

        bool coilState = (value == 0xFF00);
        if (_onWriteCoilBit)
            _onWriteCoilBit(addr, coilState);
        else
            coils[addr] = coilState;

        // La respuesta es una réplica de la solicitud
        memcpy(response, frame, 6);
        responseLen = 6;
        break;
    }
    case 0x06: // Write Single Register
    {
        uint16_t addr = word(frame[2], frame[3]);
        uint16_t value = word(frame[4], frame[5]);

        if (!validateRange(func, addr, 1, _holdingRegistersSize, _holdingRegistersSize))
            return;

        if (addr < _holdingRegistersSize)
        {
            holdingRegisters[addr] = value;
            if (_onWriteHolding)
            {
                _onWriteHolding(addr, value);
            }
        }
        // La respuesta es una réplica de la solicitud
        memcpy(response, frame, 6);
        responseLen = 6;
        break;
    }
    case 0x0F: // Write Multiple Coils
    {
        uint16_t startAddr = word(frame[2], frame[3]);
        uint16_t qty = word(frame[4], frame[5]);
        uint8_t byteCount = frame[6];

        // Validar rango
        // Para coils, podrías usar un límite de 0..1968 bits (p.ej. 123 coils?),
        // pero en general, el estándar Modbus RTU sugiere un máximo de 0x07D0 (2000) bits
        if (!validateRange(func, startAddr, qty, _coilsSize, 2000))
        {
            return;
        }

        // Verificar que la cantidad de bytes concuerde con la cantidad de coils
        // (qty + 7)/8 == byteCount
        uint8_t expectedBytes = (qty % 8) ? (qty / 8 + 1) : (qty / 8);
        if (byteCount != expectedBytes)
        {
            if (_debug)
            {
                Serial.printf("[0x0F] -> ByteCount inconsistente: %u vs esperado %u\n", byteCount, expectedBytes);
            }
            return;
        }

        // Actualizar las coils en la memoria
        // Los bits están en frame[7..(7+byteCount-1)]
        uint8_t coilDataIndex = 7;
        for (uint16_t i = 0; i < qty; i++)
        {
            uint16_t bitIndex = i % 8;
            uint16_t byteOffset = i / 8;
            uint8_t b = frame[coilDataIndex + byteOffset];
            bool state = (b & (1 << bitIndex)) != 0;
            if ((startAddr + i) < _coilsSize)
            {
                if (_onWriteCoilBit)
                    _onWriteCoilBit(startAddr + i, state);
                else
                    coils[startAddr + i] = state;
            }
        }

        // La respuesta: 6 bytes (ID, Func, startAddrHi, startAddrLo, qtyHi, qtyLo)
        // luego se agregan 2 bytes de CRC
        response[0] = _slaveID;
        response[1] = 0x0F;
        response[2] = highByte(startAddr);
        response[3] = lowByte(startAddr);
        response[4] = highByte(qty);
        response[5] = lowByte(qty);

        responseLen = 6;
        break;
    }
    case 0x10: // Write Multiple Registers
    {
        uint16_t startAddr = word(frame[2], frame[3]);
        uint16_t qty = word(frame[4], frame[5]);
        uint8_t byteCount = frame[6];

        // Validar rango
        if (!validateRange(func, startAddr, qty, _holdingRegistersSize, 123))
        {
            // Típicamente, 123 regs es el máximo para 0x10
            return;
        }

        // Verificar que byteCount == qty*2
        if (byteCount != (qty * 2))
        {
            if (_debug)
            {
                Serial.printf("[0x10] -> ByteCount inconsistente: %u vs esperado %u\n", byteCount, qty * 2);
            }
            return;
        }

        // Actualizar los holding registers
        // Los datos empiezan en frame[7]
        uint8_t dataIndex = 7;
        for (uint16_t i = 0; i < qty; i++)
        {
            uint16_t hi = frame[dataIndex + i * 2];
            uint16_t lo = frame[dataIndex + i * 2 + 1];
            uint16_t value = (hi << 8) | lo;
            uint16_t addr = startAddr + i;
            if (addr < _holdingRegistersSize)
            {
                holdingRegisters[addr] = value;
                if (_onWriteHolding)
                {
                    _onWriteHolding(addr, value);
                }
            }
        }

        // La respuesta: 6 bytes (ID, Func=0x10, startAddrHi, startAddrLo, qtyHi, qtyLo)
        response[0] = _slaveID;
        response[1] = 0x10;
        response[2] = highByte(startAddr);
        response[3] = lowByte(startAddr);
        response[4] = highByte(qty);
        response[5] = lowByte(qty);

        responseLen = 6;
        break;
    }
    case 0x16: // Mask Write Register
    {
        // Formato de solicitud:
        // [ID][0x16][AddrHi][AddrLo][AndHi][AndLo][OrHi][OrLo] + CRC

        uint16_t addr = word(frame[2], frame[3]);
        uint16_t andMask = word(frame[4], frame[5]);
        uint16_t orMask = word(frame[6], frame[7]);

        // Validar que el addr esté dentro de los holding registers
        // Usamos qty=1 (solo 1 registro), y un máximo p.ej. = _holdingRegistersSize
        if (!validateRange(func, addr, 1, _holdingRegistersSize, _holdingRegistersSize))
        {
            return; // Ya envía excepción
        }

        // Aplicar la operación de enmascarado
        if (addr < _holdingRegistersSize)
        {
            uint16_t currentVal = holdingRegisters[addr];
            uint16_t newVal = (currentVal & andMask) | (orMask & ~andMask);
            holdingRegisters[addr] = newVal;

            if (_onWriteHolding)
            {
                // Llamamos al callback para avisar del cambio
                _onWriteHolding(addr, newVal);
            }
        }

        // La respuesta es una réplica de la solicitud de 8 bytes (ID..OrLo)
        // y luego se añaden 2 bytes de CRC al final
        memcpy(response, frame, 8);
        responseLen = 8;
        break;
    }
    case 0x17:
    {
        // Read/Write Multiple Registers
        // Solicitud:
        // [ID][0x17][ReadAddrHi][ReadAddrLo][ReadQtyHi][ReadQtyLo]
        //        [WriteAddrHi][WriteAddrLo][WriteQtyHi][WriteQtyLo]
        //        [ByteCount] [Write Data ... ] + CRC
        //
        // Respuesta:
        // [ID][0x17][ByteCount][Read Data ...] + CRC

        uint16_t readAddr = word(frame[2], frame[3]);
        uint16_t readQty = word(frame[4], frame[5]);
        uint16_t writeAddr = word(frame[6], frame[7]);
        uint16_t writeQty = word(frame[8], frame[9]);
        uint8_t writeByteCount = frame[10];

        // Validar rango para la lectura (max 125) y la escritura (max 121 según estándar)
        if (!validateRange(func, readAddr, readQty, _holdingRegistersSize, 125))
        {
            return;
        }
        if (!validateRange(func, writeAddr, writeQty, _holdingRegistersSize, 121))
        {
            return;
        }

        // Verificar que writeByteCount == writeQty * 2
        if (writeByteCount != (writeQty * 2))
        {
            if (_debug)
            {
                Serial.printf("[ModbusSlave] (0x17) ByteCount inconsistente\n");
            }
            sendException(func, MB_EX_ILLEGAL_DATA_VALUE);
            return;
        }

        // 1) Escribir primero los WriteQty registros
        //   Datos empiezan en frame[11..]
        uint8_t dataIndex = 11;
        for (uint16_t i = 0; i < writeQty; i++)
        {
            uint16_t hi = frame[dataIndex + i * 2];
            uint16_t lo = frame[dataIndex + i * 2 + 1];
            uint16_t value = (hi << 8) | lo;
            uint16_t addr = writeAddr + i;

            if (addr < _holdingRegistersSize)
            {
                holdingRegisters[addr] = value;
                if (_onWriteHolding)
                {
                    _onWriteHolding(addr, value);
                }
            }
        }

        // 2) Leer readQty registros
        //   Montamos la respuesta
        uint8_t readByteCount = readQty * 2;
        response[0] = _slaveID;
        response[1] = 0x17;
        response[2] = readByteCount;
        uint8_t pos = 3;
        for (uint16_t i = 0; i < readQty; i++)
        {
            uint16_t addr = readAddr + i;
            uint16_t regVal = 0;
            if (addr < _holdingRegistersSize)
            {
                regVal = (_onReadHolding) ? _onReadHolding(addr) : holdingRegisters[addr];
            }
            response[pos++] = highByte(regVal);
            response[pos++] = lowByte(regVal);
        }

        responseLen = 3 + readByteCount;
        break;
    }
    default:
        if (_debug)
        {
            Serial.printf("[ModbusSlave] Función 0x%02X no soportada, ignorando.\n", func);
        }
        sendException(func, MB_EX_ILLEGAL_FUNCTION);
        return;
    }

    sendResponse(response, responseLen);
}

void JWModbusSlave::poll()
{
    static unsigned long lastByteTime = 0;
    // Lee bytes disponibles en el buffer del puerto
    while (_serial->available())
    {
        lastByteTime = millis();

        // Lee byte a byte hasta completar un frame.
        // Para esta versión mínima, asumimos que se recibe el frame completo de una vez.
        if (_frameLength < sizeof(_frame))
        {
            _frame[_frameLength++] = _serial->read();
            if (_debug)
            {
                Serial.printf("[ModbusSlave] Byte %u leído: 0x%02X\n", _frameLength, _frame[_frameLength - 1]);
            }
        }
        else
        {
            // Overflow: se recibió más de lo que cabe en _frame
            if (_debug)
            {
                Serial.println("[ModbusSlave] Frame overflow, limpiando buffer");
            }
            clearFrame();
        }
    }

    // Si se detecta que no llegan nuevos bytes por un período (por ejemplo, 10ms),
    // consideramos que se terminó de recibir el frame.
    if (_frameLength > 0 && _serial->available() == 0)
    {

        if (_debug)
        {
            Serial.printf("[ModbusSlave] Procesando frame de longitud %u\n", _frameLength);
        }

        // Procesa el frame recibido
        processRequest((uint8_t *)_frame, _frameLength);
        clearFrame(); // Reinicia el buffer para el próximo mensaje.
    }
}

// Setters para configurar el tamaño efectivo de cada área de memoria.
void JWModbusSlave::setHoldingRegistersSize(uint16_t size)
{
    if (size > MODBUS_MAX_HOLDING_REGISTERS)
        size = MODBUS_MAX_HOLDING_REGISTERS;
    _holdingRegistersSize = size;
}
void JWModbusSlave::setInputRegistersSize(uint16_t size)
{
    if (size > MODBUS_MAX_INPUT_REGISTERS)
        size = MODBUS_MAX_INPUT_REGISTERS;
    _inputRegistersSize = size;
}
void JWModbusSlave::setCoilsSize(uint16_t size)
{
    if (size > MODBUS_MAX_COILS)
        size = MODBUS_MAX_COILS;
    _coilsSize = size;
}
void JWModbusSlave::setDiscreteInputsSize(uint16_t size)
{
    if (size > MODBUS_MAX_DISCRETE_INPUTS)
        size = MODBUS_MAX_DISCRETE_INPUTS;
    _discreteInputsSize = size;
}

// Setters para los callbacks
void JWModbusSlave::setOnReadHoldingRegister(ReadRegisterCallback callback)
{
    _onReadHolding = callback;
}
void JWModbusSlave::setOnWriteHoldingRegister(WriteRegisterCallback callback)
{
    _onWriteHolding = callback;
}

void JWModbusSlave::setOnReadCoilBit(ReadCoilCallback callback)
{
    _onReadCoilBit = callback;
}
void JWModbusSlave::setOnWriteCoilBit(WriteCoilCallback callback)
{
    _onWriteCoilBit = callback;
}

void JWModbusSlave::setOnReadDiscreteBit(ReadDiscreteCallback callback)
{
    _onReadDiscreteBit = callback;
}

void JWModbusSlave::setOnReadInputRegister(ReadInputRegisterCallback callback)
{
    _onReadInputReg = callback;
}

void JWModbusSlave::setPreTransmission(void (*callback)())
{
    _preTransmission = callback;
}

void JWModbusSlave::setPostTransmission(void (*callback)())
{
    _postTransmission = callback;
}

static void modbusPollTask(void *pvParameters)
{
    JWModbusSlave *slave = (JWModbusSlave *)pvParameters;
    for (;;)
    {
        slave->poll();
        vTaskDelay(5 / portTICK_PERIOD_MS); // Delay breve para ceder CPU
    }
}

void JWModbusSlave::startPollTask(UBaseType_t priority, size_t stackSize, BaseType_t coreID)
{
    // Crea la tarea y almacena el TaskHandle
    xTaskCreatePinnedToCore(
        modbusPollTask,   // Función de la tarea
        "ModbusPollTask", // Nombre de la tarea
        stackSize,        // Tamaño del stack
        this,             // Parámetro: la instancia actual
        priority,         // Prioridad de la tarea
        &_pollTaskHandle, // Almacena el handle
        coreID            // Núcleo en el que se ejecutará
    );
}

void JWModbusSlave::setFreeRTOSPoll(bool enable, UBaseType_t priority, size_t stackSize, BaseType_t coreID)
{
    if (enable)
    {
        if (_pollTaskHandle == NULL)
        { // No está ya en ejecución
            startPollTask(priority, stackSize, coreID);
        }
    }
    else
    {
        if (_pollTaskHandle != NULL)
        {
            // Elimina la tarea FreeRTOS
            vTaskDelete(_pollTaskHandle);
            _pollTaskHandle = NULL;
        }
    }
}