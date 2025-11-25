/**
 * file            modbus.c
 * brief           modbus functions
 */

/*
 * Copyright (c) 2025 DevHeads
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of modbus tcp library.
 *
 * Author: Umesh Lokhande
 */

#include "modbus.h"
#include "main.h"

// Function to generate a Modbus exception response
// frame: Pointer to the Modbus frame
// length: Length of the Modbus frame
// exceptionCode: The Modbus exception code
// response: Pointer to the response buffer
// responseLength: Pointer to the length of the response buffer
static void generateModbusException(const uint8_t *frame, uint8_t exceptionCode, uint8_t *response, uint16_t *responseLength)
{
	// Prepare the response frame for Modbus TCP (no CRC)
	response[0] = frame[0];        // Slave ID
	response[1] = frame[1] | 0x80; // Function code with high bit set to indicate exception
	response[2] = exceptionCode;   // Exception code

	printDebug("Modbus TCP exception response generated\n");

	// Set the response length (no CRC, only 3 bytes)
	*responseLength = 3;
}

static bool handleReadDiscreteInputs(const uint8_t *frame, ModbusSlaveData *slave, uint8_t *response, uint16_t *responseLength)
{
	uint16_t startAddress = (uint16_t)frame[2] << 8 | (uint16_t)frame[3];
	uint16_t quantityOfInputs = (uint16_t)frame[4] << 8 | (uint16_t)frame[5];

	if ((startAddress + quantityOfInputs) > slave->numDiscreteInputs)
	{
		generateModbusException(frame, ILLEGAL_DATA_ADDRESS, response, responseLength);
		return FALSE;
	}

	response[0] = slave->slaveID;
	response[1] = frame[1];
	response[2] = (quantityOfInputs + 7) / 8;

	for (uint16_t i = 0; i < quantityOfInputs; i++)
	{
		bool state;
		state = dbGetDiscreteInputState(slave->discreteInputs, startAddress + i);
		if (state)
		{
			response[3 + (i / 8)] |= (1 << (i % 8));
		}
		else
		{
			response[3 + (i / 8)] &= ~(1 << (i % 8));
		}
	}

	// For Modbus TCP, no CRC is appended
	*responseLength = 3 + response[BYTE_COUNT_OFFSET];
	return TRUE;
}

static bool handleReadHoldingRegisters(const uint8_t *frame, ModbusSlaveData *slave, uint8_t *response, uint16_t *responseLength)
{
	uint16_t startAddress = (uint16_t)frame[2] << 8 | (uint16_t)frame[3];
	uint16_t quantityOfRegisters = (uint16_t)frame[4] << 8 | (uint16_t)frame[5];

	if ((startAddress + quantityOfRegisters) > slave->numHoldingRegs)
	{
		generateModbusException(frame, ILLEGAL_DATA_ADDRESS, response, responseLength);
		return FALSE;
	}

	response[0] = slave->slaveID;
	response[1] = frame[1];
	response[2] = quantityOfRegisters * 2; //length in bytes

	for (uint16_t i = 0; i < quantityOfRegisters; i++)
	{
		uint16_t value;
		value = dbGetHoldingRegister(slave->holdingRegisters, startAddress + i);
		response[3 + (i * 2)] = (value >> 8) & 0xFF;
		response[4 + (i * 2)] = value & 0xFF;
	}

	*responseLength = 3 + response[BYTE_COUNT_OFFSET];
	return TRUE;
}

static bool handleWriteMultipleRegisters(const uint8_t *frame, ModbusSlaveData *slave, uint8_t *response, uint16_t *responseLength)
{
	uint16_t startAddress = (uint16_t)frame[2] << 8 | (uint16_t)frame[3];
	uint16_t quantityOfRegisters = (uint16_t)frame[4] << 8 | (uint16_t)frame[5];
	uint8_t byteCount = frame[6];

	if ((startAddress + quantityOfRegisters) > slave->numHoldingRegs)
	{
		generateModbusException(frame, ILLEGAL_DATA_ADDRESS, response, responseLength);
		return FALSE;
	}

	for (uint16_t i = 0; i < quantityOfRegisters; i++)
	{
		uint16_t value = (uint16_t)frame[7 + (i * 2)] << 8 | (uint16_t)frame[8 + (i * 2)];
		dbSetHoldingRegister(slave->holdingRegisters, startAddress + i, value);
	}

	for (uint16_t i = 0; i < 6; i++)
	{
		response[i] = frame[i];
	}


	*responseLength = 6;

	slaveCallback(startAddress + HOLDING_REGISTERS_START_ADDRESS, quantityOfRegisters);
	return TRUE;
}



static bool handleWriteMultipleCoils(const uint8_t *frame, ModbusSlaveData *slave, uint8_t *response, uint16_t *responseLength)
{
	uint16_t startAddress = (uint16_t)frame[2] << 8 | (uint16_t)frame[3];
	uint16_t quantityOfCoils = (uint16_t)frame[4] << 8 | (uint16_t)frame[5];
	uint8_t byteCount = frame[6];

	if ((startAddress + quantityOfCoils) > slave->numCoils)
	{
		generateModbusException(frame, ILLEGAL_DATA_ADDRESS, response, responseLength);
		return FALSE;
	}

	for (uint16_t i = 0; i < quantityOfCoils; i++)
	{
		bool state = (frame[7 + (i / 8)] & (1 << (i % 8))) != 0;
		dbSetCoilState(slave->coils, startAddress + i, state);
	}

	for (uint16_t i = 0; i < 6; i++)
	{
		response[i] = frame[i];
	}

	*responseLength = 6;

	slaveCallback(startAddress + COILS_START_ADDRESS, quantityOfCoils);

	return TRUE;
}


static bool handleWriteSingleCoil(const uint8_t *frame, ModbusSlaveData *slave, uint8_t *response, uint16_t *responseLength)
{
	uint16_t address = (uint16_t)frame[2] << 8 | (uint16_t)frame[3];
	uint16_t value = (uint16_t)frame[4] << 8 | (uint16_t)frame[5];

	if (address >= slave->numCoils)
	{
		generateModbusException(frame, ILLEGAL_DATA_ADDRESS, response, responseLength);
		return FALSE;
	}

	bool state = (value == 0xFF00);
	dbSetCoilState(slave->coils, address, state);

	for (uint16_t i = 0; i < 6; i++)
	{
		response[i] = frame[i];
	}

	*responseLength = 6;

	slaveCallback(address + COILS_START_ADDRESS, 1);

	return TRUE;
}




// Function to calculate the length of the Modbus frame
uint16_t getModSlaveFrameLen(const uint8_t *frame)
{
	uint8_t function_code = frame[1];

	switch (function_code) {
	case 1: case 2: case 3: case 4: // Read Coils, Inputs, Holding Registers, Input Registers
		return 8;
	case 5: case 6: // Write Single Coil/Register
		return 8;
	case 15: case 16: { // Write Multiple Coils/Registers
		uint8_t byte_count = frame[6];
		return 9 + byte_count;
	}
	default:
		return -1; // Unsupported function code
	}
}

// Function to parse a Modbus frame
// frame: Pointer to the Modbus frame
// length: Length of the Modbus frame
// slave: Pointer to the ModbusSlaveData structure
// response: Pointer to the response buffer
// responseLength: Pointer to the length of the response buffer
// Returns TRUE if the frame is valid, FALSE otherwise
static bool parseModbusFrame(const uint8_t *frame, uint16_t length, ModbusSlaveData *slave, uint8_t *response, uint16_t *responseLength)
{
	ExceptionCodes exceptionCode = NO_EXCEPTION;

	// A valid Modbus frame must be at least 4 bytes long (address, function, data, CRC)
	if (length < MIN_MODBUS_FRAME_LENGTH)
	{
		printDebug("Invalid frame length, function code:%d\n", frame[1]);
		return FALSE;
	}

	// Check if the slave ID matches or if it's a broadcast message (slave ID 0)
	if (frame[0] != slave->slaveID && frame[0] != MODBUS_BROADCAST_ADDRESS)
	{
		printDebug("Invalid slave ID, function code:%d\n", frame[1]);
		return FALSE;
	}



	// Extract the function code from the frame (second byte)
	uint8_t functionCode = frame[1];

	// Handle the function code
	switch (functionCode)
	{
	case READ_DISCRETE_INPUTS:
		return handleReadDiscreteInputs(frame, slave, response, responseLength);
	case READ_HOLDING_REGISTERS:
		return handleReadHoldingRegisters(frame, slave, response, responseLength);
	case WRITE_MULTIPLE_REGISTERS:
		return handleWriteMultipleRegisters(frame, slave, response, responseLength);
	case WRITE_SINGLE_COIL:
		return handleWriteSingleCoil(frame, slave, response, responseLength);
	case WRITE_MULTIPLE_COILS:
		return handleWriteMultipleCoils(frame, slave, response, responseLength);
	default:
		exceptionCode = ILLEGAL_FUNCTION;
		generateModbusException(frame, exceptionCode, response, responseLength);
		return FALSE;
	}
}


// Function to handle a Modbus request
// frame: Pointer to the Modbus frame
// length: Length of the Modbus frame
// slave: Pointer to the ModbusSlaveData structure
// response: Pointer to the response buffer
// responseLength: Pointer to the length of the response buffer
// Returns TRUE if the request is handled, FALSE otherwise
bool handleModbusRequest(const uint8_t *frame, uint16_t length, uint8_t *response, uint16_t *responseLength, ModbusSlaveData *slave)
{
	//    uint8_t response[128] = {0};
	//    uint16_t responseLength = 0;

	// Parse the Modbus frame
	if (parseModbusFrame(frame, length, slave, response, responseLength))
	{
		// Write frame to serial
		printDebug("Modbus frame processed\n");

		return TRUE; // Frame is valid and response is prepared
	}
	else
	{
		// Write exception response to serial
		//        if ((response[0] != MODBUS_BROADCAST_ADDRESS) && (responseLength > 0))
		//        {
		//            RS485_Transmit(response, responseLength);
		//        }

		return FALSE; // Frame is invalid or error occurred
	}
}


// Function to write uint8_t data to holding registers
bool writeDataToHoldingRegisters(ModbusSlaveData *data, size_t startAddress, uint8_t *inputArray, size_t length)
{
    if ((startAddress + (length / 2) - HOLDING_REGISTERS_START_ADDRESS) >= data->numHoldingRegs)
    {
        return false; // Length exceeds the number of holding registers
    }
    for (size_t i = 0; i < length / 2; i++)
    {
        setHoldingRegister(data, startAddress + i, (uint16_t)((inputArray[2 * i] << 8) | (inputArray[2 * i + 1])));
    }
    return true; // Success
}

