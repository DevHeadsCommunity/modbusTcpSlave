/**
 * file            db.h
 * brief           data read write, memory allocation
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


#ifndef DB_H
#define DB_H

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define COILS_START_ADDRESS 00001
#define DISCRETE_INPUTS_START_ADDRESS 10001
#define INPUT_REGISTERS_START_ADDRESS 30001
#define HOLDING_REGISTERS_START_ADDRESS 40001

// Define boolean alternatives for cross-platform compatibility
#define TRUE 1
#define FALSE 0

typedef struct
{
    uint8_t slaveID;
    uint16_t *holdingRegisters;
    uint16_t *inputRegisters;
    uint8_t *coils;
    uint8_t *discreteInputs;
    size_t numHoldingRegs;
    size_t numInputRegs;
    size_t numCoils;
    size_t numDiscreteInputs;
} ModbusSlaveData;

int initModbusSlaveData(ModbusSlaveData *data, uint8_t id, size_t numHoldingRegs, size_t numInputRegs, size_t numCoils, size_t numDiscreteInputs);

void freeModbusSlaveData(ModbusSlaveData *data);

bool dbGetDiscreteInputState(uint8_t *discreteInputs, size_t n);

void dbSetDiscreteInputState(uint8_t *discreteInputs, size_t n, bool state);

uint16_t dbGetHoldingRegister(uint16_t *holdingRegisters, size_t n);

void dbSetHoldingRegister(uint16_t *holdingRegisters, size_t n, uint16_t value);

bool getDiscreteInputState(ModbusSlaveData *data, size_t n, bool *state);

bool getHoldingRegister(ModbusSlaveData *data, size_t n, uint16_t *value);

bool setHoldingRegister(ModbusSlaveData *data, size_t n, uint16_t value);

bool setDiscreteInputState(ModbusSlaveData *data, size_t n, bool state);

#endif // DB_H
