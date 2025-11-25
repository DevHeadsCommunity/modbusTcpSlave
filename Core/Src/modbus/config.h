/**
 * file            config.h
 * brief           modbus configuration
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
 * This file is part of modbus library.
 *
 * Author: Umesh Lokhande
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "db.h"
#include "modbus.h"

const uint8_t slaveID = 1;

const size_t numOfHoldingRegs = 24;
const size_t numOfInputRegs = 16;
const size_t numOfCoils = 16;
const size_t numOfDisInput = 16;

//slave structure object
ModbusSlaveData modbusSlave;

// Modbus address definitions
enum modAddress
{
	RTC_DATE = HOLDING_REGISTERS_START_ADDRESS,
	RTC_MONTH ,
	RTC_YEAR ,
	RTC_DAY ,

	RTC_HOURS,
	RTC_MINUTES,
	RTC_SECONDS,
	RTC_SUBSECONDS,

	modbusLcdData,

	DI_buttonStatus = DISCRETE_INPUTS_START_ADDRESS,
};


#endif // MODBUS_SLAVE_H
