/*
 * Gpio.h:
 *	Another Peripheral Library for the raspberry PI.
 *	Copyright (c) 2019 Alger Pike
 ***********************************************************************
 * This file is part of APLPIe:
 *	https://github.com/AlgerP572/APLPIe
 *
 *    APLPIe is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    APLPIe is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with APLPIe.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
*
 * code snipets taken from WiringPi:
 *	Arduino like Wiring library for the Raspberry Pi.
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *	Copyright (c) 2012-2019 Gordon Henderson
 *
 */
#pragma once

#include <pthread.h>
#include <stdint.h>

#include "../Headers/Peripheral.h"
#include "../Headers/hw-addresses.h"

#define GPSET0OFFSET 0x0000001C

// GPIO Function Select bits
enum class PinMode
{
	Input = 0b000,
	Output = 0b001,
	Alt0 = 0b100,
	Alt1 = 0b101,
	Alt2 = 0b110,
	Alt3 = 0b111,
	Alt4 = 0b011,
	Alt5 = 0b010
};

enum class PudMode
{
	Off = 0b00, // disable pull - up / down
	PullDown = 0b01, // Enable Pull Down control
	PullUp = 0b10, // Enable Pull Up control
	Reserved = 0b11
};

struct IntTrigger {
	enum Enum
	{
		Setup = 0b00,
		Falling = 0b01,
		Rising = 0b10,
		Both = 0b11
	};

	static const char* ToStr(const IntTrigger::Enum& intTrigger)
	{
		switch (intTrigger)
		{
		case Setup:    return "setup";
		case Falling:  return "falling";
		case Rising:   return "rising";
		case Both:     return "both";
		default:
			break;
		}
		return "";
	}
};

enum class PinState
{
	Low = 0b00,
	High = 0b01,
	Unknown = 0b11
};

struct GpioRegisters
{
	union
	{
		uint32_t GPFSEL[6];
		struct
		{
			uint32_t GPFSEL0; // GPIO Function Select 
			uint32_t GPFSEL1;
			uint32_t GPFSEL2;
			uint32_t GPFSEL3;
			uint32_t GPFSEL4;
			uint32_t GPFSEL5;
		};
	};
	uint32_t Resereved0;

	union
	{
		uint32_t GPSET[2]; // GPIO Pin Output Set
		struct
		{
			uint32_t GPSET0; // GPIO Pin Output Set 0
			uint32_t GPSET1; // GPIO Pin Output Set 1
		};
	};
	uint32_t Resereved1;

	union
	{
		uint32_t GPCLR[2]; // GPIO Pin Output Clear
		struct
		{
			uint32_t GPCLR0; // GPIO Pin Output Clear 0
			uint32_t GPCLR1; // GPIO Pin Output Clear 1
		};
	};
	uint32_t Resereved2;

	union
	{
		uint32_t GPLEV[2]; // GPIO Pin Level
		struct
		{
			uint32_t GPLEV0; // GPIO Pin Level 0
			uint32_t GPLEV1; // GPIO Pin Level 1
		};
	};
	uint32_t Resereved3;

	union
	{
		uint32_t GPEDS[2]; // GPIO Pin Event Detect Status
		struct
		{
			uint32_t GPEDS0; // GPIO Pin Event Detect Status 0
			uint32_t GPEDS1; // GPIO Pin Event Detect Status 1
		};
	};
	uint32_t Resereved4;

	union
	{
		uint32_t GPREN[2]; // GPIO Pin Rising Edge Detect Enable
		struct
		{
			uint32_t GPREN0; // GPIO Pin Rising Edge Detect Enable 0
			uint32_t GPREN1; // GPIO Pin Rising Edge Detect Enable 1
		};
	};
	uint32_t Resereved5;

	union
	{
		uint32_t GPFEN[2]; // GPIO Pin Falling Edge Detect Enable
		struct
		{
			uint32_t GPFEN0; // GPIO Pin Falling Edge Detect Enable 0
			uint32_t GPFEN1; // GPIO Pin Falling Edge Detect Enable 1
		};
	};
	uint32_t Resereved6;

	union
	{
		uint32_t GPHEN[2]; // GPIO Pin High Detect Enable
		struct
		{
			uint32_t GPHEN0; // GPIO Pin High Detect Enable 0
			uint32_t GPHEN1; // GPIO Pin High Detect Enable 1
		};
	};
	uint32_t Resereved7;

	union
	{
		uint32_t GPLEN[2]; // GPIO Pin Low Detect Enable
		struct
		{
			uint32_t GPLEN0; // GPIO Pin Low Detect Enable 0
			uint32_t GPLEN1; // GPIO Pin Low Detect Enable 1
		};
	};
	uint32_t Resereved8;

	union
	{
		uint32_t GPAREN[2]; // GPIO Pin Async. Rising Edge Detect
		struct
		{
			uint32_t GPAREN0; // GPIO Pin Async. Rising Edge Detect 0
			uint32_t GPAREN1; // GPIO Pin Async. Rising Edge Detect 1
		};
	};
	uint32_t Resereved9;

	union
	{
		uint32_t GPAFEN[2]; // GPIO Pin Async. Falling Edge Detect
		struct
		{
			uint32_t GPAFEN0; // GPIO Pin Async. Falling Edge Detect 0
			uint32_t GPAFEN1; // GPIO Pin Async. Falling Edge Detect 1
		};
	};
	uint32_t Resereved10;
	
	uint32_t GPPUD; // GPIO Pin Pull - up / down Enable
	
	union
	{
		uint32_t GPPUDCLK[2]; // GPIO Pin Pull-up/down Enable Clock
		struct
		{
			uint32_t GPPUDCLK0; // GPIO Pin Pull-up/down Enable Clock 
			uint32_t GPPUDCLK1;
		};
	};
	uint32_t Resereved11;
	uint32_t Test;
};

struct InterruptInfo
{
	int Pin;
	void* Arg;
	int Fd;
	pthread_t ThreadId;
	int EventFd;
	bool Waiting;

	InterruptInfo()
	{
		Pin = -1;
		Fd = -1;
		Arg = NULL;
		ThreadId = -1;
		EventFd = -1;
		Waiting = false;
	}
};

class Gpio : public PeripheralTemplate<GpioRegisters>
{
private:	
	bool ClearInterupts(int pin) noexcept;
	bool SetPinEdgeTrigger(int pin, IntTrigger::Enum edgeTrigger) noexcept;

	static InterruptInfo _interruptInfo[64];
	static void(*IsrFunctions[64])(void*);

	static void* InterruptHandler(void *arg) noexcept;
	static int WaitForInterrupt(int bcmPin, int mS) noexcept;

public:
	Gpio(const char* name);
	void virtual SysInit();
	void virtual SysUninit();
	
	void Export(int pin);
	void Unexport(int pin);
	bool SetIsr(int pin, IntTrigger::Enum mode, void(*function)(void*), void* arg) noexcept;
	bool ClearIsr(int pin) noexcept;
	void SetPinMode(int pin, PinMode mode) noexcept;
	void SetPudMode(int pin, PudMode mode) noexcept;
	PinState ReadPin(int pin) noexcept;
	uint32_t ReadPins031() noexcept;
	uint32_t ReadPins3253() noexcept;
	void WritePin(int pin, PinState value) noexcept;
	void WritePins031(uint32_t pinsMask, uint32_t value) noexcept;
	void WritePins3253(uint32_t pinsMask, uint32_t value) noexcept;
};