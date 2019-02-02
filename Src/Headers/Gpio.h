#pragma once

#include <pthread.h>
#include <stdint.h>

#include "../Headers/Peripheral.h"

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

enum class IntTrigger
{
	Setup = 0b00,
	Falling = 0b01,
	Rising = 0b10,
	Both = 0b11
};

enum class PinState
{
	Low = 0b00,
	High = 0b01
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

	uint32_t GPLEV0; // GPIO Pin Level
	uint32_t GPLEV1;
	uint32_t Resereved3;

	uint32_t GPEDS0; // GPIO Pin Event Detect Status
	uint32_t GPEDS1;
	uint32_t Resereved4;

	uint32_t GPREN0; // GPIO Pin Rising Edge Detect Enable
	uint32_t GPREN1;
	uint32_t Resereved5;

	uint32_t GPFEN0; // GPIO Pin Falling Edge Detect Enable
	uint32_t GPFEN1;
	uint32_t Resereved6;

	uint32_t GPHEN0; // GPIO Pin High Detect Enable
	uint32_t GPHEN1;
	uint32_t Resereved7;

	uint32_t GPLEN0; // GPIO Pin Low Detect Enable
	uint32_t GPLEN1;
	uint32_t Resereved8;

	uint32_t GPAREN0; // GPIO Pin Async. Rising Edge Detect
	uint32_t GPAREN1;
	uint32_t Resereved9;

	uint32_t GPAFEN0; // GPIO Pin Async. Falling Edge Detect
	uint32_t GPAFEN1;
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

typedef struct
{
	union
	{
		PeripheralInfo info;
		volatile GpioRegisters* Base;
	};
} GpioInfo;

class Gpio : public Peripheral
{
private:
	GpioInfo _gpio;
	static int _sysFds[64];
	static pthread_mutex_t _pinMutex;
	static int _pinPass;
	static void (*IsrFunctionsExt[64])(void*);

	static int WaitForInterruptExt(int bcmPin, int mS);
	static void* InterruptHandlerExt(void *arg);

public:
	Gpio(const char* name);
	void virtual SysInit();
	void virtual SysUninit();

	bool SetIsr(int pin, IntTrigger mode, void(*function)(void*), void* arg);
	void SetPinMode(int pin, PinMode mode);
	void SetPudMode(int pin, PudMode mode);
	void WritePin(int pin, PinState value);
	void WritePins031(uint32_t pinsMask, uint32_t value);

	void TestPinExample(int pin);
};