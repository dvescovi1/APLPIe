/*
 * Gpio.cpp:
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
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <poll.h>
#include <errno.h>
#include <string.h>
#include<sys/types.h>
#include<sys/wait.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <fcntl.h>

#include "../Headers/Delay.h"
#include "../Headers/Gpio.h"
#include "../Headers/ScreenLog.h"

#include "../Headers/hw-addresses.h"

static uint8_t gpioToGPFSEL[] =
{
  0,0,0,0,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,1,1,
  2,2,2,2,2,2,2,2,2,2,
  3,3,3,3,3,3,3,3,3,3,
  4,4,4,4,4,4,4,4,4,4,
  5,5,5,5,5,5,5,5,5,5,
};

static uint8_t gpioToShift[] =
{
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
};

static uint8_t gpioToRegister[] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
};

InterruptInfo Gpio::_interruptInfo[64];

Gpio::Gpio(const char* name) :
	PeripheralTemplate<GpioRegisters, GPIO_BASE>(name)
{
}

void Gpio::SysInit()
{
	// TODO: Investigate using use mode /sys/class/gpio/gpiochip0
	// but other peripherals currently also need /dev/mem so
	// its lower priority...
	//
	// /sys/class/gpio/ is used for interrupt access since that
	// is the only way to do it...
	PeripheralTemplate<GpioRegisters, GPIO_BASE>::SysInit();	
}

void Gpio::SysUninit()
{
	for (int i = 0; i < 64; i++)
	{
		if (_interruptInfo[i].Fd == -1)
			continue;

		close(_interruptInfo[i].Fd);
	}

	PeripheralTemplate<GpioRegisters, GPIO_BASE>::SysUninit();
}

void Gpio::Export(int pin)
{
	if (pin < 0 || pin > 53)
	{
		DBG("PudMode: pin must be 0-53 provided: (%d)", pin);
		return;
	}
	
	int fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd == -1)
	{
		DBG("Failed to open export for writing: %s", strerror(errno));
	}

	char buffer[8];
	int writeBytes = snprintf(buffer,
		sizeof(buffer),
		"%d",
		pin);
	write(fd,
		buffer,
		writeBytes);
	close(fd);
}

void Gpio::Unexport(int pin)
{
	if (pin < 0 || pin > 53)
	{
		DBG("PudMode: pin must be 0-53 provided: (%d)", pin);
		return;
	}

	int fd = open("/sys/class/gpio/unexport", O_WRONLY);
	if (fd == -1)
	{
		fprintf(stderr, "Failed to open unexport for writing!");
	}

	char buffer[8];
	ssize_t writeBytes;
	writeBytes = snprintf(buffer,
		sizeof(buffer),
		"%d",
		pin);
	write(fd,
		buffer,
		writeBytes);
	close(fd);
}

void Gpio::SetPinMode(int pin, PinMode mode) noexcept
{
	if (pin < 0 || pin > 53)
	{
		DBG("PudMode: pin must be 0-53 provided: (%d)", pin);
		return;
	}

	uint32_t fSel = gpioToGPFSEL[pin];
	uint32_t shift = gpioToShift[pin];

	volatile uint32_t* address = &Base->GPFSEL[fSel];	
	*address |= ((uint32_t) mode << shift);
}

void Gpio::SetPudMode(int pin, PudMode mode) noexcept
{
	if (pin < 0 || pin > 53)
	{
		DBG("PudMode: pin must be 0-53 provided: (%d)", pin);
		return;
	}

	/*
	GPIO Pull - up / down Clock Registers(GPPUDCLKn)
		SYNOPSIS The GPIO Pull - up / down Clock Registers control the actuation of internal pull - downs on
		the respective GPIO pins.These registers must be used in conjunction with the GPPUD
		register to effect GPIO Pull - up / down changes.The following sequence of events is
		required :
	1. Write to GPPUD to set the required control signal(i.e.Pull - up or Pull - Down or neither
		to remove the current Pull - up / down)
		2. Wait 150 cycles – this provides the required set - up time for the control signal
		3. Write to GPPUDCLK0 / 1 to clock the control signal into the GPIO pads you wish to
		modify – NOTE only the pads which receive a clock will be modified, all others will
		retain their previous state.
		4. Wait 150 cycles – this provides the required hold time for the control signal
		5. Write to GPPUD to remove the control signal
		6. Write to GPPUDCLK0 / 1 to remove the clock
	*/
	uint32_t pUDClk = gpioToRegister[pin];
	volatile uint32_t* addressPud = &Base->GPPUD;
	volatile uint32_t* addressPudClk = &Base->GPPUDCLK[pUDClk];

	*addressPud = (uint32_t)mode & 3;
	Delay::Microseconds(5);
	*addressPudClk = 1 << (pin & 31);
	Delay::Microseconds(5);
	*addressPud = 0;
	Delay::Microseconds(5);
	*addressPudClk = 0;
	Delay::Microseconds(5);
}

void Gpio::WritePin(int pin, PinState value) noexcept
{
	if (pin < 0 || pin > 53)
	{
		DBG("Isr: pin must be 0-53 provided: (%d)", pin);
		return;
	}
	
	if (value == PinState::Low)
	{		

		volatile uint32_t* addressClr =  &(Base->GPCLR[gpioToRegister[pin]]);
		
		*addressClr = 1 << (pin & 31);
	}
	else
	{		
		volatile uint32_t* addressSet = &(Base->GPSET[gpioToRegister[pin]]);
		
		*addressSet = 1 << (pin & 31);
	}
}

void Gpio::WritePins031(uint32_t pinsToWrite, uint32_t value) noexcept
{
	volatile uint32_t* addressClr = &(Base->GPCLR0);
	*addressClr = pinsToWrite & ~value;

	volatile uint32_t* addressSet = &(Base->GPSET0);
	*addressSet = pinsToWrite & value;
}

void Gpio::WritePins3253(uint32_t pinsToWrite, uint32_t value) noexcept
{
	volatile uint32_t* addressClr = &(Base->GPCLR1);
	*addressClr = pinsToWrite & ~value;

	volatile uint32_t* addressSet = &(Base->GPSET1);
	*addressSet = pinsToWrite & value;
}

bool Gpio::SetIsr(int pin, IntTrigger::Enum mode, void(*function)(void*), void* arg)
{
	if (pin < 0 || pin > 53)
	{
		DBG("Isr: pin must be 0-53 provided: (%d)", pin);
		return false;
	}

	if (mode == IntTrigger::Setup)
	{
		DBG("Isr: warning mode setup passed nopthing to do!");
		return false;
	}

	SetPinEdgeTrigger(pin, mode);
	ClearInterupts(pin);

	IsrFunctions[pin] = function;

	_interruptInfo[pin].Pin = pin;
	_interruptInfo[pin].Arg = arg;

	pthread_t threadId;	
	pthread_create(&threadId,
		NULL,
		InterruptHandler,
		(void*) &_interruptInfo[pin]);
	return 0;
}

bool Gpio::SetPinEdgeTrigger(int pin, IntTrigger::Enum edgeTrigger) noexcept
{
	int fd;
	char fName[32];

	sprintf(fName, "/sys/class/gpio/gpio%d/edge", pin);
	fd = open(fName, O_WRONLY);
	if (fd == -1)
	{
		DBG("Isr: Unable to open GPIO edge interface for pin %d: %s",
			pin,
			strerror(errno));
		return false;
	}

	char buffer[8];
	ssize_t bytes_written;
	bytes_written = snprintf(buffer,
		8,
		"%s\n",
		IntTrigger::ToStr(edgeTrigger));
	write(fd,
		buffer,
		bytes_written);
	close(fd);
	return true;
}

bool Gpio::ClearInterupts(int pin) noexcept
{
	if (_interruptInfo[pin].Fd == -1)
	{
		char fName[32];

		sprintf(fName, "/sys/class/gpio/gpio%d/value", pin);
		if ((_interruptInfo[pin].Fd = open(fName, O_RDWR)) < 0)
		{
			DBG("Unable to open %s: %s",
				fName,
				strerror(errno));

			return false;
		}
	}

	int isrCount;
	ioctl(_interruptInfo[pin].Fd,
		FIONREAD,
		&isrCount);

	for (int i = 0; i < isrCount; ++i)
	{
		char  intValue;
		read(_interruptInfo[pin].Fd, &intValue, 1);
	}
	return true;
}

/*
*
* code snipets taken from WiringPi :
* Arduino like Wiring library for the Raspberry Pi.
*	https ://projects.drogon.net/raspberry-pi/wiringpi/
*	Copyright(c) 2012 - 2019 Gordon Henderson
*
*/
int piHiPri(const int pri)
{
	sched_param sched;

	memset(&sched, 0, sizeof(sched));

	if (pri > sched_get_priority_max(SCHED_RR))
	{
		sched.sched_priority = sched_get_priority_max(SCHED_RR);
	}
	else
	{
		sched.sched_priority = pri;
	}

	return sched_setscheduler(0, SCHED_RR, &sched);
}

void (*Gpio::IsrFunctions[64])(void*);

int Gpio::WaitForInterrupt(int pin, int mS) noexcept
{
	if (_interruptInfo[pin].Fd == -1)
	{
		DBG("WaitForIsr: unexpected fd for pin %d is -1", pin);
		return -2;
	}
	
	pollfd polls;
	polls.fd = _interruptInfo[pin].Fd;
	polls.events = POLLPRI | POLLERR;	
	int event = poll(&polls,
		1,
		mS);

	if (event > 0)
	{
		lseek(_interruptInfo[pin].Fd,
			0,
			SEEK_SET);

		uint8_t value;
		read(_interruptInfo[pin].Fd,
			&value,
			1);
	}
	return event;
}

void* Gpio::InterruptHandler(void *arg) noexcept
{
	InterruptInfo* pIntInfo = (InterruptInfo*)arg;

	piHiPri(55);

	while (true)
	{
		int result = WaitForInterrupt(pIntInfo->Pin, -1);
		if (result > 0)
		{

			IsrFunctions[pIntInfo->Pin](pIntInfo->Arg);
		}
	}
	return NULL;
}