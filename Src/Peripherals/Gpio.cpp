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

#include "../Headers/Gpio.h"
#include "../Headers/ScreenLog.h"

#include "../Headers/hw-addresses.h"

// gpioToGPFSEL:
//	Map a BCM_GPIO pin to it's Function Selection
//	control port. (GPFSEL 0-5)
//	Groups of 10 - 3 bits per Function - 30 bits per port

static uint8_t gpioToGPFSEL[] =
{
  0,0,0,0,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,1,1,
  2,2,2,2,2,2,2,2,2,2,
  3,3,3,3,3,3,3,3,3,3,
  4,4,4,4,4,4,4,4,4,4,
  5,5,5,5,5,5,5,5,5,5,
};


// gpioToShift
//	Define the shift up for the 3 bits per pin in each GPFSEL port

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

Gpio::Gpio(const char* name) :
	Peripheral(name)
{
}

void Gpio::SysInit()
{
	int memSize = sizeof(GpioRegisters);
	int pages = memSize / _pageSize + (memSize % _pageSize) > 0 ? 1 : 0;

	Map(GPIO_BASE, pages * _pageSize, _gpio.info);
	DBG("Gpio Base: %p", _gpio.info.MappedAddress);
}

void Gpio::SysUninit()
{
	Unmap(_gpio.info);
}

void Gpio::SetPinMode(int pin, PinMode mode)
{
	uint32_t fSel = gpioToGPFSEL[pin];
	uint32_t shift = gpioToShift[pin];

	volatile uint32_t* address = &_gpio.Base->GPFSEL[fSel];	
	*address |= ((uint32_t) mode << shift);
}

void Gpio::SetPudMode(int pin, PudMode mode)
{
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
	volatile uint32_t* addressPud = &_gpio.Base->GPPUD;
	volatile uint32_t* addressPudClk = &_gpio.Base->GPPUDCLK[pUDClk];

	*addressPud = (uint32_t)mode & 3;
	DelayMicroseconds(5);
	*addressPudClk = 1 << (pin & 31);
	DelayMicroseconds(5);
	*addressPud = 0;
	DelayMicroseconds(5);
	*addressPudClk = 0;
	DelayMicroseconds(5);
}

void Gpio::WritePin(int pin, PinState value)
{
	
	if (value == PinState::Low)
	{		

		volatile uint32_t* addressClr =  &(_gpio.Base->GPCLR[gpioToRegister[pin]]);
		
		*addressClr = 1 << (pin & 31);
	}
	else
	{		
		volatile uint32_t* addressSet = &(_gpio.Base->GPSET[gpioToRegister[pin]]);
		
		*addressSet = 1 << (pin & 31);
	}
}

void Gpio::WritePins031(uint32_t pinsToWrite, uint32_t value)
{
	volatile uint32_t* addressClr = &(_gpio.Base->GPCLR[gpioToRegister[0]]);
	*addressClr = pinsToWrite & ~value;

	volatile uint32_t* addressSet = &(_gpio.Base->GPSET[gpioToRegister[0]]);
	*addressSet = pinsToWrite & value;
}

bool Gpio::SetIsr(int pin, IntTrigger mode, void(*function)(void*), void* arg)
{
	pthread_t threadId;
	const char *modeS;
	char fName[64];
	char  pinS[8];
	pid_t pid;
	int   count, i;
	char  c;

	if ((pin < 0) || (pin > 63))
	{
		DBG("ISR: pin must be 0-63 (%d)\n", pin);
		return false;
	}

	// Now export the pin and set the right edge
	//	We're going to use the gpio program to do this, so it assumes
	//	a full installation of wiringPi. It's a bit 'clunky', but it
	//	is a way that will work when we're running in "Sys" mode, as
	//	a non-root user. (without sudo)

	if (mode != IntTrigger::Setup)
	{
		if (mode == IntTrigger::Falling)
			modeS = "falling";
		else if (mode == IntTrigger::Rising)
			modeS = "rising";
		else
			modeS = "both";

		sprintf(pinS, "%d", pin);

		if ((pid = fork()) < 0)
		{
			DBG("wSR: fork failed: %s\n", strerror(errno));
			return false;
		}

		if (pid == 0)	// Child, exec
		{
			if (access("/usr/local/bin/gpio", X_OK) == 0)
			{
				execl("/usr/local/bin/gpio", "gpio", "edge", pinS, modeS, (char *)NULL);
				DBG("execl failed: %s\n", strerror(errno));
				return false;
			}
			else if (access("/usr/bin/gpio", X_OK) == 0)
			{
				execl("/usr/bin/gpio", "gpio", "edge", pinS, modeS, (char *)NULL);
				DBG("execl failed: %s\n", strerror(errno));
				return false;
			}
			else
			{
				DBG("Can't find gpio program\n");
				return false;
			}
		}
		else
		{
			wait(NULL);
		}

		if (_sysFds[pin] == -1)
		{
			sprintf(fName, "/sys/class/gpio/gpio%d/value", pin);
			if ((_sysFds[pin] = open(fName, O_RDWR)) < 0)
			{
				DBG("Unable to open %s: %s\n", fName, strerror(errno));
				return false;
			}
		}

		// Clear any initial pending interrupt

		ioctl(_sysFds[pin], FIONREAD, &count);
		for (i = 0; i < count; ++i)
			read(_sysFds[pin], &c, 1);

		IsrFunctionsExt[pin] = function;

		pthread_mutex_lock(&_pinMutex);
		_pinPass = pin;
		pthread_create(&threadId, NULL, InterruptHandlerExt, arg);
		while (_pinPass != -1)
			DelayMilliseconds(1);
		pthread_mutex_unlock(&_pinMutex);
	}

	return 0;
}

void Gpio::TestPinExample(int pin)
{
	SetPinMode(pin, PinMode::Output);

	// Toggle the pin
	WritePin(pin, PinState::High);
	// Toggle the pin
	WritePin(pin, PinState::Low);
}

int Gpio::_sysFds[64] =
{
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

int piHiPri(const int pri)
{
	struct sched_param sched;

	memset(&sched, 0, sizeof(sched));

	if (pri > sched_get_priority_max(SCHED_RR))
		sched.sched_priority = sched_get_priority_max(SCHED_RR);
	else
		sched.sched_priority = pri;

	return sched_setscheduler(0, SCHED_RR, &sched);
}

pthread_mutex_t Gpio::_pinMutex;
int Gpio::_pinPass;

void (*Gpio::IsrFunctionsExt[64])(void*);

int Gpio::WaitForInterruptExt(int bcmPin, int mS)
{
	int fd, x;
	uint8_t c;
	struct pollfd polls;

	if ((fd = _sysFds[bcmPin]) == -1)
		return -2;
	
	polls.fd = fd;
	polls.events = POLLPRI | POLLERR;	
	x = poll(&polls, 1, mS);	

	if (x > 0)
	{
		lseek(fd, 0, SEEK_SET);
		(void)read(fd, &c, 1);
	}
	return x;
}

void* Gpio::InterruptHandlerExt(void *arg)
{
	int myPin;

	(void)piHiPri(55);	// Only effective if we run as root

	myPin = _pinPass;
	_pinPass = -1;

	for (;;)
	{
		int result = WaitForInterruptExt(myPin, -1);
		if (result > 0)
			IsrFunctionsExt[myPin](arg);
	}

	return NULL;
}