/*
 * Main.cpp:
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
 */
#include <stdio.h>
#include <stdint.h>
#include <signal.h>

#include "./Tests/Tests.h"
#include "./Headers/Clock.h"
#include "./Headers/Dma.h"
#include "./Headers/Gpio.h"
#include "./Headers/Pwm.h"
#include "./Headers/Display.h"


// Use BCM pin numbers i.e. the ones on the breakout boards...
#define PinOut0 19
#define PinOut1 26

#define DmaPin0 5
#define DmaPin1 6

#define PinIn0 16
#define PinIn1 20

static Dma dma("Dma");
static Gpio gpio("Gpio");
static Pwm pwm("Pwm");
static Clock clock1("Clock");

const int numPeripherals = 4;
static Peripheral** peripherals = new Peripheral*[numPeripherals];

// Display (These select the digit to display.)
#define   DisplayPin0    2
#define   DisplayPin1    3
#define   DisplayPin2    8
#define   DisplayPin3    7

#define   DisplayCharPin0 17
#define   DisplayCharPin1 18
#define   DisplayCharPin2 27
#define   DisplayCharPin3 22
#define   DisplayCharPin4 23
#define   DisplayCharPin5 24
#define   DisplayCharPin6 25
#define   DisplayCharPin7 4

static CharacterDisplayPins characterPins = CharacterDisplayPins(DisplayCharPin0,
	DisplayCharPin1,
	DisplayCharPin2,
	DisplayCharPin3,
	DisplayCharPin4,
	DisplayCharPin5,
	DisplayCharPin6,
	DisplayCharPin7);

static FourDigitSevenSegmentDisplay display(gpio,
	DisplayPin0,
	DisplayPin1,
	DisplayPin2,
	DisplayPin3,
	characterPins);

const int numDevices = 1;
static Device** devices = new Device*[numDevices];

class Program
{
private:
	static void SysInit(void);
	static void SysUninit(void);
	static void SigHandler(int sig);

public:
	static int Main(void);
};

int main(void)
{
	peripherals[0] = &dma;
	peripherals[1] = &gpio;
	peripherals[2] = &pwm;
	peripherals[3] = &clock1;

	devices[0] = &display;
	return Program::Main();
}

int Program::Main(void)
{
	SysInit();

	// Use BCM pin numbers i.e. the ones on the breakout boards...
	Test::WritePin(gpio, PinOut0);
	Test::WritePin(gpio, PinOut1);

	
	Test::WritePin(gpio, DmaPin0);
	Test::WritePin(gpio, DmaPin1);

	Test::ReadPin(gpio, PinIn0);
	Test::ReadPin(gpio, PinIn1);

//	Test::ClockEnableDisable(clock1);

	Test::PwmTest(pwm);

	uint8_t count = 0;
	while (count++ < 127)
	{
		Test::DmaMemoryToMemory(dma, count);
	}

	Test::Display(display);

	Test::DmaGpio(dma,
		gpio,
		DmaPin0,
		DmaPin1);

	Test::DmaGpioPwmGated(dma,
		pwm,
		clock1,
		gpio,
		DmaPin0,
		DmaPin1);

	Test::DmaMemoryToMemoryDoubleBuffered(dma,
		gpio,
		DmaPin1);

	Test::DmaGpioDoubleBuffered(dma,
		gpio,
		DmaPin0,
		DmaPin1);

	SysUninit();
	return 0;
}

void Program::SysInit(void)
{
	// Register signals 
	signal(SIGQUIT, SigHandler);
	signal(SIGABRT, SigHandler);
	
	// Devices depend on peripherals so init them first.
	for (int i = 0; i < numPeripherals; i++)
	{
		peripherals[i]->SysInit();
	}

	// After fresh PI reboot and a break point on the following:
	// ls /sys/class/gpio
	// should show no exports...
	// Also look at this point in the output window for status
	// on the init process for the peripherls (shows mappings)
	gpio.Export(PinOut0);
	gpio.Export(PinOut1);

	gpio.Export(DmaPin0);
	gpio.Export(DmaPin1);

	gpio.Export(PinIn0);
	gpio.Export(PinIn1);

	gpio.Export(DisplayPin0);
	gpio.Export(DisplayPin1);
	gpio.Export(DisplayPin2);
	gpio.Export(DisplayPin3);

	gpio.Export(DisplayCharPin0);
	gpio.Export(DisplayCharPin1);
	gpio.Export(DisplayCharPin2);
	gpio.Export(DisplayCharPin3);
	gpio.Export(DisplayCharPin4);
	gpio.Export(DisplayCharPin5);
	gpio.Export(DisplayCharPin6);
	gpio.Export(DisplayCharPin7);

	// now
	// ls /sys/class/gpio
	// should show
	// Pin0 and Pin1 exported.

	// Devices depend on peripherals so init peripherals first.
	for (int i = 0; i < numDevices; i++)
	{
		devices[i]->SysInit();
	}
}

void Program::SysUninit(void)
{
	// Devices depend on peripherals so Uninit devices first.
	for (int i = 0; i < numDevices; i++)
	{
		devices[i]->SysUninit();
	}

	// ls /sys/class/gpio
	// should show
	// Pin0 and Pin1 exported.
	gpio.Unexport(PinOut0);
	gpio.Unexport(PinOut1);

	gpio.Unexport(DmaPin0);
	gpio.Unexport(DmaPin1);

	gpio.Unexport(PinIn0);
	gpio.Unexport(PinIn1);

	gpio.Unexport(DisplayPin0);
	gpio.Unexport(DisplayPin1);
	gpio.Unexport(DisplayPin2);
	gpio.Unexport(DisplayPin3);

	gpio.Unexport(DisplayCharPin0);
	gpio.Unexport(DisplayCharPin1);
	gpio.Unexport(DisplayCharPin2);
	gpio.Unexport(DisplayCharPin3);
	gpio.Unexport(DisplayCharPin4);
	gpio.Unexport(DisplayCharPin5);
	gpio.Unexport(DisplayCharPin6);
	gpio.Unexport(DisplayCharPin7);

	// now
	// ls /sys/class/gpio
	// should show
	// Pin0 and Pin1 NOT exported.

	for (int i = 0; i < numPeripherals; i++)
	{
		peripherals[i]->SysUninit();
	}
}

void Program::SigHandler(int sig)
{
	SysUninit();
}