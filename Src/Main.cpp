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


// Use BCM pin numbers i.e. the ones on the breakout boards...
#define Pin0 19
#define Pin1 26

static Dma dma("Dma");
static Gpio gpio("Gpio");
static Pwm pwm("Pwm");
static Clock clock1("Clock");

const int numPeripherals = 4;
static Peripheral** peripherals = new Peripheral*[numPeripherals];

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
	return Program::Main();
}

int Program::Main(void)
{
	SysInit();

	// Use BCM pin numbers i.e. the ones on the breakout boards...
	Test::WritePin(&gpio, Pin0);
	Test::WritePin(&gpio, Pin1);

	Test::ClockEnableDisable(&clock1);

	Test::PwmTest(&pwm);

	uint8_t count = 0;
	while (count++ < 127)
	{
		Test::DmaMemoryToMemory(&dma, count);
	}

	SysUninit();
	return 0;
}

void Program::SysInit(void)
{
	// Register signals 
	signal(SIGQUIT, SigHandler);
	signal(SIGABRT, SigHandler);

	peripherals[0] = &dma;
	peripherals[1] = &gpio;
	peripherals[2] = &pwm;
	peripherals[3] = &clock1;

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
	gpio.Export(Pin0);
	gpio.Export(Pin1);

	// now
	// ls /sys/class/gpio
	// should show
	// Pin0 and Pin1 exported.	
}

void Program::SysUninit(void)
{
	// ls /sys/class/gpio
	// should show
	// Pin0 and Pin1 exported.
	gpio.Unexport(Pin0);
	gpio.Unexport(Pin1);

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