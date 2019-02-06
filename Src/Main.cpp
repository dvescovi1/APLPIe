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

#include "./Headers/Clock.h"
#include "./Headers/Dma.h"
#include "./Headers/Gpio.h"
#include "./Headers/Pwm.h"
#include "./Headers/Timer.h"
#include "./Headers/Delay.h"
#include "./Headers/DmaMemory.h"

// Use BCM pin numbers i.e. the ones on the breakout boards...
#define Pin0 19
#define Pin1 26

static Dma dma("Dma Controller");
static Gpio gpio("Gpio Controller");
static Pwm pwm("Pwm Controller");
static Clock clock1("Clock Controller");

const int numPeripherals = 4;
static Peripheral** peripherals = new Peripheral*[numPeripherals];

class Test
{
public:
	static void WritePin(Gpio* gpio, int pin);
	static void ClockEnableDisable(Clock* clock);
	static void PwmTest(Pwm* pwm);
	static void DmaMemoryToMemory(Dma* dma, uint8_t count);
};

class Program
{
private:
	static void SysInit(void);
	static void SysUninit(void);
	static void SigHandler(int sig);

public:
	static int Main(void);
};

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

int Program::Main(void)
{
	SysInit();
		
	// Use BCM pin numbers i.e. the ones on the breakout boards...
	Test::WritePin(&gpio, Pin0);
	Test::WritePin(&gpio, Pin1);

	Test::ClockEnableDisable(&clock1);

	Test::PwmTest(&pwm);

	uint8_t count = 0;
	while(count++ < 127)
	{
		Test::DmaMemoryToMemory(&dma, count);
	}

	SysUninit();
	return 0;
}

int main(void)
{
	return Program::Main();
}

void Test::WritePin(Gpio* gpio, int pin)
{
	gpio->SetPinMode(pin, PinMode::Output);

	// Toggle the pin
	gpio->WritePin(pin, PinState::High);
	Delay::Milliseconds(1000);
	gpio->WritePin(pin, PinState::Low);
}

void Test::ClockEnableDisable(Clock* clock)
{
	clock->Disable();
	//clock->Enable();
}

void Test::PwmTest(Pwm* pwm)
{
	pwm->Base.Base->DMAC = 0; //disable DMA
	pwm->Base.Base->CTL |= PWM_CTL_CLRFIFO; //clear pwm
	Delay::Microseconds(100);

	pwm->Base.Base->STA = PWM_STA_ERRS; //clear PWM errors
	Delay::Microseconds(100);

	pwm->Base.Base->DMAC = PWM_DMAC_EN | PWM_DMAC_DREQ(PWM_FIFO_SIZE) | PWM_DMAC_PANIC(PWM_FIFO_SIZE); //DREQ is activated at queue < PWM_FIFO_SIZE
	pwm->Base.Base->RNG1 = BITS_PER_CLOCK; //used only for timing purposes; #writes to PWM FIFO/sec = PWM CLOCK / RNG1
	pwm->Base.Base->CTL = PWM_CTL_REPEATEMPTY1 | PWM_CTL_ENABLE1 | PWM_CTL_USEFIFO1;
}

void Test::DmaMemoryToMemory(Dma* dma, uint8_t count)
{
	DmaMemory dmaMemory;

	// This may be used by the sys PWM but.. we are having our own custom use
	// case for the PWM so as long as no other software is using the built in
	// PWM driver we should be OK.
	int dmaChan = 5;

	DmaMem_t* dmaSource = dmaMemory.AllocDmaPage();
	DmaMem_t* dmaDest = dmaMemory.AllocDmaPage();

	//write a few bytes to the source page:
	volatile char* srcArray = (volatile char*)dmaSource->virtual_addr;
	srcArray[0] = 'h';
	srcArray[1] = 'e';
	srcArray[2] = 'l';
	srcArray[3] = 'l';
	srcArray[4] = 'o';
	srcArray[5] = ' ';
	srcArray[6] = 'w';
	srcArray[7] = 'o';
	srcArray[8] = 'r';
	srcArray[9] = 'l';
	srcArray[10] = 'd';
	srcArray[11] = ' ';
	srcArray[12] = count;
	srcArray[13] = 0; //null terminator used for printf call.

	//allocate 1 page for the control blocks	
	DmaMem_t* dmaControl = dmaMemory.AllocDmaPage();

	//dedicate the first 8 words of this page to holding the cb.
	DmaControlBlock *cb1 = (DmaControlBlock*)dmaControl->virtual_addr;

	//fill the control block:
	cb1->TI = DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC; //after each byte copied, we want to increment the source and destination address of the copy, otherwise we'll be copying to the same address.
	cb1->SOURCE_AD = (uint32_t)dmaSource->bus_addr; //set source and destination DMA address
	cb1->DEST_AD = (uint32_t)dmaDest->bus_addr;
	cb1->TXFR_LEN = 14; //transfer 12 bytes
	cb1->STRIDE = 0; //no 2D stride
	cb1->NEXTCONBK = 0; //no next control block

	printf("destination was initially: '%s'\n", (volatile char*)dmaDest->virtual_addr);
	dma->Base.Base->ENABLE |= 1 << dmaChan;

	// make sure to disable dma first.
	dma->Base.Base->Chan[dmaChan].CS = DMA_CS_RESET;

	// Wait until the DMA is disabled.
	do {} while ((dma->Base.Base->Chan[dmaChan].CS & 0x01) > 0);

	// Start the transfer
	dma->Base.Base->Chan[dmaChan].DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR; // clear debug error flags
	dma->Base.Base->Chan[dmaChan].CONBLK_AD = (uint32_t)dmaControl->bus_addr; //we have to point it to the PHYSICAL address of the control block (cb1)
	dma->Base.Base->Chan[dmaChan].CS = DMA_CS_ACTIVE; //set active bit, but everything else is 0.

	// WAit until the transfer is complete.
	do {} while ((dma->Base.Base->Chan[dmaChan].CS & 0x01) > 0);

	// Display the result. (Character 12 increments each time through the loop)
	printf("destination reads: '%s'\n", (volatile char*)dmaDest->virtual_addr);

	// Validate the response...
	for (int i = 0; i < 14; i++)
	{
		if (*((volatile char*)dmaDest->virtual_addr + i) != srcArray[i])
		{
			printf("\n");
			printf("dma failed %d src: %c, dst: %c\n", i, srcArray[i], *((volatile char*)dmaDest->virtual_addr));
			printf("source reads: '%s'\n", (volatile char*)dmaSource->virtual_addr);
			printf("destination reads: '%s'\n", (volatile char*)dmaDest->virtual_addr);
			printf("\n");
			break;
		}
	}

	dmaMemory.FreeDmaPage(dmaSource);
	dmaMemory.FreeDmaPage(dmaDest);
	dmaMemory.FreeDmaPage(dmaControl);
}
