/*
 * Delay.h:
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
#include "../Headers/Delay.h"
#include "../Headers/Clock.h"
#include "../Headers/Dma.h"
#include "../Headers/DmaMemory.h"
#include "../Headers/Gpio.h"
#include "../Headers/Pwm.h"

#include "../Headers/Display.h"

#include "Tests.h"

void Test::WritePin(Gpio& gpio, int pin)
{
	gpio.SetPinMode(pin, PinMode::Output);

	// Toggle the pin
	gpio.WritePin(pin, PinState::High);
	Delay::Milliseconds(1000);
	gpio.WritePin(pin, PinState::Low);
}

void Test::ReadPin(Gpio& gpio, int pin)
{
// Suppressing the warning for illustrative purposes
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
	gpio.SetPinMode(pin, PinMode::Input);
	gpio.SetPudMode(pin, PudMode::PullUp);	

	// Read the pin
	volatile PinState state = gpio.ReadPin(pin);

	// Optional do something to change state
	// Press and hold a button perhaps...
	Delay::Milliseconds(1000);
	state = gpio.ReadPin(pin);
#pragma GCC diagnostic pop
}

void Test::ClockEnableDisable(Clock& clock)
{
	clock.Disable();
	//clock.Enable();
}

void Test::PwmTest(Pwm& pwm)
{
	pwm.Base->DMAC = 0; //disable DMA
	pwm.Base->CTL |= PWM_CTL_CLRFIFO; //clear pwm
	Delay::Microseconds(100);

	pwm.Base->STA = PWM_STA_ERRS; //clear PWM errors
	Delay::Microseconds(100);

	pwm.Base->DMAC = PWM_DMAC_EN | PWM_DMAC_DREQ(PWM_FIFO_SIZE) | PWM_DMAC_PANIC(PWM_FIFO_SIZE); //DREQ is activated at queue < PWM_FIFO_SIZE
	pwm.Base->RNG1 = BITS_PER_CLOCK; //used only for timing purposes; #writes to PWM FIFO/sec = PWM CLOCK / RNG1
	pwm.Base->CTL = PWM_CTL_REPEATEMPTY1 | PWM_CTL_ENABLE1 | PWM_CTL_USEFIFO1;
}

void Test::DmaMemoryToMemory(Dma& dma, uint8_t count)
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
	dma.Base->ENABLE |= 1 << dmaChan;

	// make sure to disable dma first.
	dma.Base->Chan[dmaChan].CS = DMA_CS_RESET;

	// Wait until the DMA is disabled.
	do {} while ((dma.Base->Chan[dmaChan].CS & 0x01) > 0);

	// Start the transfer
	dma.Base->Chan[dmaChan].DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR; // clear debug error flags
	dma.Base->Chan[dmaChan].CONBLK_AD = (uint32_t)dmaControl->bus_addr; //we have to point it to the PHYSICAL address of the control block (cb1)
	dma.Base->Chan[dmaChan].CS = DMA_CS_ACTIVE; //set active bit, but everything else is 0.

	// WAit until the transfer is complete.
	do {} while ((dma.Base->Chan[dmaChan].CS & 0x01) > 0);

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

void Test::Display(FourDigitSevenSegmentDisplay& display)
{
	static int count;

	while (count < 8000)
	{
		display.SetDisplayValue(count++);
		display.Display();
	}
}