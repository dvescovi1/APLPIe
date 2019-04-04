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
#include <sys/types.h>
#include <stddef.h>
#include <unistd.h>
#include <signal.h>

#include "../Headers/Delay.h"
#include "../Headers/Clock.h"
#include "../Headers/Dma.h"
#include "../Headers/DmaMemory.h"
#include "../Headers/Gpio.h"
#include "../Headers/Pwm.h"

#include "../Headers/Display.h"
#include "../Headers/PulseGenerator.h"

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

static uint32_t interruptCount = 0;
static bool interruptActive = false;

void IntTestIsr(void* arg)
{	
	interruptCount++;
	interruptActive = true;	
}

void Test::InterruptTest(Gpio& gpio)
{
	gpio.SetIsr(6,
		IntTrigger::Rising,
		IntTestIsr,
		(void*) NULL);
	uint32_t spinCount = 0;
	interruptActive = true;
	do
	{
		if (interruptActive)
		{
			gpio.WritePin(6, PinState::Low);
			for (int i = 0; i < 150; i++);
			interruptActive = false;
			gpio.WritePin(6, PinState::High);
		}
		else
		{
			spinCount++;
		}

	} while (interruptCount < 10000);

	DBG("interruptCount = %u, spinCount = %u",
		interruptCount,
		spinCount);
	gpio.ClearIsr(6);
}

void Test::ClockEnableDisable(Clock& clock)
{
	clock.PwmDisable();
	clock.PwmEnable();
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
	DmaControlBlock *cb1 = (DmaControlBlock*)dmaControl->virtual_addr;

	//fill the control block:
	cb1->TI = DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC; //after each byte copied, we want to increment the source and destination address of the copy, otherwise we'll be copying to the same address.
	cb1->SOURCE_AD = (uint32_t)dmaSource->bus_addr; //set source and destination DMA address
	cb1->DEST_AD = (uint32_t)dmaDest->bus_addr;
	cb1->TXFR_LEN = 14; //transfer 12 bytes
	cb1->STRIDE = 0; //no 2D stride
	cb1->NEXTCONBK = 0; //no next control block

	printf("destination was initially: '%s'\n", (volatile char*)dmaDest->virtual_addr);
	dma.Stop(dmaChan);
	dma.Start(dmaChan, (uint32_t)dmaControl->bus_addr);

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

#define SOURCE_BUFFER_FRAMES 42
#define CLOCK_DIVTEST 80 //# to divide the NOMINAL_CLOCK_FREQ by before passing it to the PWM peripheral.
//gpio frames per second is a product of the nominal clock frequency divided by BITS_PER_CLOCK and divided again by CLOCK_DIV
//At 500,000 frames/sec, memory bandwidth does not appear to be an issue (jitter of -1 to +2 uS)
//attempting 1,000,000 frames/sec results in an actual 800,000 frames/sec, though with a lot of jitter.
//Note that these numbers might very with heavy network or usb usage.
//  eg at 500,000 fps, with 1MB/sec network download, jitter is -1 to +30 uS
//     at 250,000 fps, with 1MB/sec network download, jitter is only -3 to +3 uS
void logDmaChannelHeader(DmaChannel *h)
{
	printf("Dma Ch Header:\n CS: 0x%08x\n CONBLK_AD: 0x%08x\n TI: 0x%08x\n SOURCE_AD: 0x%08x\n DEST_AD: 0x%08x\n TXFR_LEN: %u\n STRIDE: 0x%08x\n NEXTCONBK: 0x%08x\n DEBUG: 0x%08x\n",
		h->CS,
		h->CONBLK_AD,
		h->TI,
		h->SOURCE_AD,
		h->DEST_AD,
		h->TXFR_LEN,
		h->STRIDE,
		h->NEXTCONBK,
		h->DEBUG);
}

void logDmaControlBlock(struct DmaControlBlock *b) {
	printf("Dma Control Block:\n TI: 0x%08x\n SOURCE_AD: 0x%08x\n DEST_AD: 0x%08x\n TXFR_LEN: 0x%08x\n STRIDE: 0x%08x\n NEXTCONBK: 0x%08x\n unused: 0x%08x %08x\n", b->TI, b->SOURCE_AD, b->DEST_AD, b->TXFR_LEN, b->STRIDE, b->NEXTCONBK, b->_reserved[0], b->_reserved[1]);
}

struct GpioBufferFrame {
	//custom structure used for storing the GPIO buffer.
	//These BufferFrame's are DMA'd into the GPIO memory, potentially using the DmaEngine's Stride facility
	uint32_t gpset[2];
	uint32_t gpclr[2];
};

void Test::DmaGpio(Dma& dma, Gpio& gpio, int outPin0, int outPin1)
{
	//now set our pin as an output:
	gpio.SetPinMode(outPin0, PinMode::Output);
	gpio.SetPinMode(outPin1, PinMode::Output);
	
	// SynchTestPulse to be sure i/o is working
	gpio.WritePin(outPin0, PinState::High);
	gpio.WritePin(outPin1, PinState::High);

	// Give scope enough time to see DMA as a
	// second event.  This will vary and may
	// or may not be needed for your scope.
	Delay::Milliseconds(500);

	// configure DMA...
	// Just DMA one page worth:
	// Dma control block is bigger in size so use it to determine num Blocks.
	size_t numSrcBlocks = getpagesize() / sizeof(struct DmaControlBlock);

	DmaMemory dmaMemory;
	DmaMem_t* srcPage = dmaMemory.AllocDmaPage();	
	printf("mappedPhysSrcPage: virt %p phys %p\n",
		srcPage->virtual_addr,
		srcPage->bus_addr);

	GpioBufferFrame* gpioFrameBase = (GpioBufferFrame*) srcPage->virtual_addr;

	// Program the blocks to toggle pins
	for (size_t i = 0; i < numSrcBlocks; i++)
	{
		if (i % 2 == 0)
		{ 
			GpioBufferFrame* item = gpioFrameBase + i;
			item->gpclr[0] =  1 << outPin0;
			item->gpset[0] =  1 << outPin1;
		}
		else
		{
			GpioBufferFrame* item = gpioFrameBase + i;
			item->gpclr[0] =  1 << outPin1;
			item->gpset[0] =  1 << outPin0;
		}
	}

	//allocate memory for the control blocks
	size_t cbPageBytes = numSrcBlocks * sizeof(DmaControlBlock);
	DmaMem_t* cbPage = dmaMemory.AllocDmaPage();

	// pointer for user mode access
	struct DmaControlBlock *cbBaseVirt = (struct DmaControlBlock*)cbPage->virtual_addr;

	// pointer for DMA controller.
	volatile DmaControlBlock *cbBasePhys = (struct DmaControlBlock*)cbPage->bus_addr;
	
	// Program control blocks
	for (size_t i = 0; i < numSrcBlocks; i++)
	{
		GpioBufferFrame* sourceAddress = (GpioBufferFrame*)srcPage->bus_addr + i;
		cbBaseVirt[i].TI = DMA_CB_TI_SRC_INC |
			DMA_CB_TI_DEST_INC |
			DMA_CB_TI_NO_WIDE_BURSTS |
			DMA_CB_TI_TDMODE;
		cbBaseVirt[i].SOURCE_AD = (uint32_t)(sourceAddress);
		cbBaseVirt[i].DEST_AD = GPIO_BASE_BUS + offsetof(GpioRegisters, GPSET0);
		cbBaseVirt[i].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
		cbBaseVirt[i].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);
		
		uint32_t* nextIdx = (i + 1) < numSrcBlocks ?
			(uint32_t*)(cbBasePhys + (i + 1)) :
			0;
		cbBaseVirt[i].NEXTCONBK = (uint32_t)nextIdx;
	}
	for (size_t i = 0; i < cbPageBytes; i += getpagesize()) {
		printf("virt cb[%i] -> virt: 0x%08x phys: (0x%08x)\n",
			i,
			cbPage->virtual_addr,
			cbPage->bus_addr);
	}
	
	int channel = 5;
	
	dma.EnableChannel(channel);
	dma.Stop(channel);
	
		
	uint32_t firstAddr = cbPage->bus_addr;
	dma.Start(channel, firstAddr);

	volatile DmaChannel* chan = dma.Base->Chan + channel;
	int count = 0;
	do
	{
		logDmaChannelHeader((DmaChannel*)chan);
		count++;

	}while (chan->CS & DMA_CS_ACTIVE);
	
	
	dma.Stop(channel);
	printf("Exiting cleanly %d:\n", count);
	
	dmaMemory.FreeDmaPage(cbPage);
	dmaMemory.FreeDmaPage(srcPage);
}

void Test::DmaGpioPwmGated(Dma& dma, Pwm& pwm, Clock& clock, Gpio& gpio, int outPin0, int outPin1)
{
	//now set our pin as an output:
	gpio.SetPinMode(outPin0, PinMode::Output);
	gpio.SetPinMode(outPin1, PinMode::Output);

	// SynchTestPulse to be sure i/o is working
	gpio.WritePin(outPin0, PinState::High);
	gpio.WritePin(outPin1, PinState::High);

	// Give scope enough time to see DMA as a
	// second event.  This will vary and may
	// or may not be needed for your scope.
	Delay::Milliseconds(500); // Also nice place for breakpoint :o
	
	//configure DMA...
	//First, allocate memory for the source:
	size_t numSrcBlocks = SOURCE_BUFFER_FRAMES; //We want apx 1M blocks/sec.
	
	DmaMemory dmaMemory;
	DmaMem_t* srcPage = dmaMemory.AllocDmaPage();
	printf("mappedPhysSrcPage: virt %p phys %p\n",
		srcPage->virtual_addr,
		srcPage->bus_addr);

	//cast virtSrcPage to a GpioBufferFrame array:
	struct GpioBufferFrame *srcArray = (GpioBufferFrame*)srcPage->bus_addr; //Note: calling virtToPhys on srcArray will return NULL. Use srcArrayCached for that.
	GpioBufferFrame *srcArrayCached = (GpioBufferFrame*)srcPage->virtual_addr;

	for (size_t i = 0; i < numSrcBlocks; i++)
	{
		if (i % 2 == 0)
		{
			volatile GpioBufferFrame* item = srcArrayCached + i;
			item->gpclr[0] = 1 << outPin0;
			item->gpset[0] = 1 << outPin1;
		}
		else
		{
			volatile GpioBufferFrame* item = srcArrayCached + i;
			item->gpclr[0] = 1 << outPin1;
			item->gpset[0] = 1 << outPin0;
		}
	}
	
	//configure PWM clock:
	clock.PwmDisable();
	clock.PwmSetDivider(CLOCK_DIVTEST);
	clock.PwmEnable();
		
	pwm.Stop();	
	pwm.Start(BITS_PER_CLOCK);

	DmaMem_t* cbPage = dmaMemory.AllocDmaPage();

	//fill the control blocks:
	volatile struct DmaControlBlock *cbArrCached = (struct DmaControlBlock*)cbPage->virtual_addr;
	struct DmaControlBlock *cbArr = (struct DmaControlBlock*)cbPage->bus_addr;

	printf("#dma blocks: %i, #src blocks: %i\n", numSrcBlocks * 2, numSrcBlocks);
	for (size_t i = 0; i < numSrcBlocks * 2; i += 2)
	{
		//pace DMA through PWM
		cbArrCached[i].TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_NO_WIDE_BURSTS | DMA_CB_TI_TDMODE;
		cbArrCached[i].SOURCE_AD = (uint32_t)(srcArray + (i / 2)); //The data written doesn't matter, but using the GPIO source will hopefully bring it into L2 for more deterministic timing of the next control block.
		cbArrCached[i].DEST_AD = PWM_BASE_BUS + offsetof(PwmRegisters, FIF1); //write to the FIFO
		cbArrCached[i].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(1) | DMA_CB_TXFR_LEN_XLENGTH(4);
		cbArrCached[i].STRIDE = i / 2;
		cbArrCached[i].NEXTCONBK = (uint32_t)(cbArr + (i + 1));
		//copy buffer to GPIOs
		cbArrCached[i + 1].TI = DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC | DMA_CB_TI_NO_WIDE_BURSTS | DMA_CB_TI_TDMODE;
		cbArrCached[i + 1].SOURCE_AD = (uint32_t)(srcArray + (i / 2));
		cbArrCached[i + 1].DEST_AD = GPIO_BASE_BUS + offsetof(GpioRegisters, GPSET0);
		cbArrCached[i + 1].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
		cbArrCached[i + 1].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);
		uint32_t* nextIdx = (i + 2) < (numSrcBlocks * 2) ?
			(uint32_t*) (cbArr + (i + 2)) :
			0; //last block should loop back to the first block
		cbArrCached[i + 1].NEXTCONBK = (uint32_t) nextIdx; //(uint32_t)physCbPage + ((void*)&cbArr[(i+2)%maxIdx] - virtCbPage);
	}
	printf("virt cb -> virt: 0x%08x phys: (0x%08x)\n",
		cbPage->virtual_addr,
		cbPage->bus_addr);
	
	//source: http://virtualfloppy.blogspot.com/2014/01/dma-support-at-last.html
	//cat /sys/module/dma/parameters/dmachans gives a bitmask of DMA channels that are not used by GPU. Results: ch 1, 3, 6, 7 are reserved.
	//dmesg | grep "DMA"; results: Ch 2 is used by SDHC host
	//ch 0 is known to be used for graphics acceleration
	//Thus, applications can use ch 4, 5, or the LITE channels @ 8 and beyond.
	//If using LITE channels, then we can't use the STRIDE feature, so that narrows it down to ch 4 and ch 5.
	int channel = 5;
	
	dma.EnableChannel(channel);
	dma.Stop(channel);
	
	uint32_t firstAddr = cbPage->bus_addr;
	dma.Start(channel, firstAddr);

	volatile DmaChannel* chan = dma.Base->Chan + channel;
	while (chan->CS & DMA_CS_ACTIVE)
	{
		logDmaChannelHeader((DmaChannel*)chan);
	}
	
	dma.Stop(channel);
	printf("Exiting cleanly:\n");
	
	dmaMemory.FreeDmaPage(cbPage);
	dmaMemory.FreeDmaPage(srcPage);
}

// just hack for unit testing...
static int maxCountMemory = 1000;
static int currentCountMemory = 0;
volatile static uint32_t* stopAddressMemory;

void OutPin0Isr(void* arg)
{
	// Here you reprogram the buffer that is not currently being executed
	// and / or put the next address of the last control block to zero
	// when you want the Dma to stop.
	currentCountMemory++;
	if (currentCountMemory > maxCountMemory)
	{
		*stopAddressMemory = 0;
	}
}

void Test::DmaMemoryToMemoryDoubleBuffered(Dma& dma, Gpio& gpio, int outPin0)
{
	//now set our pin as an output:
	gpio.SetPinMode(outPin0, PinMode::Output);

	// SynchTestPulse to be sure i/o is working
	gpio.WritePin(outPin0, PinState::High);
	gpio.WritePin(outPin0, PinState::Low);
	gpio.WritePin(outPin0, PinState::High);
	gpio.WritePin(outPin0, PinState::Low);


	// Give scope enough time to see DMA as a
	// second event.  This will vary and may
	// or may not be needed for your scope.
	Delay::Milliseconds(500); // Also nice place for breakpoint :o

	// To double buffer the software will reserve
	// an I/O pin to toggle when it is going to switch
	// to the second buffer.  This way we can attach
	// an interrupt handler to it.
	gpio.SetIsr(outPin0,
		IntTrigger::Both,
		OutPin0Isr,
		NULL);
		
	// configure DMA...
	// This may be used by the sys PWM but.. we are having our own custom use
	// case for the PWM so as long as no other software is using the built in
	// PWM driver we should be OK.
	int dmaChan = 5;

	size_t pageSize = getpagesize();
	size_t numSrcBlocks = pageSize / sizeof(struct DmaControlBlock);	

	DmaMemory dmaMemory;	
	DmaMem_t* dmaControlBlocks = dmaMemory.AllocDmaPage();
		
	DmaMem_t* sourceMemoryFramePages[numSrcBlocks];
	DmaMem_t* destMemoryFramePages[numSrcBlocks];

	for (size_t i = 0; i < numSrcBlocks; i++)
	{

		sourceMemoryFramePages[i] = dmaMemory.AllocDmaPage();
		destMemoryFramePages[i] = dmaMemory.AllocDmaPage();
		printf("gpioFramePage%d: virt %p phys %p\n",
			i,
			sourceMemoryFramePages[i]->virtual_addr,
			sourceMemoryFramePages[i]->bus_addr);

		if (i == (numSrcBlocks - 1) / 2)
		{
			// Ideally we would not reserve a whole page for one GpioFrame but this
			// is only for illustrative purposes  and simplifies the code.
			GpioBufferFrame* item0 = (GpioBufferFrame*) sourceMemoryFramePages[i]->virtual_addr;
			item0->gpset[0] = 1 << outPin0;
			item0->gpset[1] = 0;
			item0->gpclr[0] = 0;
			item0->gpclr[1] = 0;
		}
		else if (i == numSrcBlocks - 1)
		{
			// Ideally we would not reserve a whole page for one GpioFrame but this
			// is only for illustrative purposes  and simplifies the code.
			GpioBufferFrame* item1 = (GpioBufferFrame*)sourceMemoryFramePages[i]->virtual_addr;
			item1->gpset[0] = 0;
			item1->gpset[1] = 0;
			item1->gpclr[0] = 1 << outPin0;
			item1->gpclr[1] = 0;
		}
		else
		{
			unsigned char data = 0;
			volatile unsigned char* pData = (volatile unsigned char*)sourceMemoryFramePages[i]->virtual_addr;

			for (size_t j = 0; j < pageSize; j++)
			{
				*pData = data++;
				pData++;
			}
		}
	}
		
	DmaControlBlock *virtCb = (DmaControlBlock*) (dmaControlBlocks->virtual_addr);
	DmaControlBlock *physCb = (DmaControlBlock*)(dmaControlBlocks->bus_addr);

	for (size_t i = 0; i < numSrcBlocks; i++)
	{
		//fill the control block:
		virtCb->TI = DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC; //after each byte copied, we want to increment the source and destination address of the copy, otherwise we'll be copying to the same address.
		virtCb->SOURCE_AD = (uint32_t)sourceMemoryFramePages[i]->bus_addr; //set source and destination DMA address

		if (i == (numSrcBlocks - 1) / 2 ||
			i == numSrcBlocks - 1)
		{
			virtCb->TI = DMA_CB_TI_SRC_INC |
				DMA_CB_TI_DEST_INC |
				DMA_CB_TI_NO_WIDE_BURSTS |
				DMA_CB_TI_TDMODE;
			virtCb->DEST_AD = GPIO_BASE_BUS + offsetof(GpioRegisters, GPSET0);
			virtCb->TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
			virtCb->STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);
		}
		else
		{
			virtCb->DEST_AD = (uint32_t)destMemoryFramePages[i]->bus_addr;
			virtCb->TXFR_LEN = pageSize;
			virtCb->STRIDE = 0; //no 2D stride
		}

		physCb++;
		virtCb->NEXTCONBK = (uint32_t)physCb;
		if (i == numSrcBlocks - 1)
		{			
			stopAddressMemory = &virtCb->NEXTCONBK;
			virtCb->NEXTCONBK = (uint32_t) dmaControlBlocks->bus_addr;
		}
		virtCb++;
	}

	dma.Stop(dmaChan);
	dma.Start(dmaChan, (uint32_t)dmaControlBlocks->bus_addr);

	// WAit until the transfer is complete.
	do {} while ((dma.Base->Chan[dmaChan].CS & 0x01) > 0);
	do {} while (currentCountMemory < maxCountMemory);

	// Validate the response...
	for (size_t i = 0; i < numSrcBlocks; i++)
	{
		for (size_t j = 0; j < pageSize; j++)
		{
			volatile char* source = (volatile char*) sourceMemoryFramePages[i]->virtual_addr + j;
			volatile char* dest = (volatile char*) destMemoryFramePages[i]->virtual_addr + j;

			if ( *dest != *source)
			{
				printf("\n");
				printf("dma failed %d, %d src: %c, dst: %c\n", i, j, *source, *dest);
				printf("\n");
				break;
			}
		}
	}

	for (size_t i = 0; i < numSrcBlocks; i++)
	{
		dmaMemory.FreeDmaPage(sourceMemoryFramePages[i]);
		dmaMemory.FreeDmaPage(destMemoryFramePages[i]);
	}
	gpio.ClearIsr(outPin0);
}

// just hack for unit testing...
static int maxCountGpio = 1000;
static int currentCountGpio = 0;
volatile static uint32_t* stopAddressGpio;

void OutPin1Isr(void* arg)
{
	// Here you reprogram the buffer that is not currently being executed
	// and / or put the next address of the last control block to zero
	// when you want the Dma to stop.
	currentCountGpio++;
	if (currentCountGpio > maxCountGpio)
	{
		*stopAddressGpio = 0;
	}
}

void Test::DmaGpioDoubleBuffered(Dma& dma, Gpio& gpio, int outPin0, int outPin1)
{	
	//now set our pin as an output:
	gpio.SetPinMode(outPin0, PinMode::Output);
	gpio.SetPinMode(outPin1, PinMode::Output);

	// SynchTestPulse to be sure i/o is working
	gpio.WritePin(outPin0, PinState::High);
	gpio.WritePin(outPin1, PinState::High);
		
	// Give scope enough time to see DMA as a
	// second event.  This will vary and may
	// or may not be needed for your scope.
	Delay::Milliseconds(500); // Also nice place for breakpoint :o

	// To double buffer the software will reserve
	// an I/O pin to toggle when it is going to switch
	// to the second buffer.  This way we can attach
	// an interrupt handler to it.
	gpio.SetIsr(outPin1,
		IntTrigger::Rising,
		OutPin1Isr,
		NULL);

	// configure DMA...
	// Just DMA one page worth:
	// Dma control block is bigger in size so use it to determine num Blocks.
	size_t numSrcBlocks = getpagesize() / sizeof(struct DmaControlBlock);

	DmaMemory dmaMemory;
	DmaMem_t* gpioFramePage0 = dmaMemory.AllocDmaPage();
	printf("gpioFramePage0: virt %p phys %p\n",
		gpioFramePage0->virtual_addr,
		gpioFramePage0->bus_addr);
	DmaMem_t* gpioFramePage1 = dmaMemory.AllocDmaPage();
	printf("gpioFramePage1: virt %p phys %p\n",
		gpioFramePage1->virtual_addr,
		gpioFramePage1->bus_addr);

	GpioBufferFrame* gpioFrameBase0 = (GpioBufferFrame*)gpioFramePage0->virtual_addr;
	GpioBufferFrame* gpioFrameBase1 = (GpioBufferFrame*)gpioFramePage1->virtual_addr;

	// Program the blocks to toggle pins
	for (size_t i = 0; i < numSrcBlocks; i++)
	{

		// With this arrangment the buffer will generate
		// a falling edge on Pin 1 at the end of this buffer.
		// and a rising edge at the start.  To filter the
		// interrupt for the first transfer set pin 1 to
		// high before the Dma starts.
		if (i == 0)
		{
			GpioBufferFrame* item0 = gpioFrameBase0 + i;
			item0->gpset[0] = 1 << outPin1;

			GpioBufferFrame* item1 = gpioFrameBase1 + i;
			item1->gpset[0] = 1 << outPin1;
		}
		if (i == (numSrcBlocks - 1))
		{
			GpioBufferFrame* item0 = gpioFrameBase0 + i;
			item0->gpclr[0] = 1 << outPin1;

			GpioBufferFrame* item1 = gpioFrameBase1 + i;
			item1->gpclr[0] = 1 << outPin1;
		}
	
		if (i % 2 == 0)
		{
			GpioBufferFrame* item0 = gpioFrameBase0 + i;
			item0->gpclr[0] = 1 << outPin0;

			GpioBufferFrame* item1 = gpioFrameBase1 + i;
			item1->gpclr[0] = 1 << outPin0;
		}
		else
		{
			GpioBufferFrame* item0 = gpioFrameBase0 + i;
			item0->gpset[0] = 1 << outPin0;

			GpioBufferFrame* item1 = gpioFrameBase1 + i;
			item1->gpset[0] = 1 << outPin0;
		}
	}

	//allocate memory for the control blocks
	size_t cbPageBytes = numSrcBlocks * sizeof(DmaControlBlock);
	DmaMem_t* cbPage0 = dmaMemory.AllocDmaPage();
	DmaMem_t* cbPage1 = dmaMemory.AllocDmaPage();

	// pointer for user mode access
	struct DmaControlBlock *cbBase0Virt = (struct DmaControlBlock*)cbPage0->virtual_addr;
	struct DmaControlBlock *cbBase1Virt = (struct DmaControlBlock*)cbPage1->virtual_addr;

	// pointer for DMA controller.
	volatile DmaControlBlock *cbBase0Phys = (struct DmaControlBlock*)cbPage0->bus_addr;
	volatile DmaControlBlock *cbBase1Phys = (struct DmaControlBlock*)cbPage1->bus_addr;

	// Program control blocks
	for (size_t i = 0; i < numSrcBlocks; i++)
	{
		GpioBufferFrame* sourceAddress0 = (GpioBufferFrame*)gpioFramePage0->bus_addr + i;
		cbBase0Virt[i].TI = DMA_CB_TI_SRC_INC |
			DMA_CB_TI_DEST_INC |
			DMA_CB_TI_NO_WIDE_BURSTS |
			DMA_CB_TI_TDMODE;
		cbBase0Virt[i].SOURCE_AD = (uint32_t)(sourceAddress0);
		cbBase0Virt[i].DEST_AD = GPIO_BASE_BUS + offsetof(GpioRegisters, GPSET0);
		cbBase0Virt[i].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
		cbBase0Virt[i].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);

		uint32_t* nextIdx0 = (i + 1) < numSrcBlocks ?
			(uint32_t*)(cbBase0Phys + (i + 1)) :
			0;
		cbBase0Virt[i].NEXTCONBK = (uint32_t)nextIdx0;

		GpioBufferFrame* sourceAddress1 = (GpioBufferFrame*)gpioFramePage1->bus_addr + i;
		cbBase1Virt[i].TI = DMA_CB_TI_SRC_INC |
			DMA_CB_TI_DEST_INC |
			DMA_CB_TI_NO_WIDE_BURSTS |
			DMA_CB_TI_TDMODE;
		cbBase1Virt[i].SOURCE_AD = (uint32_t)(sourceAddress1);
		cbBase1Virt[i].DEST_AD = GPIO_BASE_BUS + offsetof(GpioRegisters, GPSET0);
		cbBase1Virt[i].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
		cbBase1Virt[i].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);

		uint32_t* nextIdx1 = (i + 1) < numSrcBlocks ?
			(uint32_t*)(cbBase1Phys + (i + 1)) :
			0;
		cbBase1Virt[i].NEXTCONBK = (uint32_t)nextIdx1;
	}

	// here we connext the end of the first buffer to the beginning
	// of the second buffer and the end of the second buffer to the
	// beginning of the first buffer.
	cbBase0Virt[numSrcBlocks - 1].NEXTCONBK = (uint32_t)cbBase1Phys;
	cbBase1Virt[numSrcBlocks - 1].NEXTCONBK = (uint32_t)cbBase0Phys;

	volatile uint32_t* lastIdx = &(cbBase1Virt[numSrcBlocks - 1].NEXTCONBK);
	stopAddressGpio = lastIdx;

	for (size_t i = 0; i < cbPageBytes; i += getpagesize()) {
		printf("virt cb0[%i] -> virt: 0x%08x phys: (0x%08x)\n",
			i,
			cbPage0->virtual_addr,
			cbPage0->bus_addr);
		printf("virt cb1[%i] -> virt: 0x%08x phys: (0x%08x)\n",
			i,
			cbPage1->virtual_addr,
			cbPage1->bus_addr);
	}

	int channel = 5;

	dma.EnableChannel(channel);
	dma.Stop(channel);


	uint32_t firstAddr = cbPage0->bus_addr;
	dma.Start(channel, firstAddr);

	volatile DmaChannel* chan = dma.Base->Chan + channel;
	int count = 0;
	do
	{
		logDmaChannelHeader((DmaChannel*)chan);
		count++;
		printf("\ninterrupt count = %d\n\n", currentCountGpio);

	} while (chan->CS & DMA_CS_ACTIVE);


	dma.Stop(channel);
	printf("Exiting cleanly %d:\n", count);

	dmaMemory.FreeDmaPage(cbPage0);
	dmaMemory.FreeDmaPage(cbPage1);
	dmaMemory.FreeDmaPage(gpioFramePage0);
	dmaMemory.FreeDmaPage(gpioFramePage1);

	gpio.ClearIsr(outPin1);
}

void Test::FastestPulseTrain(PulseGenerator& pulseGenerator)
{
	// Give scope enough time to see DMA as a
	// second event.  This will vary and may
	// or may not be needed for your scope.
	Delay::Milliseconds(500); // Also nice place for breakpoint :o

	PulseTrain pulseTrain(1 << 5);

	for (int i = 0; i < 3; i++)
	{
		AddFastPulseTrain(pulseTrain);

		pulseTrain.Add(PinState::Low, MICROSEC_TO_RNG1(5));
		pulseTrain.Add(PinState::High, MICROSEC_TO_RNG1(5));
	}

	pulseGenerator.Add(pulseTrain);

	// start sync pin high so we see the end on
	// scope.
	pulseGenerator.WriteSyncPinState(PinState::Low);
	pulseGenerator.WriteSyncPinState(PinState::High);
	pulseGenerator.WriteSyncPinState(PinState::Low);
	pulseGenerator.WriteSyncPinState(PinState::High);

	pulseGenerator.Start();
	do {} while (pulseGenerator.IsRunning());
}

void Test::GeneratePulseTrain(PulseGenerator& pulseGenerator)
{
	// Give scope enough time to see DMA as a
	// second event.  This will vary and may
	// or may not be needed for your scope.
	Delay::Milliseconds(500); // Also nice place for breakpoint :o

	int c = 0;
	for (int j = 0; j < 5; j++)
	{
		PulseTrain pulseTrain(1 << 5);

		for (int i = 0; i < j; i++)
		{

			AddLongPulseTrain(pulseTrain);

			pulseTrain.Add(PinState::Low, MICROSEC_TO_RNG1(500));
			pulseTrain.Add(PinState::High, MICROSEC_TO_RNG1(500));
		}

		pulseGenerator.Add(pulseTrain);

		// start sync pin high so we see the end on
		// scope.
		pulseGenerator.WriteSyncPinState(PinState::Low);
		pulseGenerator.WriteSyncPinState(PinState::High);
		pulseGenerator.WriteSyncPinState(PinState::Low);
		pulseGenerator.WriteSyncPinState(PinState::High);

		pulseGenerator.Start();
		do {} while (pulseGenerator.IsRunning());
	}
}

void Test::AddLongPulseTrain(PulseTrain& pulseTrain)
{
	for (int i = 0; i < 12; i++)
	{
		pulseTrain.Add(PinState::Low, MICROSEC_TO_RNG1(58));
		pulseTrain.Add(PinState::High, MICROSEC_TO_RNG1(58));
	}

	pulseTrain.Add(PinState::Low, MICROSEC_TO_RNG1(100));
	pulseTrain.Add(PinState::High, MICROSEC_TO_RNG1(100));

	for (int i = 0; i < 8; i++)
	{
		pulseTrain.Add(PinState::Low, MICROSEC_TO_RNG1(58));
		pulseTrain.Add(PinState::High, MICROSEC_TO_RNG1(58));
	}

	pulseTrain.Add(PinState::Low, MICROSEC_TO_RNG1(100));
	pulseTrain.Add(PinState::High, MICROSEC_TO_RNG1(100));

	for (int i = 0; i < 8; i++)
	{
		pulseTrain.Add(PinState::Low, MICROSEC_TO_RNG1(100));
		pulseTrain.Add(PinState::High, MICROSEC_TO_RNG1(100));
	}

	pulseTrain.Add(PinState::Low, MICROSEC_TO_RNG1(100));
	pulseTrain.Add(PinState::High, MICROSEC_TO_RNG1(100));

	for (int i = 0; i < 8; i++)
	{
		pulseTrain.Add(PinState::Low, MICROSEC_TO_RNG1(58));
		pulseTrain.Add(PinState::High, MICROSEC_TO_RNG1(58));
	}

	pulseTrain.Add(PinState::Low, MICROSEC_TO_RNG1(58));
	pulseTrain.Add(PinState::High, MICROSEC_TO_RNG1(58));
}

void Test::AddFastPulseTrain(PulseTrain& pulseTrain)
{
	for (int i = 0; i < 40; i++)
	{
		pulseTrain.Add(PinState::Low, MICROSEC_TO_RNG1(0.06));
		pulseTrain.Add(PinState::High, MICROSEC_TO_RNG1(0.06));
	}	
}