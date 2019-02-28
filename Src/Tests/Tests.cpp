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

//config settings:
#define PWM_FIFO_SIZE 1 //The DMA transaction is paced through the PWM FIFO. The PWM FIFO consumes 1 word every N uS (set in clock settings). Once the fifo has fewer than PWM_FIFO_SIZE words available, it will request more data from DMA. Thus, a high buffer length will be more resistant to clock drift, but may occasionally request multiple frames in a short succession (faster than FRAME_PER_SEC) in the presence of bus contention, whereas a low buffer length will always space frames AT LEAST 1/FRAMES_PER_SEC seconds apart, but may experience clock drift.

// limit to one page for early coding
#define SOURCE_BUFFER_FRAMES 42
//#define SOURCE_BUFFER_FRAMES 8192 //number of gpio timeslices to buffer. These are processed at ~1 million/sec. So 1000 framse is 1 ms. Using a power-of-two is a good idea as it simplifies some of the arithmetic (modulus operations)
#define SCHED_PRIORITY 30 //Linux scheduler priority. Higher = more realtime

#define NOMINAL_CLOCK_FREQ 500000000 //PWM Clock runs at 500 MHz, unless overclocking
#define BITS_PER_CLOCK 10 //# of bits to be used in each PWM cycle. Effectively acts as a clock divisor for us, since the PWM clock is in bits/second
#define CLOCK_DIV 80 //# to divide the NOMINAL_CLOCK_FREQ by before passing it to the PWM peripheral.
//gpio frames per second is a product of the nominal clock frequency divided by BITS_PER_CLOCK and divided again by CLOCK_DIV
//At 500,000 frames/sec, memory bandwidth does not appear to be an issue (jitter of -1 to +2 uS)
//attempting 1,000,000 frames/sec results in an actual 800,000 frames/sec, though with a lot of jitter.
//Note that these numbers might very with heavy network or usb usage.
//  eg at 500,000 fps, with 1MB/sec network download, jitter is -1 to +30 uS
//     at 250,000 fps, with 1MB/sec network download, jitter is only -3 to +3 uS
#define FRAMES_PER_SEC NOMINAL_CLOCK_FREQ/BITS_PER_CLOCK/CLOCK_DIV
#define SEC_TO_FRAME(s) ((int64_t)(s)*FRAMES_PER_SEC)
#define USEC_TO_FRAME(u) (SEC_TO_FRAME(u)/1000000)
#define FRAME_TO_SEC(f) ((int64_t)(f)*BITS_PER_CLOCK*CLOCK_DIV/NOMINAL_CLOCK_FREQ)
#define FRAME_TO_USEC(f) FRAME_TO_SEC((int64_t)(f)*1000000)

#define TIMER_CLO    0x00000004 //lower 32-bits of 1 MHz timer
#define TIMER_CHI    0x00000008 //upper 32-bits


#define PAGE_SIZE 4096 //mmap maps pages of memory, so we must give it multiples of this size
#define GPFSEL0   0x00000000 //gpio function select. There are 6 of these (32 bit registers)
#define GPFSEL1   0x00000004
#define GPFSEL2   0x00000008
#define GPFSEL3   0x0000000c
#define GPFSEL4   0x00000010
#define GPFSEL5   0x00000014
//bits 2-0 of GPFSEL0: set to 000 to make Pin 0 an output. 001 is an input. Other combinations represent alternate functions
//bits 3-5 are for pin 1.
//...
//bits 27-29 are for pin 9.
//GPFSEL1 repeats, but bits 2-0 are Pin 10, 27-29 are pin 19.
//...
#define GPSET0    0x0000001C //GPIO Pin Output Set. There are 2 of these (32 bit registers)
#define GPSET1    0x00000020
//writing a '1' to bit N of GPSET0 makes that pin HIGH.
//writing a '0' has no effect.
//GPSET0[0-31] maps to pins 0-31
//GPSET1[0-21] maps to pins 32-53
#define GPCLR0    0x00000028 //GPIO Pin Output Clear. There are 2 of these (32 bits each)
#define GPCLR1    0x0000002C
//GPCLR acts the same way as GPSET, but clears the pin instead.
#define GPLEV0    0x00000034 //GPIO Pin Level. There are 2 of these (32 bits each)

//physical addresses for the DMA peripherals, as found in the processor documentation:
#define DMACH(n) (0x100*(n))
//DMA Channel register sets (format of these registers is found in DmaChannelHeader struct):
//#define DMACH0   0x00000000
//#define DMACH1   0x00000100
//#define DMACH2   0x00000200
//#define DMACH3   0x00000300
//...
//Each DMA channel has some associated registers, but only CS (control and status), CONBLK_AD (control block address), and DEBUG are writeable
//DMA is started by writing address of the first Control Block to the DMA channel's CONBLK_AD register and then setting the ACTIVE bit inside the CS register (bit 0)
//Note: DMA channels are connected directly to peripherals, so physical addresses should be used (affects control block's SOURCE, DEST and NEXTCONBK addresses).
#define DMAENABLE 0x00000ff0 //bit 0 should be set to 1 to enable channel 0. bit 1 enables channel 1, etc.

//flags used in the DmaChannelHeader struct:
#define DMA_CS_RESET (1<<31)
#define DMA_CS_ABORT (1<<30)
#define DMA_CS_DISDEBUG (1<<28) //DMA will not stop when debug signal is asserted
#define DMA_CS_PRIORITY(x) ((x)&0xf << 16) //higher priority DMA transfers are serviced first, it would appear
#define DMA_CS_PRIORITY_MAX DMA_CS_PRIORITY(7)
#define DMA_CS_PANIC_PRIORITY(x) ((x)&0xf << 20)
#define DMA_CS_PANIC_PRIORITY_MAX DMA_CS_PANIC_PRIORITY(7)
#define DMA_CS_END (1<<1)
#define DMA_CS_ACTIVE (1<<0)

#define DMA_DEBUG_READ_ERROR (1<<2)
#define DMA_DEBUG_FIFO_ERROR (1<<1)
#define DMA_DEBUG_READ_LAST_NOT_SET_ERROR (1<<0)

//flags used in the DmaControlBlock struct:
#define DMA_CB_TI_NO_WIDE_BURSTS (1<<26)
#define DMA_CB_TI_PERMAP_NONE (0<<16)
#define DMA_CB_TI_PERMAP_DSI  (1<<16)
//... (more found on page 61 of BCM2835 pdf
#define DMA_CB_TI_PERMAP_PWM  (5<<16)
//...
#define DMA_CB_TI_SRC_DREQ    (1<<10)
#define DMA_CB_TI_SRC_INC     (1<<8)
#define DMA_CB_TI_DEST_DREQ   (1<<6)
#define DMA_CB_TI_DEST_INC    (1<<4)
#define DMA_CB_TI_TDMODE      (1<<1)


//https://dev.openwrt.org/browser/trunk/target/linux/brcm2708/patches-3.10/0070-bcm2708_fb-DMA-acceleration-for-fb_copyarea.patch?rev=39770 says that YLENGTH should actually be written as # of copies *MINUS ONE*
#define DMA_CB_TXFR_LEN_YLENGTH(y) (((y-1)&0x4fff) << 16)
#define DMA_CB_TXFR_LEN_XLENGTH(x) ((x)&0xffff)
#define DMA_CB_TXFR_YLENGTH_MASK (0x4fff << 16)
#define DMA_CB_STRIDE_D_STRIDE(x)  (((x)&0xffff) << 16)
#define DMA_CB_STRIDE_S_STRIDE(x)  ((x)&0xffff)


//Dma Control Blocks must be located at addresses that are multiples of 32 bytes
#define DMA_CONTROL_BLOCK_ALIGNMENT 32 

#define PWM_CTL  0x00000000 //control register
#define PWM_STA  0x00000004 //status register
#define PWM_DMAC 0x00000008 //DMA control register
#define PWM_RNG1 0x00000010 //channel 1 range register (# output bits to use per sample)
#define PWM_DAT1 0x00000014 //channel 1 data
#define PWM_FIF1 0x00000018 //channel 1 fifo (write to this register to queue an output)
#define PWM_RNG2 0x00000020 //channel 2 range register
#define PWM_DAT2 0x00000024 //channel 2 data

#define PWM_CTL_USEFIFO2 (1<<13)
#define PWM_CTL_REPEATEMPTY2 (1<<10)
#define PWM_CTL_ENABLE2 (1<<8)
#define PWM_CTL_CLRFIFO (1<<6)
#define PWM_CTL_USEFIFO1 (1<<5)
#define PWM_CTL_REPEATEMPTY1 (1<<2)
#define PWM_CTL_ENABLE1 (1<<0)

#define PWM_STA_BUSERR (1<<8)
#define PWM_STA_GAPERRS (0xf << 4)
#define PWM_STA_FIFOREADERR (1<<3)
#define PWM_STA_FIFOWRITEERR (1<<2)
#define PWM_STA_ERRS PWM_STA_BUSERR | PWM_STA_GAPERRS | PWM_STA_FIFOREADERR | PWM_STA_FIFOWRITEERR

#define PWM_DMAC_EN (1<<31)
#define PWM_DMAC_PANIC(P) (((P)&0xff)<<8)
#define PWM_DMAC_DREQ(D) (((D)&0xff)<<0)

//The following is undocumented :( Taken from http://www.scribd.com/doc/127599939/BCM2835-Audio-clocks
#define CM_PWMCTL 0xa0
#define CM_PWMDIV 0xa4
//each write to CM_PWMTL and CM_PWMDIV requires the password to be written:
#define CM_PWMCTL_PASSWD 0x5a000000
#define CM_PWMDIV_PASSWD 0x5a000000
//MASH is used to achieve fractional clock dividers by introducing artificial jitter.
//if you want constant frequency (even if it may not be at 100% CORRECT frequency), use MASH0
//if clock divisor is integral, then there's no need to use MASH, and anything above MASH1 can introduce jitter.
#define CM_PWMCTL_MASH(x) (((x)&0x3) << 9)
#define CM_PWMCTL_MASH0 CM_PWMTRL_MASH(0)
#define CM_PWMCTL_MASH1 CM_PWMTRL_MASH(1)
#define CM_PWMCTL_MASH2 CM_PWMTRL_MASH(2)
#define CM_PWMCTL_MASH3 CM_PWMTRL_MASH(3)
#define CM_PWMCTL_FLIP (1<<8) //use to inverse clock polarity
#define CM_PWMCTL_BUSY (1<<7) //read-only flag that indicates clock generator is running.
#define CM_PWMCTL_KILL (1<<5) //write a 1 to stop & reset clock generator. USED FOR DEBUG ONLY
#define CM_PWMCTL_ENAB (1<<4) //gracefully stop/start clock generator. BUSY flag will go low once clock is off.
#define CM_PWMCTL_SRC(x) ((x)&0xf) //clock source. 0=gnd. 1=oscillator. 2-3=debug. 4=PLLA per. 5=PLLC per. 6=PLLD per. 7=HDMI aux. 8-15=GND
#define CM_PWMCTL_SRC_OSC CM_PWMCTL_SRC(1)
#define CM_PWMCTL_SRC_PLLA CM_PWMCTL_SRC(4)
#define CM_PWMCTL_SRC_PLLC CM_PWMCTL_SRC(5)
#define CM_PWMCTL_SRC_PLLD CM_PWMCTL_SRC(6)

//max clock divisor is 4095
#define CM_PWMDIV_DIVI(x) (((x)&0xfff) << 12)
#define CM_PWMDIV_DIVF(x) ((x)&0xfff)

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

struct PwmHeader {
	volatile uint32_t CTL;  // 0x00000000 //control register
		//16-31 reserved
		//15 MSEN2 (0: PWM algorithm, 1:M/S transmission used)
		//14 reserved
		//13 USEF2 (0: data register is used for transmission, 1: FIFO is used for transmission)
		//12 POLA2 (0: 0=low, 1=high. 1: 0=high, 1=low (inversion))
		//11 SBIT2; defines the state of the output when no transmission is in place
		//10 RPTL2; 0: transmission interrupts when FIFO is empty. 1: last data in FIFO is retransmitted when FIFO is empty
		//9  MODE2; 0: PWM mode. 1: serializer mode
		//8  PWMEN2; 0: channel is disabled. 1: channel is enabled
		//7  MSEN1;
		//6  CLRF1; writing a 1 to this bit clears the channel 1 (and channel 2?) fifo
		//5  USEF1;
		//4  POLA1;
		//3  SBIT1;
		//2  RPTL1;
		//1  MODE1;
		//0  PWMEN1;   
	volatile uint32_t STA;  // 0x00000004 //status register
		//13-31 reserved
		//9-12 STA1-4; indicates whether each channel is transmitting
		//8    BERR; Bus Error Flag. Write 1 to clear
		//4-7  GAPO1-4; Gap Occured Flag. Write 1 to clear
		//3    RERR1; Fifo Read Error Flag (attempt to read empty fifo). Write 1 to clear
		//2    WERR1; Fifo Write Error Flag (attempt to write to full fifo). Write 1 to clear
		//1    EMPT1; Reads as 1 if fifo is empty
		//0    FULL1; Reads as 1 if fifo is full
	volatile uint32_t DMAC; // 0x00000008 //DMA control register
		//31   ENAB; set to 1 to enable DMA
		//16-30 reserved
		//8-15 PANIC; DMA threshold for panic signal
		//0-7  DREQ;  DMA threshold for DREQ signal
	uint32_t _padding1;
	volatile uint32_t RNG1; // 0x00000010 //channel 1 range register (# output bits to use per sample)
		//0-31 PWM_RNGi; #of bits to modulate PWM. (eg if PWM_RNGi=1024, then each 32-bit sample sent through the FIFO will be modulated into 1024 bits.)
	volatile uint32_t DAT1; // 0x00000014 //channel 1 data
		//0-31 PWM_DATi; Stores the 32-bit data to be sent to the PWM controller ONLY WHEN USEFi=0 (FIFO is disabled)
	volatile uint32_t FIF1; // 0x00000018 //channel 1 fifo (write to this register to queue an output)
		//writing to this register will queue a sample into the fifo. If 2 channels are enabled, then each even sample (0-indexed) is sent to channel 1, and odd samples are sent to channel 2. WRITE-ONLY
	uint32_t _padding2;
	volatile uint32_t RNG2; // 0x00000020 //channel 2 range register
	volatile uint32_t DAT2; // 0x00000024 //channel 2 data
		//0-31 PWM_DATi; Stores the 32-bit data to be sent to the PWM controller ONLY WHEN USEFi=1 (FIFO is enabled). TODO: Typo???
};

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
		cbBaseVirt[i].DEST_AD = GPIO_BASE_BUS + GPSET0;
		cbBaseVirt[i].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
		cbBaseVirt[i].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);
		
		uint32_t* nextIdx = (i + 1) < numSrcBlocks ?
			(uint32_t*)(cbBasePhys + (i + 1)) :
			0;
		cbBaseVirt[i].NEXTCONBK = (uint32_t)nextIdx;
	}
	for (size_t i = 0; i < cbPageBytes; i += PAGE_SIZE) {
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
			GpioBufferFrame* item = srcArrayCached + i;
			item->gpclr[0] = 1 << outPin0;
			item->gpset[0] = 1 << outPin1;
		}
		else
		{
			GpioBufferFrame* item = srcArrayCached + i;
			item->gpclr[0] = 1 << outPin1;
			item->gpset[0] = 1 << outPin0;
		}
	}
	
	//configure PWM clock:
	//*(clockBaseMem + CM_PWMCTL / 4) = CM_PWMCTL_PASSWD | ((*(clockBaseMem + CM_PWMCTL / 4))&(~CM_PWMCTL_ENAB)); //disable clock
	//do {} while (*(clockBaseMem + CM_PWMCTL / 4) & CM_PWMCTL_BUSY); //wait for clock to deactivate
	clock.PwmDisable();

	//*(clockBaseMem + CM_PWMDIV / 4) = CM_PWMDIV_PASSWD | CM_PWMDIV_DIVI(CLOCK_DIV); //configure clock divider (running at 500MHz undivided)
	//*(clockBaseMem + CM_PWMCTL / 4) = CM_PWMCTL_PASSWD | CM_PWMCTL_SRC_PLLD; //source 500MHz base clock, no MASH.
	clock.PwmSetDivider(CLOCK_DIV);

	//*(clockBaseMem + CM_PWMCTL / 4) = CM_PWMCTL_PASSWD | CM_PWMCTL_SRC_PLLD | CM_PWMCTL_ENAB; //enable clock
	//do {} while (*(clockBaseMem + CM_PWMCTL / 4) & CM_PWMCTL_BUSY == 0); //wait for clock to activate
	clock.PwmEnable();
		
	pwm.Stop();	
	pwm.Start(BITS_PER_CLOCK);

	//allocate memory for the control blocks
	size_t cbPageBytes = numSrcBlocks * sizeof(struct DmaControlBlock) * 3; //3 cbs for each source block

	DmaMem_t* cbPage = dmaMemory.AllocDmaPage();

	//fill the control blocks:
	struct DmaControlBlock *cbArrCached = (struct DmaControlBlock*)cbPage->virtual_addr;
	struct DmaControlBlock *cbArr = (struct DmaControlBlock*)cbPage->bus_addr;

	printf("#dma blocks: %i, #src blocks: %i\n", numSrcBlocks * 2, numSrcBlocks);
	for (size_t i = 0; i < numSrcBlocks * 2; i += 2)
	{
		//pace DMA through PWM
		cbArrCached[i].TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_NO_WIDE_BURSTS | DMA_CB_TI_TDMODE;
		cbArrCached[i].SOURCE_AD = (uint32_t)(srcArray + (i / 2)); //The data written doesn't matter, but using the GPIO source will hopefully bring it into L2 for more deterministic timing of the next control block.
		cbArrCached[i].DEST_AD = PWM_BASE_BUS + PWM_FIF1; //write to the FIFO
		cbArrCached[i].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(1) | DMA_CB_TXFR_LEN_XLENGTH(4);
		cbArrCached[i].STRIDE = i / 2;
		cbArrCached[i].NEXTCONBK = (uint32_t)(cbArr + (i + 1));
		//copy buffer to GPIOs
		cbArrCached[i + 1].TI = DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC | DMA_CB_TI_NO_WIDE_BURSTS | DMA_CB_TI_TDMODE;
		cbArrCached[i + 1].SOURCE_AD = (uint32_t)(srcArray + (i / 2));
		cbArrCached[i + 1].DEST_AD = GPIO_BASE_BUS + GPSET0;
		cbArrCached[i + 1].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
		cbArrCached[i + 1].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);
		uint32_t* nextIdx = (i + 2) < (numSrcBlocks * 2) ?
			(uint32_t*) (cbArr + (i + 2)) :
			0; //last block should loop back to the first block
		cbArrCached[i + 1].NEXTCONBK = (uint32_t) nextIdx; //(uint32_t)physCbPage + ((void*)&cbArr[(i+2)%maxIdx] - virtCbPage);
	}
	for (size_t i = 0; i < cbPageBytes; i += PAGE_SIZE) {
		printf("virt cb[%i] -> virt: 0x%08x phys: (0x%08x)\n",
			i,
			cbPage->virtual_addr,
			cbPage->bus_addr);
	}
	//source: http://virtualfloppy.blogspot.com/2014/01/dma-support-at-last.html
	//cat /sys/module/dma/parameters/dmachans gives a bitmask of DMA channels that are not used by GPU. Results: ch 1, 3, 6, 7 are reserved.
	//dmesg | grep "DMA"; results: Ch 2 is used by SDHC host
	//ch 0 is known to be used for graphics acceleration
	//Thus, applications can use ch 4, 5, or the LITE channels @ 8 and beyond.
	//If using LITE channels, then we can't use the STRIDE feature, so that narrows it down to ch 4 and ch 5.
	int channel = 5;
	
	dma.EnableChannel(channel);

	//configure the DMA header to point to our control block:
	//volatile DmaChannel* dmaHeader = (struct DmaChannel*)(dmaBaseMem + DMACH(dmaCh) / 4); //must divide by 4, as dmaBaseMem is uint32_t*
	//printf("Previous DMA header:\n");
	//logDmaChannelHeader((DmaChannel*) dmaHeader);
	//abort any previous DMA:
	//dmaHeader->NEXTCONBK = 0; //NEXTCONBK is read-only.
	//dmaHeader->CS |= DMA_CS_ABORT; //make sure to disable dma first.
	//usleep(100); //give time for the abort command to be handled.

	//dmaHeader->CS = DMA_CS_RESET;
	//usleep(100);

	//writeBitmasked(&dmaHeader->CS, DMA_CS_END, DMA_CS_END); //clear the end flag
	//dmaHeader->DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR; // clear debug error flags
	dma.Stop(channel);

	//uint32_t firstAddr = cbPage->bus_addr;//virtToUncachedPhys(cbArrCached, pagemapfd);
	//printf("starting DMA @ CONBLK_AD=0x%08x\n", firstAddr);
	//dmaHeader->CONBLK_AD = firstAddr; //(uint32_t)physCbPage + ((void*)cbArr - virtCbPage); //we have to point it to the PHYSICAL address of the control block (cb1)
	//dmaHeader->CS = DMA_CS_PRIORITY(7) | DMA_CS_PANIC_PRIORITY(7) | DMA_CS_DISDEBUG; //high priority (max is 7)
	//dmaHeader->CS = DMA_CS_PRIORITY(7) | DMA_CS_PANIC_PRIORITY(7) | DMA_CS_DISDEBUG | DMA_CS_ACTIVE; //activate DMA. 

	uint32_t firstAddr = cbPage->bus_addr;
	dma.Start(5, firstAddr);

	volatile DmaChannel* chan = dma.Base->Chan + channel;
	while (chan->CS & DMA_CS_ACTIVE)
	{
		logDmaChannelHeader((DmaChannel*)chan);

	}
	
	dma.Stop(channel);
	printf("Exiting cleanly:\n");	

	// New code
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

	// Give scope enough time to see DMA as a
	// second event.  This will vary and may
	// or may not be needed for your scope.
	Delay::Milliseconds(500); // Also nice place for breakpoint :o

	// To double buffer the software will reserve
	// an I/O pin to toggle when it is going to switch
	// to the second buffer.  This way we can attach
	// an interrupt handler to it.
	gpio.SetIsr(outPin0,
		IntTrigger::Rising,
		OutPin0Isr,
		NULL);

	// configure DMA...
	size_t numSrcBlocks = getpagesize() / sizeof(struct DmaControlBlock);

	DmaMemory dmaMemory;
	DmaMem_t* memoryFramePage0[128];

	for (size_t i = 0; i < numSrcBlocks; i++)
	{

		memoryFramePage0[i] = dmaMemory.AllocDmaPage();
		printf("gpioFramePage%d: virt %p phys %p\n",
			i,
			memoryFramePage0[i]->virtual_addr,
			memoryFramePage0[i]->bus_addr);
	}


	gpio.ClearIsr(outPin0);

	for (size_t i = 0; i < numSrcBlocks; i++)
	{
		dmaMemory.FreeDmaPage(memoryFramePage0[i]);
	}
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
		cbBase0Virt[i].DEST_AD = GPIO_BASE_BUS + GPSET0;
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
		cbBase1Virt[i].DEST_AD = GPIO_BASE_BUS + GPSET0;
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

	for (size_t i = 0; i < cbPageBytes; i += PAGE_SIZE) {
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