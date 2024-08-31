/*
 * Dma.h:
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
* Some ideas and code taken from:
*
* https://github.com/Wallacoloo/Raspberry-Pi-DMA-Example : DMA Raspberry Pi Examples
*Author : Colin Wallace
*/
#ifndef DMA_H
#define DMA_H


#include <stdint.h>

#include "../Headers/Peripheral.h"
#include "../Headers/hw-addresses.h"

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

struct DmaChannel {
	uint32_t CS; //Control and Status
		//31    RESET; set to 1 to reset DMA
		//30    ABORT; set to 1 to abort current DMA control block (next one will be loaded & continue)
		//29    DISDEBUG; set to 1 and DMA won't be paused when debug signal is sent
		//28    WAIT_FOR_OUTSTANDING_WRITES; set to 1 and DMA will wait until peripheral says all writes have gone through before loading next CB
		//24-27 reserved
		//20-23 PANIC_PRIORITY; 0 is lowest priority
		//16-19 PRIORITY; bus scheduling priority. 0 is lowest
		//9-15  reserved
		//8     ERROR; read as 1 when error is encountered. error can be found in DEBUG register.
		//7     reserved
		//6     WAITING_FOR_OUTSTANDING_WRITES; read as 1 when waiting for outstanding writes
		//5     DREQ_STOPS_DMA; read as 1 if DREQ is currently preventing DMA
		//4     PAUSED; read as 1 if DMA is paused
		//3     DREQ; copy of the data request signal from the peripheral, if DREQ is enabled. reads as 1 if data is being requested, else 0
		//2     INT; set when current CB ends and its INTEN=1. Write a 1 to this register to clear it
		//1     END; set when the transfer defined by current CB is complete. Write 1 to clear.
		//0     ACTIVE; write 1 to activate DMA (load the CB before hand)
	uint32_t CONBLK_AD; //Control Block Address
	uint32_t TI; //transfer information; see DmaControlBlock.TI for description
	uint32_t SOURCE_AD; //Source address
	uint32_t DEST_AD; //Destination address
	uint32_t TXFR_LEN; //transfer length.
	uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1
	uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
	uint32_t DEBUG; //controls debug settings
	uint8_t padding[220]; // makes this 256 bytes aligned
};

struct DmaControlBlock {
	uint32_t TI; //transfer information
		//31:27 unused
		//26    NO_WIDE_BURSTS
		//21:25 WAITS; number of cycles to wait between each DMA read/write operation
		//16:20 PERMAP; peripheral number to be used for DREQ signal (pacing). set to 0 for unpaced DMA.
		//12:15 BURST_LENGTH
		//11    SRC_IGNORE; set to 1 to not perform reads. Used to manually fill caches
		//10    SRC_DREQ; set to 1 to have the DREQ from PERMAP gate requests.
		//9     SRC_WIDTH; set to 1 for 128-bit moves, 0 for 32-bit moves
		//8     SRC_INC;   set to 1 to automatically increment the source address after each read (you'll want this if you're copying a range of memory)
		//7     DEST_IGNORE; set to 1 to not perform writes.
		//6     DEST_DREG; set to 1 to have the DREQ from PERMAP gate *writes*
		//5     DEST_WIDTH; set to 1 for 128-bit moves, 0 for 32-bit moves
		//4     DEST_INC;   set to 1 to automatically increment the destination address after each read (Tyou'll want this if you're copying a range of memory)
		//3     WAIT_RESP; make DMA wait for a response from the peripheral during each write. Ensures multiple writes don't get stacked in the pipeline
		//2     unused (0)
		//1     TDMODE; set to 1 to enable 2D mode
		//0     INTEN;  set to 1 to generate an interrupt upon completion
	uint32_t SOURCE_AD; //Source address
	uint32_t DEST_AD; //Destination address
	uint32_t TXFR_LEN; //transfer length.
	uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1
	uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
	uint32_t _reserved[2];
};

typedef struct
{
	DmaChannel Chan[15];
	uint32_t Reserved0[56];
	uint32_t INT_STATUS; // Interrupt status of each DMA channel
	uint32_t Reserved1[3];
	uint32_t ENABLE; // Global enable bits for each DMA channe
} DmaRegisters;

typedef struct
{
	union
	{
		PeripheralInfo info;
		volatile DmaRegisters* Base;
	};
} DmaInfo;


class Dma : public PeripheralTemplate<DmaRegisters>
{
private:	
	DmaInfo _dmaChannel15;

public:	
	Dma(const char* name);

	void virtual SysInit();
	void virtual SysUninit();

	void EnableChannel(int channel);
	void Start(int channel, uint32_t controlBlock);
	void Stop(int channel);
};

#endif /* DMA_H */
