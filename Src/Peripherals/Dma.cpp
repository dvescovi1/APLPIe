/*
 * Dma.cpp:
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
#include <unistd.h>

#include "../Headers/Delay.h"
#include "../Headers/Dma.h"
#include "../Headers/DmaMemory.h"

#include "../Headers/ScreenLog.h"
#include "../Headers/hw-addresses.h"

//flags used in the DmaChannelHeader struct:
#define DMA_CS_RESET (1<<31)
#define DMA_CS_ABORT (1<<30)
#define DMA_CS_DISDEBUG (1<<28) //DMA will not stop when debug signal is asserted
#define DMA_CS_ACTIVE (1<<0)
#define DMA_CS_END (1<<1)

#define DMA_DEBUG_READ_ERROR (1<<2)
#define DMA_DEBUG_FIFO_ERROR (1<<1)
#define DMA_DEBUG_READ_LAST_NOT_SET_ERROR (1<<0)

#define DMA_CS_PRIORITY(x) ((x)&0xf << 16) //higher priority DMA transfers are serviced first, it would appear
#define DMA_CS_PRIORITY_MAX DMA_CS_PRIORITY(7)
#define DMA_CS_PANIC_PRIORITY(x) ((x)&0xf << 20)

Dma::Dma(const char* name) :
	PeripheralTemplate<DmaRegisters>(name, DMA_BASE)
{	
}

void Dma::SysInit()
{
	PeripheralTemplate<DmaRegisters>::SysInit();

	// Map the 16th channel
	Peripheral::Map(DMA15_BASE,
		sizeof(DmaChannel),
		_dmaChannel15.info);
	DBG("DMA channel [15]: %p", _dmaChannel15.info.MappedAddress);
}

void Dma::SysUninit()
{
	PeripheralTemplate<DmaRegisters>::Unmap();
	Peripheral::Unmap(_dmaChannel15.info);
}

void Dma::EnableChannel(int channel)
{
	volatile uint32_t* enable = &Base->ENABLE;
	*enable |= 1 << channel;
}

void Dma::Start(int channel, volatile uint32_t controlBlock)
{
	volatile DmaChannel* chan = &(Base->Chan[channel]);
	
	chan->CONBLK_AD = controlBlock;
	chan->CS |= DMA_CS_PRIORITY(7) |
		DMA_CS_PANIC_PRIORITY(7) |
		DMA_CS_DISDEBUG |
		DMA_CS_ACTIVE;
	do {} while ((chan->CS & DMA_CS_ACTIVE) == 0);
}

void Dma::Stop(int channel)
{
	volatile DmaChannel* chan = &(Base->Chan[channel]);

	chan->CS |= DMA_CS_ABORT;
	Delay::Microseconds(100);
	chan->CS |= DMA_CS_RESET;
	do {} while ((chan->CS & DMA_CS_ACTIVE) == DMA_CS_ACTIVE);
	
	chan->CS |= DMA_CS_END;
	
	chan->DEBUG |= DMA_DEBUG_READ_ERROR |
		DMA_DEBUG_FIFO_ERROR |
		DMA_DEBUG_READ_LAST_NOT_SET_ERROR;
}


