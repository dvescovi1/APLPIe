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

#include "../Headers/Dma.h"
#include "../Headers/DmaMemory.h"

#include "../Headers/ScreenLog.h"
#include "../Headers/hw-addresses.h"

Dma::Dma(const char* name) :
	PeripheralTemplate<DmaRegisters, DMA_BASE>(name)
{	
}

void Dma::SysInit()
{
	PeripheralTemplate<DmaRegisters, DMA_BASE>::SysInit();

	// Map the 16th channel
	Peripheral::Map(DMA15_BASE,
		sizeof(DmaChannel),
		_dmaChannel15.info);
	DBG("DMA channel [15]: %p", _dmaChannel15.info.MappedAddress);
}

void Dma::SysUninit()
{
	PeripheralTemplate<DmaRegisters, DMA_BASE>::Unmap();
	Peripheral::Unmap(_dmaChannel15.info);
}