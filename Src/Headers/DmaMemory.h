/*
 * DmaMemory.h:
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
#ifndef DMAMEMORY_H
#define DMAMEMORY_H


#include <stdint.h>

#define INVALID_HANDLE - 1

typedef struct
{
	unsigned  handle;
	uintptr_t bus_addr;
	uintptr_t *virtual_addr;
	unsigned  size;
} DmaMem_t;

class DmaMemory
{
private:
	int _fdMbox = INVALID_HANDLE;
	int _fdMem = INVALID_HANDLE;

	void MbOpen();
	void MbClose();
	int MbProperty(void *buf);
	unsigned MbAllocateMemory(unsigned size, unsigned align, unsigned flags);
	unsigned MbLockMemory(unsigned handle);
	unsigned MbUnlockMemory(unsigned handle);
	unsigned MbReleaseMemory(unsigned handle);
	void*MbMapMem(unsigned base, unsigned size);
	int MbUnmapMem(void *addr, unsigned size);
	int MbDMAAlloc(DmaMem_t *DMAMemP, unsigned size, uint32_t pi_mem_flag);
	int InitMboxBlock(DmaMem_t* dmaMbox);

public:
	DmaMemory();
	virtual ~DmaMemory();
	DmaMem_t* AllocDmaPage();
	void FreeDmaPage(DmaMem_t *DMAMemP);
};

#endif /* DMAMEMORY_H */