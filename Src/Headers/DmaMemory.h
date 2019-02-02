#pragma once

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
