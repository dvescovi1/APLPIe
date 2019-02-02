#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "../Headers/DmaMemory.h"

/* ======================================================================= */

/*
https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
*/

// Use of this device may have side effects.  This is the GPU.
// but this driver can allocate memeory suitable for DMA in that
// it is unchached.  TODO: change this to a similar function that
// is designed for hte DMA peripheral... when / if it ever exists.
//
// One goal here is to only use already existing RPi kernel resources.
//
// Also since we are 'stealing' memory from the GPU care is taken to
// minimize the number of pages to as few as possible.
#define MB_DEV1 "/dev/vcio"
#define MB_DEV_MAJOR 100
#define MB_IOCTL _IOWR(MB_DEV_MAJOR, 0, char *)
#define BUS_TO_PHYS(x) ((x)&~0xC0000000)
#define MB_END_TAG 0
#define MB_PROCESS_REQUEST 0

#define MB_ALLOCATE_MEMORY_TAG 0x3000C
#define MB_LOCK_MEMORY_TAG     0x3000D
#define MB_UNLOCK_MEMORY_TAG   0x3000E
#define MB_RELEASE_MEMORY_TAG  0x3000F

DmaMemory::DmaMemory()
{

}

DmaMemory::~DmaMemory()
{
	if (_fdMbox != -1)
	{
		MbClose();
	}
	if (_fdMem == -1)
	{
		close(_fdMem);
	}
}

void DmaMemory::MbOpen()
{
	if (_fdMbox == INVALID_HANDLE)
	{
		_fdMbox = open(MB_DEV1, 0);
	}
}

void DmaMemory::MbClose()
{
	if (_fdMbox != INVALID_HANDLE)
	{
		close(_fdMbox);
	}
}

int DmaMemory::MbProperty(void *buf)
{
	return ioctl(_fdMbox,
		MB_IOCTL,
		buf);
}

unsigned DmaMemory::MbAllocateMemory(unsigned size, unsigned align, unsigned flags)
{
	int i = 1;
	unsigned p[32];

	p[i++] = MB_PROCESS_REQUEST;
	p[i++] = MB_ALLOCATE_MEMORY_TAG;
	p[i++] = 12;
	p[i++] = 12;
	p[i++] = size;
	p[i++] = align;
	p[i++] = flags;
	p[i++] = MB_END_TAG;
	p[0] = i * sizeof(*p);

	MbProperty(p);

	return p[5];
}

unsigned DmaMemory::MbLockMemory(unsigned handle)
{
	int i = 1;
	unsigned p[32];

	p[i++] = MB_PROCESS_REQUEST;
	p[i++] = MB_LOCK_MEMORY_TAG;
	p[i++] = 4;
	p[i++] = 4;
	p[i++] = handle;
	p[i++] = MB_END_TAG;
	p[0] = i * sizeof(*p);

	MbProperty(p);

	return p[5];
}

unsigned DmaMemory::MbUnlockMemory(unsigned handle)
{
	int i = 1;
	unsigned p[32];

	p[i++] = MB_PROCESS_REQUEST;
	p[i++] = MB_UNLOCK_MEMORY_TAG;
	p[i++] = 4;
	p[i++] = 4;
	p[i++] = handle;
	p[i++] = MB_END_TAG;
	p[0] = i * sizeof(*p);

	MbProperty(p);

	return p[5];
}

unsigned DmaMemory::MbReleaseMemory(unsigned handle)
{
	int i = 1;
	unsigned p[32];

	p[i++] = MB_PROCESS_REQUEST;
	p[i++] = MB_RELEASE_MEMORY_TAG;
	p[i++] = 4;
	p[i++] = 4;
	p[i++] = handle;
	p[i++] = MB_END_TAG;
	p[0] = i * sizeof(*p);

	MbProperty(p);

	return p[5];
}

void* DmaMemory::MbMapMem(unsigned base, unsigned size)
{
	void *mem = MAP_FAILED;

	mem = mmap(0,
		size,
		PROT_READ | PROT_WRITE,
		MAP_SHARED,
		_fdMem,
		base);

	return mem;
}

int DmaMemory::MbUnmapMem(void *addr, unsigned size)
{
	/* 0 okay, -1 fail */
	return munmap(addr, size);
}

void DmaMemory::FreeDmaPage(DmaMem_t *DMAMemP)
{
	if (DMAMemP->handle)
	{
		MbUnmapMem(DMAMemP->virtual_addr, DMAMemP->size);
		MbUnlockMemory(DMAMemP->handle);
		MbReleaseMemory(DMAMemP->handle);
		DMAMemP->handle = 0;
	}
}

 int DmaMemory::MbDMAAlloc(DmaMem_t *DMAMemP, unsigned size, uint32_t pi_mem_flag)
{
	DMAMemP->size = size;

	DMAMemP->handle = MbAllocateMemory(size,
		getpagesize(),
		pi_mem_flag);

	if (DMAMemP->handle)
	{
		DMAMemP->bus_addr = MbLockMemory(DMAMemP->handle);

		DMAMemP->virtual_addr = (uintptr_t*)MbMapMem(BUS_TO_PHYS(DMAMemP->bus_addr), size);
		return 1;
	}
	return 0;
}

static volatile uint32_t pi_mem_flag = 0x0C;

int  DmaMemory::InitMboxBlock(DmaMem_t* dmaMbox)
{
	int pageSize = getpagesize();
	int result = MbDMAAlloc(dmaMbox,
		pageSize,
		pi_mem_flag);
	return result;
}

DmaMem_t* DmaMemory::AllocDmaPage()
{
	DmaMem_t* dmaPage = (DmaMem_t*)mmap(0,
		sizeof(DmaMem_t),
		PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_LOCKED,
		-1, 0);
	
	MbOpen();

	if (_fdMem == -1)
	{

		// TODO: waiting for "/dev/dma" with channels
		// available to user.
		_fdMem = open("/dev/mem", O_RDWR | O_SYNC);
	}

	InitMboxBlock(dmaPage);
	return dmaPage;
}