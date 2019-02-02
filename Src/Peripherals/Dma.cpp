#include <unistd.h>

#include "../Headers/Dma.h"
#include "../Headers/DmaMemory.h"

#include "../Headers/ScreenLog.h"
#include "../Headers/hw-addresses.h"

Dma::Dma(const char* name) :
	Peripheral(name)
{	
}

void Dma::SysInit()
{
	int memSize = sizeof(DmaRegisters);		
	int pages = memSize / _pageSize + ((memSize % _pageSize) > 0 ? 1 : 0);
	Map(DMA_BASE, pages * _pageSize, _dma.info);
	DBG("DMA channels [0-14]: %p", _dma.info.MappedAddress);

	// Mapp the 16th channel
	Map(DMA15_BASE, sizeof(DmaChannel), _dmaChannel15.info);
	DBG("DMA channels [15]: %p", _dmaChannel15.info.MappedAddress);
}

void Dma::SysUninit()
{
	Unmap(_dma.info);	
	Unmap(_dmaChannel15.info);
}

//flags used in the DmaControlBlock struct:
#define DMA_CB_TI_DEST_INC (1<<4)
#define DMA_CB_TI_SRC_INC (1<<8)

//flags used in the DmaChannelHeader struct:
#define DMA_CS_RESET (1<<31)
#define DMA_CS_ACTIVE (1<<0)

#define DMA_DEBUG_READ_ERROR (1<<2)
#define DMA_DEBUG_FIFO_ERROR (1<<1)
#define DMA_DEBUG_READ_LAST_NOT_SET_ERROR (1<<0)


static char testCount = 0;

void Dma::MemoryTestExample()
{
	DmaMemory dmaMemory;

	// This may be used by the sys PWM but.. we are having our own custom use
	// case for the PWM so as long as no other software is using the built in
	// PWM driver we should be OK.
	int dmaChan = 5;

	DmaMem_t* dmaSource = dmaMemory.AllocDmaPage();
	DmaMem_t* dmaDest = dmaMemory.AllocDmaPage();

	//write a few bytes to the source page:
	volatile char* srcArray = (volatile char*) dmaSource->virtual_addr;
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
	srcArray[12] = testCount++;
	srcArray[13] = 0; //null terminator used for printf call.

	//allocate 1 page for the control blocks	
	DmaMem_t* dmaControl = dmaMemory.AllocDmaPage();

	//dedicate the first 8 words of this page to holding the cb.
	DmaControlBlock *cb1 = (DmaControlBlock*)dmaControl->virtual_addr;

	//fill the control block:
	cb1->TI = DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC; //after each byte copied, we want to increment the source and destination address of the copy, otherwise we'll be copying to the same address.
//	cb1->SOURCE_AD = (uint32_t)physSrcPage; //set source and destination DMA address
	cb1->SOURCE_AD = (uint32_t) dmaSource->bus_addr; //set source and destination DMA address
//	cb1->DEST_AD = (uint32_t)physDestPage;
	cb1->DEST_AD = (uint32_t)dmaDest->bus_addr;
	cb1->TXFR_LEN = 14; //transfer 12 bytes
	cb1->STRIDE = 0; //no 2D stride
	cb1->NEXTCONBK = 0; //no next control block

	printf("destination was initially: '%s'\n", (volatile char*)dmaDest->virtual_addr);
	_dma.Base->ENABLE |= 1 << dmaChan;

	// make sure to disable dma first.
	_dma.Base->Chan[dmaChan].CS = DMA_CS_RESET;

	// Wait until the DMA is disabled.
	do {} while ((_dma.Base->Chan[dmaChan].CS & 0x01) > 0);

	// Start the transfer
	_dma.Base->Chan[dmaChan].DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR; // clear debug error flags
	_dma.Base->Chan[dmaChan].CONBLK_AD = (uint32_t)dmaControl->bus_addr; //we have to point it to the PHYSICAL address of the control block (cb1)
	_dma.Base->Chan[dmaChan].CS = DMA_CS_ACTIVE; //set active bit, but everything else is 0.

	// WAit until the transfer is complete.
	do {} while ((_dma.Base->Chan[dmaChan].CS & 0x01) > 0);

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

