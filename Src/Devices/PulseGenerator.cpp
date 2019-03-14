/*
 * PulseGenerator.cpp:
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
#include <stddef.h>
#include <unistd.h>

#include "../Headers/PulseGenerator.h"

struct GpioData
{
	//custom structure used for storing the GPIO buffer.
	//These BufferFrame's are DMA'd into the GPIO memory, potentially using the DmaEngine's Stride facility
	uint32_t gpset[2];
	uint32_t gpclr[2];

	void Set(uint32_t pins, PinState state)
	{
		switch (state)
		{
		case PinState::High:
		{
			gpset[0] = pins;
			gpclr[0] = 0;
		}
		break;
		case PinState::Low:
		{
			gpclr[0] = pins;
			gpset[0] = 0;
		}
		break;
		case PinState::Unknown:
		default:
			break;
		}
	}
};

struct PwmData
{	
	uint32_t clocks;
	uint32_t fifo1;

	void Set(uint32_t duration)
	{
		clocks = duration;
	}
};

struct DmaTransfer
{
	union
	{
		GpioData Gpio;
		PwmData Pwm;
	};
};

void PulseGenerator::SyncPinIsr(void* arg)
{
	// Here you reprogram the buffer that is not currently being executed
	// and / or put the next address of the last control block to zero
	// when you want the Dma to stop.
	PulseGenerator* pulseGenerator = (PulseGenerator*)arg;
//	pulseGenerator->NextChunk();
}

PulseGenerator::PulseGenerator(Gpio& gpio, Dma& dma, Pwm& pwm, Clock& clock, uint32_t bufferSyncPin, uint8_t numBufferPages) :
	Device("PulseGenerator"),
	_gpio(gpio),
	_dma(dma),
	_pwm(pwm),
	_clock(clock),
	_bufferSyncPin(bufferSyncPin),
	_numTransferPages(numBufferPages)	
{
	int pageSize = getpagesize();

	_numTransferFramesPerPage = pageSize / sizeof(DmaTransfer);
	_numControlBlocksPerPage = pageSize / sizeof(DmaControlBlock);
}

void PulseGenerator::SysInit(void)
{
	if (_buffer0Pages.size() > 0)
	{
		DBG("Warning: Dma memory allready allocated.");
	}
	if (_buffer1Pages.size() > 0)
	{
		DBG("Warning: Dma memory allready allocated.");
	}

	_gpio.SetPinMode(_bufferSyncPin, PinMode::Output);
	_gpio.WritePin(_bufferSyncPin, PinState::Low);

	// To double buffer the software will reserve
	// an I/O pin to toggle when it is going to switch
	// to the second buffer.  This way we can attach
	// an interrupt handler to it.
	_gpio.SetIsr(_bufferSyncPin,
		IntTrigger::Both,
		SyncPinIsr,
		this);

	DmaMemory dmaMemory;
	
	for (int i = 0; i < _numTransferPages; i++)
	{
		DmaMem_t* buffer0Page = dmaMemory.AllocDmaPage();
		_buffer0Pages.emplace_back(buffer0Page);

		DmaMem_t* buffer1Page = dmaMemory.AllocDmaPage();
		_buffer1Pages.emplace_back(buffer1Page);
	}

	uint32_t requiredControlPages = _numTransferPages *
		(_numTransferFramesPerPage / _numControlBlocksPerPage);
		
	for (size_t i = 0; i < requiredControlPages; i++)
	{
		DmaMem_t* cb0Page = dmaMemory.AllocDmaPage();
		_controlBlock0Pages.emplace_back(cb0Page);

		DmaMem_t* cb1Page = dmaMemory.AllocDmaPage();
		_controlBlock1Pages.emplace_back(cb1Page);
	}
}

void PulseGenerator::SysUninit(void)
{
	_gpio.ClearIsr(_bufferSyncPin);

	DmaMemory dmaMemory;

	for (size_t i = 0; i < _numTransferPages; i++)
	{
		dmaMemory.FreeDmaPage(_buffer0Pages[i]);
		dmaMemory.FreeDmaPage(_buffer1Pages[i]);
		
	}
	_buffer0Pages.clear();
	_buffer1Pages.clear();

	for (size_t i = 0; i < _controlBlock0Pages.size(); i++)
	{
		dmaMemory.FreeDmaPage(_controlBlock0Pages[i]);
		dmaMemory.FreeDmaPage(_controlBlock1Pages[i]);
	}
	_controlBlock0Pages.clear();
	_controlBlock1Pages.clear();
}

void PulseGenerator::Add(PulseTrain& pulseTrain)
{
	_pulseTracks.emplace_back(pulseTrain);
}

void PulseGenerator::Start()
{
	_clockCycle = 0;
	uint32_t numControlBlocks = ConfigureBuffer0(_clockCycle);

	_clock.PwmDisable();
	_clock.PwmSetDivider(CLOCK_DIV);
	_clock.PwmEnable();

	_pwm.Stop();
	ConfigureControlBlocks0(numControlBlocks);

	// Although undocumented it appears from empirical data
	// that changing RNG1 needs a word to be clocked in and
	// only takes effect on the next element.  So we set
	// RNG1 to the first element duration now.
	_pwm.Start(_pulseTracks[0].Timing[0].Duration);

	int channel = 5;

	_dma.EnableChannel(channel);
	_dma.Stop(channel);

	volatile uint32_t* fif1 = &_pwm.Base->FIF1;
	*fif1 = 1;
	
	uint32_t firstAddr = _controlBlock0Pages[0]->bus_addr;
	_dma.Start(channel, firstAddr);

	// TODO: remove or move outside of test.
	volatile DmaChannel* chan = _dma.Base->Chan + channel;
	do {} while (chan->CS & DMA_CS_ACTIVE);
}

void PulseGenerator::WriteSyncPinState(PinState state)
{
	_gpio.WritePin(_bufferSyncPin, state);
}

uint32_t PulseGenerator::ConfigureBuffer0(uint32_t startingClock)
{
	DmaTransfer* transferBase0 = (DmaTransfer*)_buffer0Pages[0]->virtual_addr;
	uint32_t result = 0;
	size_t trackLength = _pulseTracks[0].Timing.size();

	// i counts the pulse info which has both state and duration.
	// j tracks the number of transfer frames i.e. one for state (GPIO)
	// and one for duration (PWM)
	for (size_t i = 0;
		(i < trackLength) && (result < (_numTransferFramesPerPage - 1));
		i++)
	{

		// Although undocumented it appears from empirical data
		// that changing RNG1 needs a word to be clocked in and
		// only takes effect on the next element.  So we set
		// RNG1 to the next element (i + 1) duration now.  The
		// last element will take the first duration to keep
		// symmetry in the data structure and also in case repeat
		// is set to true.
		//
		// See the start function also this is where the first
		// duration is set to _pulseTracks[0].Timing[0].Duration.
		size_t nextDurationIndex = (i + 1) % trackLength;
		DmaTransfer* transferItem1 = transferBase0 + result;
		PwmData* item1 = (PwmData*)transferItem1;
		item1->Set(_pulseTracks[0].Timing[nextDurationIndex].Duration);
		result++;

		DmaTransfer* transferItem0 = transferBase0 + result;
		GpioData* item = (GpioData*)transferItem0;
		item->Set(_pulseTracks[0].Pin, _pulseTracks[0].Timing[i].State);
		result++;
	}

	// Now adding the sync pin (GPIO) state change
	// which can generate an ISR depending on starting
	// state.
	DmaTransfer* isrItem0 = transferBase0 + result;

	GpioData* item = (GpioData*)isrItem0;
	item->Set(1 << _bufferSyncPin, PinState::Low);

	/*for (int i = 0; i < result; i++)
	{
		DmaTransfer* transferItem0 = transferBase0 + i;
		GpioData* gpioItem = (GpioData*)transferItem0;
		i++;
		DmaTransfer* transferItem1 = transferBase0 + i;
		PwmData* pwmItem = (PwmData*)transferItem1;
		DBG("Transfer item %i: State: %d, Duration %d",
			i / 2,
			gpioItem->gpset[0],
			pwmItem->clocks);
	}*/

	return result;
}

uint32_t PulseGenerator::ConfigureBuffer1(uint32_t startingClock)
{

}

void PulseGenerator::ConfigureControlBlocks0(uint32_t numControlBlocks)
{
	volatile DmaControlBlock* cb0;
	DmaControlBlock* cbPhys0;
	DmaTransfer* dmaTransferPhys0 = (DmaTransfer*)_buffer0Pages[0]->bus_addr;

	uint32_t gpioAddress = GPIO_BASE_BUS + offsetof(GpioRegisters, GPSET0);
	uint32_t pwmAddress = PWM_BASE_BUS  + offsetof(PwmRegisters, RNG1);
		
	size_t cbIndex;
	for (size_t i = 0, j = 0; i < numControlBlocks; i++)
	{
		cbIndex = i % _numControlBlocksPerPage;
		if (cbIndex == 0)
		{
			cb0 = (DmaControlBlock*)_controlBlock0Pages[j]->virtual_addr;
			cbPhys0 = (DmaControlBlock*)_controlBlock0Pages[j]->bus_addr;
			j++;
		}

		//pace DMA through PWM
		cb0[cbIndex].TI = DMA_CB_TI_PERMAP_PWM |
			DMA_CB_TI_DEST_DREQ |
			DMA_CB_TI_SRC_INC |
			DMA_CB_TI_DEST_INC |
			DMA_CB_TI_NO_WIDE_BURSTS |
			DMA_CB_TI_TDMODE;
		cb0[cbIndex].SOURCE_AD = (uint32_t)(dmaTransferPhys0 + i); //The data written doesn't matter, but using the GPIO source will hopefully bring it into L2 for more deterministic timing of the next control block.
		cb0[cbIndex].DEST_AD = pwmAddress; //write to the FIFO
		cb0[cbIndex].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(4);
		cb0[cbIndex].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);
		if ((cbIndex + 1) % _numControlBlocksPerPage == 0)
		{
			cbPhys0 = (DmaControlBlock*)_controlBlock0Pages[j]->bus_addr;
			cb0[cbIndex].NEXTCONBK = (uint32_t)(cbPhys0);
		}
		else
		{
			cb0[cbIndex].NEXTCONBK = (uint32_t)(cbPhys0 + (cbIndex + 1));
		}

		i++;
		cbIndex = i % _numControlBlocksPerPage;
		if (cbIndex == 0)
		{
			cb0 = (DmaControlBlock*)_controlBlock0Pages[j]->virtual_addr;
		}	
		
		// setup the gpio pin states.
		cb0[cbIndex].TI = DMA_CB_TI_SRC_INC |
			DMA_CB_TI_DEST_INC |
			DMA_CB_TI_NO_WIDE_BURSTS |
			DMA_CB_TI_TDMODE;
		cb0[cbIndex].SOURCE_AD = (uint32_t)(dmaTransferPhys0 + i);
		cb0[cbIndex].DEST_AD = gpioAddress;
		cb0[cbIndex].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
		cb0[cbIndex].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);		

		if ((cbIndex + 1) % _numControlBlocksPerPage == 0)
		{
			cbPhys0 = (DmaControlBlock*)_controlBlock0Pages[j]->bus_addr;
			cb0[cbIndex].NEXTCONBK = (uint32_t)(cbPhys0);
		}
		else
		{
			cb0[cbIndex].NEXTCONBK = (uint32_t)(cbPhys0 + (cbIndex + 1));
		}
	}

	// This tiggers on the the buffer sync pin and may
	// (or may not depedning on start state) generate
	// on interrupt.
	cbIndex++;
	cb0[cbIndex].TI = DMA_CB_TI_SRC_INC |
		DMA_CB_TI_DEST_INC |
		DMA_CB_TI_NO_WIDE_BURSTS |
		DMA_CB_TI_TDMODE;
	cb0[cbIndex].SOURCE_AD = (uint32_t)(dmaTransferPhys0 + numControlBlocks);
	cb0[cbIndex].DEST_AD = GPIO_BASE_BUS + offsetof(GpioRegisters, GPSET0);
	cb0[cbIndex].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
	cb0[cbIndex].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);
	cb0[cbIndex].NEXTCONBK = (uint32_t)0;
}

void PulseGenerator::ConfigureControlBlocks1(uint32_t numControlBlocks)
{
}