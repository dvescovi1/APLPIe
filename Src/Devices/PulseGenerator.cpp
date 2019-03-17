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
	
	pulseGenerator->NextChunk();
}

PulseGenerator::PulseGenerator(Gpio& gpio, Dma& dma, Pwm& pwm, Clock& clock, uint32_t bufferSyncPin, uint8_t numBufferPages) :
	Device("PulseGenerator"),
	_gpio(gpio),
	_dma(dma),
	_pwm(pwm),
	_clock(clock),
	_bufferSyncPin(bufferSyncPin),
	_numTransferPages(numBufferPages),
	_running(false)
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
	_currentPulseSegment = 0;
	uint32_t numPulseSegments0 = ConfigureBuffer0(_currentPulseSegment);

	if (numPulseSegments0 == 0)
	{
		DBG("No control blocks found");
		return;
	}

	uint32_t numPulseSegments1 = ConfigureBuffer1(numPulseSegments0);

	_clock.PwmDisable();
	_clock.PwmSetDivider(CLOCK_DIV);
	_clock.PwmEnable();

	_pwm.Stop();

	uint32_t numControlBlocks0 = (numPulseSegments0 - _currentPulseSegment) * CONTROL_BLOCK_PER_PULSE_SEGMENT;
	ConfigureControlBlocks0(numControlBlocks0);

	uint32_t numControlBlocks1 = (numPulseSegments1 - numPulseSegments0) * CONTROL_BLOCK_PER_PULSE_SEGMENT;
	ConfigureControlBlocks1(numControlBlocks1);

	_currentPulseSegment = numPulseSegments1;

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
	_running = true;
	_dma.Start(channel, firstAddr);

	// TODO: remove or move outside of test.
	volatile DmaChannel* chan = _dma.Base->Chan + channel;
	do {} while (chan->CS & DMA_CS_ACTIVE);
}

void PulseGenerator::WriteSyncPinState(PinState state)
{
	_gpio.WritePin(_bufferSyncPin, state);
}

uint32_t PulseGenerator::ConfigureBuffer0(uint32_t startingPulseSegment)
{
	DmaTransfer* transferBase0 = (DmaTransfer*)_buffer0Pages[0]->virtual_addr;
	uint32_t currentPulseSegment = startingPulseSegment;
	size_t trackLength = _pulseTracks[0].Timing.size();

	if (trackLength == 0 ||
		currentPulseSegment >= trackLength)
	{
		DBG("Pulse tracks have no pulse segments to program.");
		return 0;
	}

	// i tracks the index into the bufferPage (0-254)
	// currentPulseSegment tracks the number of track elements
	// that have been mapped into a DMA buffer
	// and one for duration (PWM)
	//
	// using minus two so that there is no need to staddle a gpio and
	// its timing across a page boundary.
	size_t i = 0;
	
	while(i < (_numTransferFramesPerPage - 2) &&
		currentPulseSegment < trackLength)
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
		size_t nextDurationIndex = (currentPulseSegment + 1) % trackLength;
		DmaTransfer* transferItem1 = transferBase0 + i;
		PwmData* item1 = (PwmData*)transferItem1;
		item1->Set(_pulseTracks[0].Timing[nextDurationIndex].Duration);
		i++;

		DmaTransfer* transferItem0 = transferBase0 + i;
		GpioData* item = (GpioData*)transferItem0;
		item->Set(_pulseTracks[0].Pin, _pulseTracks[0].Timing[currentPulseSegment].State);
		i++;

		currentPulseSegment++;
	}

	// Now adding the sync pin (GPIO) state change
	// which can generate an ISR depending on starting
	// state.
	DmaTransfer* isrItem0 = transferBase0 + i;

	GpioData* item = (GpioData*)isrItem0;
	item->Set(1 << _bufferSyncPin, PinState::Low);

	return currentPulseSegment;
}

uint32_t PulseGenerator::ConfigureBuffer1(uint32_t startingPulseSegment)
{
	DmaTransfer* transferBase0 = (DmaTransfer*)_buffer1Pages[0]->virtual_addr;

	uint32_t currentPulseSegment = startingPulseSegment;
	size_t trackLength = _pulseTracks[0].Timing.size();

	if (trackLength == 0 ||
		currentPulseSegment >= trackLength)
	{
		DBG("Pulse tracks have no pulse segments to program.");
		return 0;
	}

	// i tracks the index into the bufferPage (0-254)
	// currentPulseSegment tracks the number of track elements
	// that have been mapped into a DMA buffer
	// and one for duration (PWM)
	//
	// using minus two so that there is no need to staddle a gpio and
	// its timing across a page boundary.
	size_t i = 0;

	while (i < (_numTransferFramesPerPage - 2) &&
		currentPulseSegment < trackLength)
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
		size_t nextDurationIndex = (currentPulseSegment + 1) % trackLength;
		DmaTransfer* transferItem1 = transferBase0 + i;
		PwmData* item1 = (PwmData*)transferItem1;
		item1->Set(_pulseTracks[0].Timing[nextDurationIndex].Duration);
		i++;

		DmaTransfer* transferItem0 = transferBase0 + i;
		GpioData* item = (GpioData*)transferItem0;
		item->Set(_pulseTracks[0].Pin, _pulseTracks[0].Timing[currentPulseSegment].State);
		i++;

		currentPulseSegment++;
	}

	// Now adding the sync pin (GPIO) state change
	// which can generate an ISR depending on starting
	// state.
	DmaTransfer* isrItem0 = transferBase0 + i;

	GpioData* item = (GpioData*)isrItem0;
	item->Set(1 << _bufferSyncPin, PinState::High);

	return currentPulseSegment;
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
	cb0[cbIndex].DEST_AD = gpioAddress;
	cb0[cbIndex].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
	cb0[cbIndex].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);
	cb0[cbIndex].NEXTCONBK = (uint32_t)0;

	// Configure the second buffer to continue to the first one.
	//
	// If the second buffer runs out before you get here DMA will stop
	// and it also built-in safety to prevent run away DMA corruption.
	volatile DmaControlBlock* cb1 = (DmaControlBlock*)_controlBlock1Pages.back()->virtual_addr;
	cb1[_numControlBlocksPerPage - 2].NEXTCONBK = (uint32_t)((DmaControlBlock*)_controlBlock0Pages[0]->bus_addr);
}

void PulseGenerator::ConfigureControlBlocks1(uint32_t numControlBlocks)
{
	volatile DmaControlBlock* cb1;
	DmaControlBlock* cbPhys1;
	DmaTransfer* dmaTransferPhys1 = (DmaTransfer*)_buffer1Pages[0]->bus_addr;

	uint32_t gpioAddress = GPIO_BASE_BUS + offsetof(GpioRegisters, GPSET0);
	uint32_t pwmAddress = PWM_BASE_BUS + offsetof(PwmRegisters, RNG1);

	size_t cbIndex;
	for (size_t i = 0, j = 0; i < numControlBlocks; i++)
	{
		cbIndex = i % _numControlBlocksPerPage;
		if (cbIndex == 0)
		{
			cb1 = (DmaControlBlock*)_controlBlock1Pages[j]->virtual_addr;
			cbPhys1 = (DmaControlBlock*)_controlBlock1Pages[j]->bus_addr;
			j++;
		}

		//pace DMA through PWM
		cb1[cbIndex].TI = DMA_CB_TI_PERMAP_PWM |
			DMA_CB_TI_DEST_DREQ |
			DMA_CB_TI_SRC_INC |
			DMA_CB_TI_DEST_INC |
			DMA_CB_TI_NO_WIDE_BURSTS |
			DMA_CB_TI_TDMODE;
		cb1[cbIndex].SOURCE_AD = (uint32_t)(dmaTransferPhys1 + i); //The data written doesn't matter, but using the GPIO source will hopefully bring it into L2 for more deterministic timing of the next control block.
		cb1[cbIndex].DEST_AD = pwmAddress; //write to the FIFO
		cb1[cbIndex].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(4);
		cb1[cbIndex].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);
		if ((cbIndex + 1) % _numControlBlocksPerPage == 0)
		{
			cbPhys1 = (DmaControlBlock*)_controlBlock0Pages[j]->bus_addr;
			cb1[cbIndex].NEXTCONBK = (uint32_t)(cbPhys1);
		}
		else
		{
			cb1[cbIndex].NEXTCONBK = (uint32_t)(cbPhys1 + (cbIndex + 1));
		}

		i++;
		cbIndex = i % _numControlBlocksPerPage;
		if (cbIndex == 0)
		{
			cb1 = (DmaControlBlock*)_controlBlock0Pages[j]->virtual_addr;
		}

		// setup the gpio pin states.
		cb1[cbIndex].TI = DMA_CB_TI_SRC_INC |
			DMA_CB_TI_DEST_INC |
			DMA_CB_TI_NO_WIDE_BURSTS |
			DMA_CB_TI_TDMODE;
		cb1[cbIndex].SOURCE_AD = (uint32_t)(dmaTransferPhys1 + i);
		cb1[cbIndex].DEST_AD = gpioAddress;
		cb1[cbIndex].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
		cb1[cbIndex].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);

		if ((cbIndex + 1) % _numControlBlocksPerPage == 0)
		{
			cbPhys1 = (DmaControlBlock*)_controlBlock1Pages[j]->bus_addr;
			cb1[cbIndex].NEXTCONBK = (uint32_t)(cbPhys1);
		}
		else
		{
			cb1[cbIndex].NEXTCONBK = (uint32_t)(cbPhys1 + (cbIndex + 1));
		}
	}

	// This tiggers on the the buffer sync pin and may
	// (or may not depedning on start state) generate
	// on interrupt.
	cbIndex++;
	cb1[cbIndex].TI = DMA_CB_TI_SRC_INC |
		DMA_CB_TI_DEST_INC |
		DMA_CB_TI_NO_WIDE_BURSTS |
		DMA_CB_TI_TDMODE;
	cb1[cbIndex].SOURCE_AD = (uint32_t)(dmaTransferPhys1 + numControlBlocks);
	cb1[cbIndex].DEST_AD = gpioAddress;
	cb1[cbIndex].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
	cb1[cbIndex].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);
	cb1[cbIndex].NEXTCONBK = (uint32_t)0;

	// Configure the first buffer to continue to the second one.
	//
	// If the first buffer runs out before you get here DMA will stop
	// and it also built-in safety to prevent run away DMA corruption.
	volatile DmaControlBlock* cb0 = (DmaControlBlock*)_controlBlock0Pages.back()->virtual_addr;
	cb0[_numControlBlocksPerPage - 2].NEXTCONBK = (uint32_t)((DmaControlBlock*)_controlBlock1Pages[0]->bus_addr);
}

void PulseGenerator::NextChunk()
{
	if (_running == false)
		return;

	// TODO: If this is not fast enough may
	// need to use a member variable...
	PinState index = _gpio.ReadPin(_bufferSyncPin);

	if (index == PinState::Low)
	{
		uint32_t numPulseSegments0 = ConfigureBuffer0(_currentPulseSegment);

		if (numPulseSegments0 <= 0)
		{
			_running = false;
			return;
		}

		uint32_t numControlBlocks0 = (numPulseSegments0 - _currentPulseSegment) * CONTROL_BLOCK_PER_PULSE_SEGMENT;
		ConfigureControlBlocks0(numControlBlocks0);
		_currentPulseSegment = numPulseSegments0;
	}
	else
	{
		uint32_t numPulseSegments1 = ConfigureBuffer1(_currentPulseSegment);

		if (numPulseSegments1 <= 0)
		{
			_running = false;
			return;
		}

		uint32_t numControlBlocks1 = (numPulseSegments1 - _currentPulseSegment) * CONTROL_BLOCK_PER_PULSE_SEGMENT;
		ConfigureControlBlocks1(numControlBlocks1);
		_currentPulseSegment = numPulseSegments1;
	}
}