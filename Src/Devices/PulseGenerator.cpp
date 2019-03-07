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
#include <unistd.h>

#include "../Headers/PulseGenerator.h"

struct GpioBufferFrame {
	//custom structure used for storing the GPIO buffer.
	//These BufferFrame's are DMA'd into the GPIO memory, potentially using the DmaEngine's Stride facility
	uint32_t gpset[2];
	uint32_t gpclr[2];
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
	_bufferSyncPin(bufferSyncPin),
	_numBufferPages(numBufferPages),
	_clock(clock)
{
	int pageSize = getpagesize();

	_numGpioFramesPerPage = pageSize / sizeof(GpioBufferFrame);
	_numControlBlocksPerPage = pageSize / sizeof(DmaControlBlock);
}

void PulseGenerator::SysInit(void)
{
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

	_buffer0Pages.clear();
	_buffer1Pages.clear();
	for (int i = 0; i < _numBufferPages; i++)
	{
		DmaMem_t* buffer0Page = dmaMemory.AllocDmaPage();
		_buffer0Pages.emplace_back(buffer0Page);

		DmaMem_t* buffer1Page = dmaMemory.AllocDmaPage();
		_buffer1Pages.emplace_back(buffer1Page);
	}

	uint32_t requiredControlPages = _numBufferPages *
		(_numGpioFramesPerPage / _numControlBlocksPerPage);

	_controlBlock0Pages.clear();
	_controlBlock1Pages.clear();
	for (int i = 0; i < requiredControlPages; i++)
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

	for (int i = 0; i < _numBufferPages; i++)
	{
		dmaMemory.FreeDmaPage(_buffer0Pages[i]);
		dmaMemory.FreeDmaPage(_buffer1Pages[i]);
		
	}
	_buffer0Pages.clear();
	_buffer1Pages.clear();

	for (int i = 0; i < _controlBlock0Pages.size(); i++)
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

#define CLOCK_DIV 80 //# to divide the NOMINAL_CLOCK_FREQ by before passing it to the PWM peripheral.

void PulseGenerator::Start()
{
	_clockCycle = 0;
	uint32_t numControlBlocks = ConfigureBuffer0(_clockCycle);

	_clock.PwmDisable();
	_clock.PwmSetDivider(CLOCK_DIV);
	_clock.PwmEnable();

	_pwm.Stop();
	_pwm.Start(BITS_PER_CLOCK);
	ConfigureControlBlocks0(numControlBlocks);

	int channel = 5;

	_dma.EnableChannel(channel);
	_dma.Stop(channel);

	uint32_t firstAddr = _controlBlock0Pages[0]->bus_addr;
	_dma.Start(channel, firstAddr);

	// TODO: remove or move outside of test.
	volatile DmaChannel* chan = _dma.Base->Chan + channel;
	do {} while (chan->CS & DMA_CS_ACTIVE);
}

uint32_t PulseGenerator::ConfigureBuffer0(uint32_t startingClock)
{
	GpioBufferFrame* gpioFrameBase0 = (GpioBufferFrame*)_buffer0Pages[0]->virtual_addr;
	uint32_t result = 0;

	for (int i = 0; i < _pulseTracks[0].Timing.size(); i++)
	{
		volatile GpioBufferFrame* item = gpioFrameBase0 + i;

		switch (_pulseTracks[0].Timing[i].State)
		{
		case PinState::High:
		{
			item->gpset[0] = _pulseTracks[0].Pin;
			item->gpclr[0] = 0;
		}
		break;
		case PinState::Low:
		{
			item->gpclr[0] = _pulseTracks[0].Pin;
			item->gpset[0] = 0;
		}
		break;
		case PinState::Unknown:
		default:
			break;
		}
		result++;
	}
	return result;
}

uint32_t PulseGenerator::ConfigureBuffer1(uint32_t startingClock)
{

}

void PulseGenerator::ConfigureControlBlocks0(uint32_t numControlBlocks)
{
	volatile DmaControlBlock *cb0 = (DmaControlBlock*)_controlBlock0Pages[0]->virtual_addr;
	volatile DmaControlBlock *cbPhys0 = (DmaControlBlock*)_controlBlock0Pages[0]->bus_addr;
	GpioBufferFrame* gpioFramePhys0 = (GpioBufferFrame*)_buffer0Pages[0]->bus_addr;

	for (int i = 0; i < numControlBlocks; i++)
	{
		cb0[i].TI = DMA_CB_TI_SRC_INC |
			DMA_CB_TI_DEST_INC |
			DMA_CB_TI_NO_WIDE_BURSTS |
			DMA_CB_TI_TDMODE;
		cb0[i].SOURCE_AD = (uint32_t)(gpioFramePhys0 + i);
		cb0[i].DEST_AD = GPIO_BASE_BUS + GPSET0OFFSET;
		cb0[i].TXFR_LEN = DMA_CB_TXFR_LEN_YLENGTH(2) | DMA_CB_TXFR_LEN_XLENGTH(8);
		cb0[i].STRIDE = DMA_CB_STRIDE_D_STRIDE(4) | DMA_CB_STRIDE_S_STRIDE(0);

		uint32_t* nextcb = (i + 1) < (numControlBlocks) ?
			(uint32_t*)(cbPhys0 + (i + 1)) :
			0; 
		cb0[i].NEXTCONBK = (uint32_t)nextcb;
	}
}

void PulseGenerator::ConfigureControlBlocks1(uint32_t numControlBlocks)
{
}