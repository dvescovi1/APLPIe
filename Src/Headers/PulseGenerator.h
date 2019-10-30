/*
 * PulseGenerator.h:
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
#pragma once

#include "Device.h"
#include "Clock.h"
#include "Gpio.h"
#include "Dma.h"
#include "DmaMemory.h"
#include "Pwm.h"

#include <vector>

// To divide the NOMINAL_CLOCK_FREQ by before passing it to
// the PWM peripheral.  Trying to nominally time the PWM
// such that 32 clocks is 1 탎ec or 31.25 MHz clock.
// Clocks to write a FIFO byte is determined by RNG1 of
// the PWM.  Not documented but I have found that RNG1
// takes  a few clocks to take effect so the duration field
// set by RNG1 is one pulse segment ahead of its state...
//
// Don't know if a full word is required but that is why I
// chose 32 bits for 1 탎ec just in case...
//
// This is the divider which should be passed to
// Clock::PwmSetDivider(int divider)
//
// Also assumes base clock is (running at 500MHz undivided)
#define CLOCK_DIV 16

// Use this to "dial in" the timing to you desitred accuracy.
//
// i.e using these default settings I get 31.25 MHz base clock
// rate.  In an ideal world this would mean a RING1 of 312
// will produce pulse segments of 10 탎ec.  On my RPI I measured
// a pulse segment duration of 10.45 탎ec. Similarly, a 624
// value for RNG1 produced 20.90 탎ec.  To get exactly 10 탎ec
// I need 4 fewer PWM clocks for a difference of 0.4 MHz.
// I can adjust the ideal timing to actual conditions with
// the following constant..
// Well... Then after upgrading to the same PI to strech the
// timing was off.  Therefore I'm just introducing a slope
// and offset so that timing can be corrected.
// After stretch upgrade I found the following data:
// Measured	 desired 탎ec
// 172	     58
// 293	     100
// 1488	     500
// Leading to these claibration constants: (your setup may
// require a different constant if 탎ec timing is required
// for your application.
//
// Ran it on a second PI and things operate as expected??
#define CLOCK_CALIBRATION_OFFSET 0.0f
#define CLOCK_CALIBRATION_SLOPE  1.0f

// Use this macro when you know you want a pulse segment
// that should have a duration of a known micro seconds.
#define MICROSEC_TO_RNG1(x) (int)((float) (CLOCK_CALIBRATION_SLOPE * x * (500.0f /* MHz*/ / CLOCK_DIV) + CLOCK_CALIBRATION_OFFSET))

// one is for the GPIO (PinState)
// one is for the PWM (duration)
#define CONTROL_BLOCK_PER_PULSE_SEGMENT 2

struct Pulse
{
	uint32_t PinMask;
	PinState State;
	uint32_t Duration; // Duration is defined by the clocks current clock frequency
};

struct PulseTrain
{	
	std::vector<Pulse> Timing;
	bool Repeat = false;
	bool Valid = false;
	uint64_t OuputCount = 0;

	PulseTrain()
	{
	}

	void Add(uint32_t pinMask, PinState pinState, uint32_t duration)
	{
		Pulse segment;
		segment.PinMask = pinMask;
		segment.State = pinState;
		segment.Duration = duration;
		Add(segment);
	}
	
	void Add(Pulse& pulse)
	{
		Timing.emplace_back(pulse);
	}

	void SetRepeat(bool repeat)
	{
		Repeat = repeat;
	}
};

struct DMABufferInfo
{
	uint32_t NextPulseSegment = 0;
	uint32_t NumberOfUsedBuffers = 0;
	bool PulseComplete = false;
};

class PulseGenerator : public Device
{
private:
	static const int _channel;

	Gpio& _gpio;
	Dma& _dma;
	Pwm& _pwm;
	Clock& _clock;
	uint32_t _bufferSyncPin;
	uint8_t _numTransferPages;
	uint32_t _numTransferFramesPerPage;
	uint32_t _numControlBlocksPerPage;
	uint32_t _currentPulseSegment;
	volatile bool _running;

	std::vector<PulseTrain*> _pulseTracks;
	std::vector<DmaMem_t*> _buffer0Pages;
	std::vector<DmaMem_t*> _buffer1Pages;

	std::vector<DmaMem_t*> _controlBlock0Pages;
	std::vector<DmaMem_t*> _controlBlock1Pages;

	static void SyncPinIsr(void* arg);

	DMABufferInfo ConfigureBuffer0(uint32_t startingPulseSegment);
	DMABufferInfo ConfigureBuffer1(uint32_t startingPulseSegment);

	void ConfigureControlBlocks0(uint32_t numControlBlocks);
	void ConfigureControlBlocks1(uint32_t numControlBlocks);

	void NextChunk();

public:
	PulseGenerator(Gpio& gpio, Dma& dma, Pwm& pwm, Clock& clock, uint32_t bufferSyncPin, uint8_t numBufferPages);
	void virtual SysInit(void);
	void virtual SysUninit(void);
	void Add(PulseTrain& pulseTrain);
	void Clear();
	void Start();
	void Stop();
	bool IsRunning();
	void WriteSyncPinState(PinState state);
};