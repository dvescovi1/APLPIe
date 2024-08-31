/*
 * Pwm.h:
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
#ifndef PWM_H
#define PWM_H


#include <stdint.h>

#include "../Headers/Peripheral.h"
#include "../Headers/hw-addresses.h"

 //# of bits to be used in each PWM cycle. Effectively acts
// as a clock divisor for us, since the PWM clock is in bits/second
#define BITS_PER_CLOCK 10 

// The DMA transaction is paced through the PWM FIFO.
// The PWM FIFO consumes 1 word every N uS (set in clock settings).
// Once the fifo has fewer than PWM_FIFO_SIZE words available,
// it will request more data from DMA.
// Thus, a high buffer length will be more resistant to clock drift,
// but may occasionally request multiple frames in a short succession
// (faster than FRAME_PER_SEC) in the presence of bus contention,
// whereas a low buffer length will always space frames
// AT LEAST 1/FRAMES_PER_SEC seconds apart, but may experience clock drift.
#define PWM_FIFO_SIZE 1 

#define PWM_CTL_USEFIFO2 (1<<13)
#define PWM_CTL_REPEATEMPTY2 (1<<10)
#define PWM_CTL_ENABLE2 (1<<8)
#define PWM_CTL_CLRFIFO (1<<6)
#define PWM_CTL_USEFIFO1 (1<<5)
#define PWM_CTL_REPEATEMPTY1 (1<<2)
#define PWM_CTL_ENABLE1 (1<<0)

#define PWM_STA_BUSERR (1<<8)
#define PWM_STA_GAPERRS (0xf << 4)
#define PWM_STA_FIFOREADERR (1<<3)
#define PWM_STA_FIFOWRITEERR (1<<2)
#define PWM_STA_ERRS PWM_STA_BUSERR | PWM_STA_GAPERRS | PWM_STA_FIFOREADERR | PWM_STA_FIFOWRITEERR

#define PWM_DMAC_EN (1<<31)
#define PWM_DMAC_PANIC(P) (((P)&0xff)<<8)
#define PWM_DMAC_DREQ(D) (((D)&0xff)<<0)

#define PWM_CTL  0x00000000 //control register
#define PWM_STA  0x00000004 //status register
#define PWM_DMAC 0x00000008 //DMA control register
#define PWM_RNG1 0x00000010 //channel 1 range register (# output bits to use per sample)
#define PWM_DAT1 0x00000014 //channel 1 data
#define PWM_FIF1 0x00000018 //channel 1 fifo (write to this register to queue an output)
#define PWM_RNG2 0x00000020 //channel 2 range register
#define PWM_DAT2 0x00000024 //channel 2 data

struct PwmRegisters
{
	uint32_t CTL; // PWM Control 
	uint32_t STA; // PWM Status 
	uint32_t DMAC; // PWM DMA Configuration
	uint32_t Reserved0;
	uint32_t RNG1; // PWM Channel 1 Range 
	uint32_t DAT1; // PWM Channel 1 Data 
	uint32_t FIF1; // PWM FIFO Input
	uint32_t Reserved1;
	uint32_t RNG2; // PWM Channel 2 Range 
	uint32_t DAT2; // PWM Channel 2 Data
};

class Pwm : public PeripheralTemplate<PwmRegisters>
{

public:
	Pwm(const char* name);
	void Stop();
	void Start(int bitsToClock);
};

#endif /* PWM_H */
