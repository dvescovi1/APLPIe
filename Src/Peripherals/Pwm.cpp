/*
 * Gpio.cpp:
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
#include <errno.h>
#include <stdio.h> 
#include <unistd.h>

#include "../Headers/Delay.h"
#include "../Headers/Pwm.h"
#include "../Headers/hw-addresses.h"

Pwm::Pwm(const char* name) :
	PeripheralTemplate<PwmRegisters>(name, PWM_BASE)
{
}

void Pwm::Stop()
{
	volatile PwmRegisters* base = Base;

	base->DMAC = 0;
	base->CTL |= PWM_CTL_CLRFIFO;
	Delay::Microseconds(100);

	base->STA = PWM_STA_ERRS;
	Delay::Microseconds(100);
}

void Pwm::Start(int bitsToClock)
{
	volatile PwmRegisters* base = Base;

	// DREQ is activated at queue < PWM_FIFO_SIZE
	base->DMAC = PWM_DMAC_EN | PWM_DMAC_DREQ(PWM_FIFO_SIZE) | PWM_DMAC_PANIC(PWM_FIFO_SIZE);

	//used only for timing purposes; #writes to PWM FIFO/sec = PWM CLOCK / RNG1
	base->RNG1 = bitsToClock; 
	base->CTL = PWM_CTL_REPEATEMPTY1 | PWM_CTL_ENABLE1 | PWM_CTL_USEFIFO1;
}