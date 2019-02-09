/*
 * Clock.cpp:
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

#include "../Headers/Peripheral.h"
#include "../Headers/Clock.h"

#include "../Headers/ScreenLog.h"
#include "../Headers/hw-addresses.h"

Clock::Clock(const char* name) :
	PeripheralTemplate<ClockRegisters>(name, CLOCK_BASE)
{
}

void Clock::Disable()
{
	uint32_t value = Base->PWMCTL;
	value |= PWMCTL_PASSWD;
	value &= ~PWMCTL_ENAB;

	Base->PCMCTL = value; //disable clock
	do {} while ((Base->PWMCTL & PWMCTL_BUSY) == PWMCTL_BUSY); //wait for clock to deactivate
}

void Clock::Enable()
{
	uint32_t value = Base->PWMCTL;
	value |= PWMCTL_PASSWD;	
	value |= PWMCTL_ENAB;

	Base->PWMCTL = value; //enable clock
	do {} while ((Base->PWMCTL & PWMCTL_BUSY) == 0); //wait for clock to activate
}

void Clock::SetDivider(int divider)
{
	uint32_t value = Base->PWMCTL;
	value |= PWMCTL_PASSWD;
	value |= PWMDIV_DIVI(divider);
	value |= PWMCTL_SRC_PLLD;

	Base->PWMCTL = value;
}