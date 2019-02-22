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

void Clock::PwmDisable()
{
	volatile ClockRegisters* base = Base;
	volatile uint32_t value = base->PWMCTL;
	value |= PWM_PASSWD;
	value &= ~PWMCTL_ENAB;

	base->PWMCTL = value; //disable clock
	do
	{
		//wait for clock to deactivate
	} while ((base->PWMCTL & PWMCTL_BUSY) == PWMCTL_BUSY);
	
}

void Clock::PwmEnable()
{
	volatile ClockRegisters* base = Base;
	volatile uint32_t value = base->PWMCTL;
	value |= PWM_PASSWD;
	value |= PWMCTL_ENAB;

	base->PWMCTL = value; //enable clock
	do
	{
		//wait for clock to activate
	} while ((base->PWMCTL & PWMCTL_BUSY) == 0);
}

void Clock::PwmSetDivider(int divider)
{
	volatile ClockRegisters* base = Base;
	volatile uint32_t dividerValue = PWM_PASSWD;

	//configure clock divider (running at 500MHz undivided)
	dividerValue |= PWMDIV_DIVI(divider);
	base->PWMDIV = dividerValue;
	
	volatile uint32_t ctlValue = base->PWMCTL;
	ctlValue |= PWM_PASSWD;

	//source 500MHz base clock, no MASH.
	ctlValue |= PWMCTL_SRC_PLLD;
	base->PWMCTL = ctlValue;
}