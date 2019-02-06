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

#include "../Headers/Pwm.h"
#include "../Headers/ScreenLog.h"

#include "../Headers/hw-addresses.h"

Pwm::Pwm(const char* name) :
	Peripheral(name)
{
}

void Pwm::SysInit()
{
	int memSize = sizeof(PwmRegisters);
	int pages = memSize / _pageSize + (memSize % _pageSize) > 0 ? 1 : 0;

	Map(PWM_BASE, pages * _pageSize, Base.info);
	DBG("Pwm Base: %p", Base.info.MappedAddress);
}

void Pwm::SysUninit()
{
	Unmap(Base.info);
}