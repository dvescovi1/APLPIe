/*
 * Tests.h:
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

#include <stdint.h>

class Gpio;
class Clock;
class Pwm;
class Dma;
class FourDigitSevenSegmentDisplay;

class Test
{
public:
	static void WritePin(Gpio& gpio, int pin);
	static void ReadPin(Gpio& gpio, int pin);
	static void ClockEnableDisable(Clock& clock);
	static void PwmTest(Pwm& pwm);
	static void DmaMemoryToMemory(Dma& dma, uint8_t count);
	static void Display(FourDigitSevenSegmentDisplay& display);
	static void DmaGpio(Dma& dma, Gpio& gpio, int outPin0, int outPin1);
	static void DmaGpioPwmGated(Dma& dma, Pwm& pwm, Clock& clock, Gpio& gpio, int outPin0, int outPin1);
	static void DmaGpioDoubleBuffered(Dma& dma, Gpio& gpio, int outPin0, int outPin1);
};
