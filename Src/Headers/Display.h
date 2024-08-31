/*
 * Display.h:
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
#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>

#include "Device.h"

class Gpio;

struct CharacterDisplayPins 
{
	CharacterDisplayPins(uint32_t charPin0,
		uint32_t charPin1,
		uint32_t charPin2,
		uint32_t charPin3,
		uint32_t charPin4,
		uint32_t charPin5,
		uint32_t charPin6,
		uint32_t charPin7)
	{
		_charPin0 = charPin0;
		_charPin1 = charPin1;
		_charPin2 = charPin2;
		_charPin3 = charPin3;
		_charPin4 = charPin4;
		_charPin5 = charPin5;
		_charPin6 = charPin6;
		_charPin7 = charPin7;

		// Mask for the display i/o pins
		//                      Bit0      |       Bit 1      |       Bit 2      |
		_characterMask = (1 << _charPin0) | (1 << _charPin1) | (1 << _charPin2) |
		//          Bit 3    |       Bit 4      |       Bit 5      |
			(1 << _charPin3) | (1 << _charPin4) | (1 << _charPin5) |
		//          Bit 6    |       Bit 7      |
			(1 << _charPin6) | (1 << _charPin7);
	}

	union
	{
		uint32_t _charPins[8];
		struct
		{
			uint32_t _charPin0;
			uint32_t _charPin1;
			uint32_t _charPin2;
			uint32_t _charPin3;
			uint32_t _charPin4;
			uint32_t _charPin5;
			uint32_t _charPin6;
			uint32_t _charPin7;
		};
	};
	uint32_t _characterMask;
};

class FourDigitSevenSegmentDisplay : public Device
{
private:
	Gpio& _gpio;
	int _pin0;
	int _pin1;
	int _pin2;
	int _pin3;

	CharacterDisplayPins& _characterPins;
	
	uint32_t _datBuf[4] = { 0,0,0,0 };

	static uint32_t const unmappedSegCode[10];
	uint32_t mappedSegCode[10];

	void RemapSegCodes();

public:
	FourDigitSevenSegmentDisplay(Gpio& gpio, int pin0, int pin1, int pin2, int pin3, CharacterDisplayPins& characterMask);
	void virtual SysInit(void);
	void virtual SysUninit(void);
	void SetDisplayValue(int value);
	void Display(void);
};

#endif /* DISPLAY_H */