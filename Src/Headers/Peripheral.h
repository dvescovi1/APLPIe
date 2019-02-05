/*
 * Periperal.h:
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

struct PeripheralInfo
{	
	// This MUST be first.  Intent is for specific periperhals to
	// create a union where this location points to peripheral
	// specific register maps.
	// 
	// Example from the Gpio periphral:
	//typedef struct
	//{
	//	union
	//	{
	//		PeripheralInfo info;
	//		volatile GpioRegisters* Base;
	//	};
	//} GpioInfo;
	volatile uint32_t* MappedAddress;
	uint32_t* BaseAddress;
	int lengthBytes;
};

class Peripheral
{
private:
	static int _fdMem;	
	const char* _name;	

protected:
	static int _pageSize;

	void Map(int addr, int length, PeripheralInfo& info);
	void Unmap(PeripheralInfo& info);
	void WriteBit(volatile uint32_t *dest, uint32_t mask, uint32_t value);
	
public:
	Peripheral(const char* name);
	virtual void SysInit() = 0;
	virtual void SysUninit() = 0;	
};
