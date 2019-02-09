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

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "../Headers/ScreenLog.h"

struct PeripheralInfo
{	
	// MappedAddress MUST be first.  Intent is for specific
	// periperhals to create a union where this location
	// points to peripheral specific register maps.
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
	//
	// Using PeripheralTemplate will create the correct unions
	// for you just define your register map and base address
	// and pass it to PeripheralTemplate<RegisterMap>(BASE_ADDR)
	// and that class will handle the rest.
	volatile uint32_t* MappedAddress;
	uint32_t BaseAddress;
	int lengthBytes;
};

// This class defines the base interface for a peripheral at the 
// high level.  It can be used to init and uninit the peripherals
// at program run time.  Usually PeripheralTemplate is used to
// instantiate the periperal itself.
class Peripheral
{
private:
	static int _fdMem;	// requires sudo will use /dev/mem

protected:
	Peripheral(const char* name);

	const char* _name;
	static int _pageSize;
	
	void Map(int addr, int length, PeripheralInfo& info);
	void Unmap(PeripheralInfo& info);
	void WriteBit(volatile uint32_t *dest, uint32_t mask, uint32_t value);

public:
		virtual void SysInit() = 0;
	virtual void SysUninit() = 0;
};

// Class used to instantiate the peripherals.  Takes two
// template arguments the struct defining the register
// layout for the peripheral and the base address for the
// peripheral.
template <typename TRegisterStruct>
class PeripheralTemplate : public Peripheral
{

protected:
	void Map(int addr, int length);
	void Unmap();

public:
	PeripheralTemplate(const char* name, uint32_t baseAddress);

	// Union where the Peripheral classes will map
	// the base address to. By passing in the 
	// register map for your peripheral you will have
	// access to the reigters via derivedPeripheral.Base
	struct
	{
		union
		{
			PeripheralInfo Info;
			volatile TRegisterStruct* Base;
		};
	};

	virtual void SysInit();
	virtual void SysUninit();
};

template<typename TRegisterStruct>
PeripheralTemplate<TRegisterStruct>::PeripheralTemplate(const char* name, uint32_t baseAddress)
	: Peripheral(name)
{
	Info.BaseAddress = baseAddress;
}

template<typename TRegisterStruct>
void PeripheralTemplate<TRegisterStruct>::Map(int addr, int length)
{
	Peripheral::Map(addr, length, Info);
}

template<typename TRegisterStruct>
void PeripheralTemplate<TRegisterStruct>::Unmap()
{
	Peripheral::Unmap(Info);
}

template<typename TRegisterStruct>
void PeripheralTemplate<TRegisterStruct>::SysInit()
{
	int memSize = sizeof(TRegisterStruct);
	int pages = memSize / _pageSize + (memSize % _pageSize) > 0 ? 1 : 0;
	
	Map(Info.BaseAddress, pages * _pageSize);
	DBG("Peripheral mapped: %s Base: %p",
		_name,
		Info.MappedAddress);
}

template<typename TRegisterStruct>
void PeripheralTemplate<TRegisterStruct>::SysUninit()
{
	Unmap();
}
