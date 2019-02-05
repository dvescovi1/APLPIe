/*
 * Periperal.cpp:
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
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "../Headers/Peripheral.h"
#include "../Headers/ScreenLog.h"

int Peripheral::_fdMem = -1;
int Peripheral::_pageSize = getpagesize();

Peripheral::Peripheral(const char* name)
{
	if (name == NULL)
	{
		DBG("Argument: Name cannot be null");
		exit(1);
	}
	_name = name;
}

void Peripheral::Map(int addr, int length, PeripheralInfo& info)
{
	// initial support for debugging as root is poor in visual studio.
	// In order to remotely debug this function or any code needing
	// root access. Maybe in the future we can avoid this hack...
	//
	// steps are:
	//
	// cd /usr/bin
	//
	// sudo mv gdb gdborig
	//
	// Now you need to create a bash script named gdb with following content.
	//
	// sudo nano gdb
	//
	// Content of the bash is; (The -E is important!!! visual studio will send
	// paths relative to the logon users home directory)
	//
	// #!/bin/sh
	// sudo -E gdborig $@
	//
	// Finally, make the script runnable.
	//
	// sudo chmod 0755 gdb
	if (_fdMem == -1)
	{
		_fdMem = open("/dev/mem", O_RDWR | O_SYNC);
		if (_fdMem < 0)
		{
			DBG("Failed to open /dev/mem (did you remember to run as root?)\n");
			DBG("%s:", strerror(errno));
			exit(1);
		}
	}

	void *mapped = mmap(NULL,
		length,
		PROT_READ | PROT_WRITE, MAP_SHARED,
		_fdMem,
		addr);
	info.BaseAddress = (uint32_t*)addr;
	info.MappedAddress = (uint32_t*)mapped;
	info.lengthBytes = length;

	//now, *mapped = memory at physical address of addr.
	if (mapped == MAP_FAILED)
	{
		DBG("failed to map memory (did you remember to run as root?)\n");
		DBG("%s:", strerror(errno));
		exit(1);
	}

	DBG("%s mapped: Physical Base: %p Mapped: %p\n", _name,
		info.BaseAddress,
		info.MappedAddress);
}

void Peripheral::Unmap(PeripheralInfo& info)
{
	munmap((void*)info.MappedAddress, info.lengthBytes);
}

void Peripheral::WriteBit(volatile uint32_t *dest, uint32_t mask, uint32_t value)
{
	//set bits designated by (mask) at the address (dest) to (value), without affecting the other bits
	//eg if x = 0b11001100
	//  writeBitmasked(&x, 0b00000110, 0b11110011),
	//  then x now = 0b11001110
	uint32_t currentValue = *dest;
	uint32_t newValue = (currentValue & (~mask)) | (value & mask);
	*dest = newValue;
}