/*
 * Ideas and code snippets taken from wiringPi :
 *https ://projects.drogon.net/raspberry-pi/wiringpi/
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
	// initial support for debugging as root in poor in visual studio.
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


/*
 * Taken from wiring Pi:
 *
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void Peripheral::DelayMicrosecondsHard(uint32_t howLong)
{
	struct timeval tNow, tLong, tEnd;

	gettimeofday(&tNow, NULL);
	tLong.tv_sec = howLong / 1000000;
	tLong.tv_usec = howLong % 1000000;
	timeradd(&tNow, &tLong, &tEnd);

	while (timercmp(&tNow, &tEnd, < ))
		gettimeofday(&tNow, NULL);
}

/*
 * delay:
 *	Wait for some number of milliseconds
 *********************************************************************************
 */

void Peripheral::DelayMilliseconds(unsigned int howLong)
{
	struct timespec sleeper, dummy;

	sleeper.tv_sec = (time_t)(howLong / 1000);
	sleeper.tv_nsec = (long)(howLong % 1000) * 1000000;

	nanosleep(&sleeper, &dummy);
}

void Peripheral::DelayMicroseconds(uint32_t howLong)
{
	struct timespec sleeper;
	unsigned int uSecs = howLong % 1000000;
	unsigned int wSecs = howLong / 1000000;

	if (howLong == 0)
	{
		return;
	}
	else if (howLong < 100)
	{
		DelayMicrosecondsHard(howLong);
	}
	else
	{
		sleeper.tv_sec = wSecs;
		sleeper.tv_nsec = (long)(uSecs * 1000L);
		nanosleep(&sleeper, NULL);
	}
}
