/*
 * Delay.cpp:
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
 *
 * code snipets taken from WiringPi: 
 *	Arduino like Wiring library for the Raspberry Pi.
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *	Copyright (c) 2012-2019 Gordon Henderson
 *
 */

#include <stddef.h>
#include <time.h>
#include <sys/time.h>
#include "./Headers/Delay.h"

/*
 * Taken from wiring Pi:
 *
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100 µS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void Delay::MicrosecondsSpin(uint32_t howLong)
{
	timeval tNow, tLong, tEnd;

	gettimeofday(&tNow, NULL);
	tLong.tv_sec = howLong / 1000000;
	tLong.tv_usec = howLong % 1000000;
	timeradd(&tNow, &tLong, &tEnd);

	while (timercmp(&tNow, &tEnd, < ))
		gettimeofday(&tNow, NULL);
}

void Delay::Microseconds(uint32_t howLong)
{
	timespec sleeper;
	unsigned int uSecs = howLong % 1000000;
	unsigned int wSecs = howLong / 1000000;

	if (howLong == 0)
	{
		return;
	}
	else if (howLong < 100)
	{
		MicrosecondsSpin(howLong);
	}
	else
	{
		sleeper.tv_sec = wSecs;
		sleeper.tv_nsec = (long)(uSecs * 1000L);
		nanosleep(&sleeper, NULL);
	}
}

void Delay::Milliseconds(unsigned int howLong)
{
	timespec sleeper, dummy;

	sleeper.tv_sec = (time_t)(howLong / 1000);
	sleeper.tv_nsec = (long)(howLong % 1000) * 1000000;

	nanosleep(&sleeper, &dummy);
}
