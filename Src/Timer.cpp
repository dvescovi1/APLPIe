/*
 * Timer.cpp:
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
#include "./Headers/Timer.h"

static void timerHandler(int sig, siginfo_t *si, void *uc)
{	
	Timer* timer;
	timer = (Timer*) si->si_value.sival_ptr;

	timer->TimesUp();
}

int Timer::Start(char const *name, std::function<void()> userFunc, int expireMS, int intervalMS)
{
	sigevent te;
	itimerspec its;
	struct sigaction sa;
	int sigNo = SIGRTMIN + 1;

	/* Set up signal handler. */
	sa.sa_flags = SA_SIGINFO;
	sa.sa_sigaction = timerHandler;
	sigemptyset(&sa.sa_mask);
	if (sigaction(sigNo, &sa, NULL) == -1)
	{
		fprintf(stderr, "%s: Failed to setup signal handling for %s.\n", "Timer.cpp", name);
		return(-1);
	}

	_userFunc = userFunc;

	/* Set and enable alarm */
	te.sigev_notify = SIGEV_SIGNAL;
	te.sigev_signo = sigNo;
	te.sigev_value.sival_ptr = this;
	timer_create(CLOCK_REALTIME,
		&te,
		&_timerID);

	unsigned long intervalns = (unsigned long) intervalMS * 1000000UL;
	unsigned long expirens = (unsigned long) expireMS * 1000000UL;

	its.it_interval.tv_sec = intervalns / 1000000000UL;
	its.it_interval.tv_nsec = intervalns % 1000000000UL;
	its.it_value.tv_sec = expirens / 1000000000UL;
	its.it_value.tv_nsec = expirens % 1000000000UL;
	timer_settime(_timerID,
		0,
		&its,
		NULL);

	return(0);
}

void Timer::TimesUp()
{
	if (_userFunc != NULL)
	{
		_userFunc();
	}
}