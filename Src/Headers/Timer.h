/*
 * Timer.h:
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
#ifndef TIMER_H
#define TIMER_H


#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <functional>


class Timer
{
private:
	timer_t _timerID;
	
	std::function<void()> _userFunc;

public:
	int Start(char const *name, std::function<void()> userFunc, int expireMS, int intervalMS);
	void TimesUp();

};

#endif /* TIMER_H */