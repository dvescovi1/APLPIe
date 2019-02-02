#pragma once
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
