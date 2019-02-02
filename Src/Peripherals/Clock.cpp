#include <errno.h>
#include <stdio.h> 

#include "../Headers/Peripheral.h"
#include "../Headers/Clock.h"

#include "../Headers/ScreenLog.h"
#include "../Headers/hw-addresses.h"

Clock::Clock(const char* name) :
	Peripheral(name)
{
}

void Clock::SysInit()
{
	int memSize = sizeof(ClockRegisters);
	int pages = memSize / _pageSize + (memSize % _pageSize) > 0 ? 1 : 0;

	Map(CLOCK_BASE, pages * _pageSize, _clock.info);
	DBG("Clock Base: %p", _clock.info.MappedAddress);
}

void Clock::SysUninit()
{
	Unmap(_clock.info);
}

void Clock::Disable()
{
	uint32_t value = _clock.Base->PWMCTL;
	value |= PWMCTL_PASSWD;
	value &= ~PWMCTL_ENAB;

	_clock.Base->PCMCTL = value; //disable clock
	do {} while ((_clock.Base->PWMCTL & PWMCTL_BUSY) == PWMCTL_BUSY); //wait for clock to deactivate
}

void Clock::Enable()
{
	uint32_t value = _clock.Base->PWMCTL;
	value |= PWMCTL_PASSWD;	
	value |= PWMCTL_ENAB;

	_clock.Base->PWMCTL = value; //enable clock
	do {} while ((_clock.Base->PWMCTL & PWMCTL_BUSY) == 0); //wait for clock to activate
}

void Clock::SetDivider(int divider)
{
	uint32_t value = _clock.Base->PWMCTL;
	value |= PWMCTL_PASSWD;
	value |= PWMDIV_DIVI(divider);
	value |= PWMCTL_SRC_PLLD;

	_clock.Base->PWMCTL = value;
}

void Clock::TestExample()
{
	Disable();
//	Enable();
}