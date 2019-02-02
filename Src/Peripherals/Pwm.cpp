#include <errno.h>
#include <stdio.h> 
#include <unistd.h>

#include "../Headers/Pwm.h"
#include "../Headers/ScreenLog.h"

#include "../Headers/hw-addresses.h"

Pwm::Pwm(const char* name) :
	Peripheral(name)
{
}

void Pwm::SysInit()
{
	int memSize = sizeof(PwmRegisters);
	int pages = memSize / _pageSize + (memSize % _pageSize) > 0 ? 1 : 0;

	Map(PWM_BASE, pages * _pageSize, _pwm.info);
	DBG("Pwm Base: %p", _pwm.info.MappedAddress);
}

void Pwm::SysUninit()
{
	Unmap(_pwm.info);
}

void Pwm::TestExample()
{
	_pwm.Base->DMAC = 0; //disable DMA
	_pwm.Base->CTL |= PWM_CTL_CLRFIFO; //clear pwm
	usleep(100);

	_pwm.Base->STA = PWM_STA_ERRS; //clear PWM errors
	usleep(100);

	_pwm.Base->DMAC = PWM_DMAC_EN | PWM_DMAC_DREQ(PWM_FIFO_SIZE) | PWM_DMAC_PANIC(PWM_FIFO_SIZE); //DREQ is activated at queue < PWM_FIFO_SIZE
	_pwm.Base->RNG1 = BITS_PER_CLOCK; //used only for timing purposes; #writes to PWM FIFO/sec = PWM CLOCK / RNG1
	_pwm.Base->CTL = PWM_CTL_REPEATEMPTY1 | PWM_CTL_ENABLE1 | PWM_CTL_USEFIFO1;
}